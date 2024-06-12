// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "pathtracer_device.h"

#include "../common/lights/ambient_light.cpp"
#include "../common/lights/directional_light.cpp"
#include "../common/lights/point_light.cpp"
#include "../common/lights/quad_light.cpp"
#include "../common/lights/spot_light.cpp"

namespace embree {

#define USE_ARGUMENT_CALLBACKS 1

//RTC_SYCL_INDIRECTLY_CALLABLE void occlusionFilterOpaque(const RTCFilterFunctionNArguments* args);
RTC_SYCL_INDIRECTLY_CALLABLE void occlusionFilterHair(const RTCFilterFunctionNArguments* args);

#undef TILE_SIZE_X
#undef TILE_SIZE_Y

#define TILE_SIZE_X 4
#define TILE_SIZE_Y 4

#define FIXED_SAMPLING 0

#define FIXED_EDGE_TESSELLATION_VALUE 4

#define ENABLE_FILTER_FUNCTION 0

#define MAX_EDGE_LEVEL 128.0f
#define MIN_EDGE_LEVEL   4.0f
#define LEVEL_FACTOR    64.0f

TutorialData data;
extern "C" int g_animation_mode;

bool g_subdiv_mode = false;
unsigned int keyframeID = 0;

#if defined(EMBREE_SYCL_TUTORIAL) && !defined(EMBREE_SYCL_RT_SIMULATION) && defined(USE_SPECIALIZATION_CONSTANTS)
const static sycl::specialization_id<RTCFeatureFlags> rtc_feature_mask(RTC_FEATURE_FLAG_ALL);
#endif
RTCFeatureFlags g_used_features = RTC_FEATURE_FLAG_NONE;

////////////////////////////////////////////////////////////////////////////////
//                               Lights                                       //
////////////////////////////////////////////////////////////////////////////////

Light_SampleRes Lights_sample(const Light* self,
                              const DifferentialGeometry& dg, /*! point to generate the sample for >*/
                              const Vec2f s)                /*! random numbers to generate the sample >*/
{
  TutorialLightType ty = self->type;
  switch (ty) {
  case LIGHT_AMBIENT    : return AmbientLight_sample(self,dg,s);
  case LIGHT_POINT      : return PointLight_sample(self,dg,s);
  case LIGHT_DIRECTIONAL: return DirectionalLight_sample(self,dg,s);
  case LIGHT_SPOT       : return SpotLight_sample(self,dg,s);
  case LIGHT_QUAD       : return QuadLight_sample(self,dg,s);
  default: {
    Light_SampleRes res;
    res.weight = Vec3fa(0,0,0);
    res.dir = Vec3fa(0,0,0);
    res.dist = 0;
    res.pdf = inf;
    return res;
  }
  }
}
  
Light_EvalRes Lights_eval(const Light* self,
                          const DifferentialGeometry& dg,
                          const Vec3fa& dir)
{
  TutorialLightType ty = self->type;
  switch (ty) {
  case LIGHT_AMBIENT     : return AmbientLight_eval(self,dg,dir);
  case LIGHT_POINT       : return PointLight_eval(self,dg,dir);
  case LIGHT_DIRECTIONAL : return DirectionalLight_eval(self,dg,dir);
  case LIGHT_SPOT        : return SpotLight_eval(self,dg,dir);
  case LIGHT_QUAD        : return QuadLight_eval(self,dg,dir);
  default: {
    Light_EvalRes res;
    res.value = Vec3fa(0,0,0);
    res.dist = inf;
    res.pdf = 0.f;
    return res;
  }
  }
}


////////////////////////////////////////////////////////////////////////////////
//                               Scene                                        //
////////////////////////////////////////////////////////////////////////////////

/* scene data */
RTCScene g_scene = nullptr;

/* accumulation buffer */
Vec3fa g_accu_vx;
Vec3fa g_accu_vy;
Vec3fa g_accu_vz;
Vec3fa g_accu_p;

#if 0
void device_key_pressed_handler(int key)
{
  if (key == 32  /* */) g_animation = !g_animation;
  if (key == 110 /*n*/) { g_use_smooth_normals = !g_use_smooth_normals; g_changed = true; }
  else device_key_pressed_default(key);
}
#endif

void assignShaders(ISPCGeometry* geometry)
{
#if ENABLE_FILTER_FUNCTION
  RTCGeometry geom = geometry->geometry;
  if (geometry->type == SUBDIV_MESH ||
      geometry->type == TRIANGLE_MESH ||
      geometry->type == QUAD_MESH ||
      geometry->type == GRID_MESH)
  {
    //rtcSetGeometryOccludedFilterFunction(geom,data.occlusionFilterOpaque);
    rtcSetGeometryEnableFilterFunctionFromArguments(geom,false);
  }
  else if (geometry->type == CURVES) {
    rtcSetGeometryOccludedFilterFunction(geom,data.occlusionFilterHair);
    rtcSetGeometryEnableFilterFunctionFromArguments(geom,true);
  }
#endif
}

typedef ISPCInstance* ISPCInstance_ptr;
typedef ISPCGeometry* ISPCGeometry_ptr;

RTCScene convertScene(ISPCScene* scene_in)
{
  for (unsigned int i=0; i<scene_in->numGeometries; i++)
  {
    ISPCGeometry* geometry = scene_in->geometries[i];
    if (geometry->type == SUBDIV_MESH) {
      g_subdiv_mode = true; break;
    }
  }

  assignShadersFunc = assignShaders;

  RTCScene scene_out = ConvertScene(g_device, g_ispc_scene, RTC_BUILD_QUALITY_MEDIUM, RTC_SCENE_FLAG_NONE, &g_used_features);
#if ENABLE_FILTER_FUNCTION
#if USE_ARGUMENT_CALLBACKS
  g_used_features = (RTCFeatureFlags)(g_used_features | RTC_FEATURE_FLAG_FILTER_FUNCTION_IN_ARGUMENTS);
#else
  g_used_features = (RTCFeatureFlags)(g_used_features | RTC_FEATURE_FLAG_FILTER_FUNCTION_IN_GEOMETRY);
#endif
#endif

  /* commit changes to scene */
  //progressStart();
  //rtcSetSceneProgressMonitorFunction(scene_out,progressMonitor,nullptr);
  rtcCommitScene (scene_out);
  //rtcSetSceneProgressMonitorFunction(scene_out,nullptr,nullptr);
  //progressEnd();

  return scene_out;
} // convertScene

Vec3fa renderPixelFunction(const TutorialData& data, float x, float y, RandomSampler& sampler, const ISPCCamera& camera, RayStats& stats, const RTCFeatureFlags features)
{
  /* radiance accumulator and weight */
  Vec3fa L = Vec3fa(0.0f);
  float time = RandomSampler_get1D(sampler);

  /* initialize ray */
  Ray ray(Vec3fa(camera.xfm.p),
                     Vec3fa(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)),0.0f,inf,time);

  /* intersect ray with scene */
  RayQueryContext context;
  InitIntersectionContext(&context);
  context.tutorialData = (void*)&data;

  RTCIntersectArguments args;
  rtcInitIntersectArguments(&args);
  args.context = &context.context;
  args.feature_mask = features;
#if USE_ARGUMENT_CALLBACKS && ENABLE_FILTER_FUNCTION
  args.filter = nullptr;
#endif
  
  rtcIntersect1(data.scene, RTCRayHit_(ray), &args);

  RayStats_addRay(stats);
  if (ray.tail == 0) {
    return Vec3fa(0.0f);
  }

  // bubble sort the hits
  for (int i = 0; i < ray.tail; i++)
  {
    for (int j = i + 1; j < ray.tail; j++)
    {
      if (ray.tfars[i] > ray.tfars[j])
      {
        std::swap(ray.tfars[i], ray.tfars[j]);
        std::swap(ray.colors[i], ray.colors[j]);
        std::swap(ray.alpha[i], ray.alpha[j]);
      }
    }
  }

  Vec3fa accumeCol = Vec3fa(0.0f, 0.0f, 0.0f);
  float sumAlpha = 0.0f;
  // volume raycasting calculation
  for (int i = 0; i < ray.tail; i++)
  {
    if (1.0f <= sumAlpha)
    {
      break;
    }
    auto colorSample = ray.colors[i];
    float currentAlpha = (1.0f - sumAlpha) * ray.alpha[i];
    accumeCol += colorSample * currentAlpha;
    sumAlpha += currentAlpha;
  }
  L += accumeCol;

  return L;
}

/* task that renders a single screen tile */
void renderPixelStandard(const TutorialData& data,
                         int x, int y,
                         int* pixels,
                         const unsigned int width,
                         const unsigned int height,
                         const float time,
                         const ISPCCamera& camera,
                         RayStats& stats,
                         const RTCFeatureFlags features)
{
  RandomSampler sampler;

  Vec3fa L = Vec3fa(0.0f);

  for (int i=0; i<data.spp; i++)
  {
    RandomSampler_init(sampler, x, y, data.accu_count*data.spp+i);

    /* calculate pixel color */
    float fx = x + RandomSampler_get1D(sampler);
    float fy = y + RandomSampler_get1D(sampler);
    L = L + renderPixelFunction(data,fx,fy,sampler,camera,stats,features);
  }
  L = L/(float)data.spp;

  /* write color to framebuffer */
  Vec3ff accu_color = data.accu[y*width+x] + Vec3ff(L.x,L.y,L.z,1.0f); data.accu[y*width+x] = accu_color;
  float f = rcp(max(0.001f,accu_color.w));
  unsigned int r = (unsigned int) (255.01f * clamp(accu_color.x*f,0.0f,1.0f));
  unsigned int g = (unsigned int) (255.01f * clamp(accu_color.y*f,0.0f,1.0f));
  unsigned int b = (unsigned int) (255.01f * clamp(accu_color.z*f,0.0f,1.0f));
  pixels[y*width+x] = (b << 16) + (g << 8) + r;
}

/* task that renders a single screen tile */
void renderTileTask (int taskIndex, int threadIndex, int* pixels,
                         const unsigned int width,
                         const unsigned int height,
                         const float time,
                         const ISPCCamera& camera,
                         const int numTilesX,
                         const int numTilesY)
{
  const int t = taskIndex;
  const unsigned int tileY = t / numTilesX;
  const unsigned int tileX = t - tileY * numTilesX;
  const unsigned int x0 = tileX * TILE_SIZE_X;
  const unsigned int x1 = min(x0+TILE_SIZE_X,width);
  const unsigned int y0 = tileY * TILE_SIZE_Y;
  const unsigned int y1 = min(y0+TILE_SIZE_Y,height);

  for (unsigned int y=y0; y<y1; y++) for (unsigned int x=x0; x<x1; x++)
  {
    renderPixelStandard(data,x,y,pixels,width,height,time,camera,g_stats[threadIndex],RTC_FEATURE_FLAG_ALL);
  }
}


/***************************************************************************************/

inline float updateEdgeLevel( ISPCSubdivMesh* mesh, const Vec3fa& cam_pos, const unsigned int e0, const unsigned int e1)
{
  const Vec3fa v0 = mesh->positions[0][mesh->position_indices[e0]];
  const Vec3fa v1 = mesh->positions[0][mesh->position_indices[e1]];
  const Vec3fa edge = v1-v0;
  const Vec3fa P = 0.5f*(v1+v0);
  const Vec3fa dist = Vec3fa(cam_pos) - P;
  return max(min(LEVEL_FACTOR*(0.5f*length(edge)/length(dist)),MAX_EDGE_LEVEL),MIN_EDGE_LEVEL);
}

void updateEdgeLevelBuffer( ISPCSubdivMesh* mesh, const Vec3fa& cam_pos, unsigned int startID, unsigned int endID )
{
  for (unsigned int f=startID; f<endID;f++)
  {
    unsigned int e = mesh->face_offsets[f];
    unsigned int N = mesh->verticesPerFace[f];
    if (N == 4) /* fast path for quads */
      for (unsigned int i=0; i<4; i++)
        mesh->subdivlevel[e+i] =  updateEdgeLevel(mesh,cam_pos,e+(i+0),e+(i+1)%4);
       else if (N == 3) /* fast path for triangles */
         for (unsigned int i=0; i<3; i++)
           mesh->subdivlevel[e+i] =  updateEdgeLevel(mesh,cam_pos,e+(i+0),e+(i+1)%3);
       else /* fast path for general polygons */
         for (unsigned int i=0; i<N; i++)
           mesh->subdivlevel[e+i] =  updateEdgeLevel(mesh,cam_pos,e+(i+0),e+(i+1)%N);
  }
}

#if defined(ISPC)
void updateEdgeLevelBufferTask (int taskIndex, int threadIndex,  ISPCSubdivMesh* mesh, const Vec3fa& cam_pos )
{
  const unsigned int size = mesh->numFaces;
  const unsigned int startID = ((taskIndex+0)*size)/taskCount;
  const unsigned int endID   = ((taskIndex+1)*size)/taskCount;
  updateEdgeLevelBuffer(mesh,cam_pos,startID,endID);
}
#endif

void updateEdgeLevels(ISPCScene* scene_in, const Vec3fa& cam_pos)
{
  for (unsigned int g=0; g<scene_in->numGeometries; g++)
  {
    ISPCGeometry* geometry = g_ispc_scene->geometries[g];
    if (geometry->type != SUBDIV_MESH) continue;
    ISPCSubdivMesh* mesh = (ISPCSubdivMesh*) geometry;
#if defined(ISPC)
    parallel_for(size_t(0),size_t( (mesh->numFaces+4095)/4096 ),[&](const range<size_t>& range) {
    const int threadIndex = (int)TaskScheduler::threadIndex();
    for (size_t i=range.begin(); i<range.end(); i++)
      updateEdgeLevelBufferTask((int)i,threadIndex,mesh,cam_pos);
  }); 
#else
    updateEdgeLevelBuffer(mesh,cam_pos,0,mesh->numFaces);
#endif
    rtcUpdateGeometryBuffer(geometry->geometry,RTC_BUFFER_TYPE_LEVEL,0);
    rtcCommitGeometry(geometry->geometry);
  }
}

/* called by the C++ code for initialization */
extern "C" void device_init (char* cfg)
{
  /* initialize last seen camera */
  g_accu_vx = Vec3fa(0.0f);
  g_accu_vy = Vec3fa(0.0f);
  g_accu_vz = Vec3fa(0.0f);
  g_accu_p  = Vec3fa(0.0f);

  TutorialData_Constructor(&data);

  //data.occlusionFilterOpaque = GET_FUNCTION_POINTER(occlusionFilterOpaque);
  //data.occlusionFilterHair = GET_FUNCTION_POINTER(occlusionFilterHair);
  
} // device_init

extern "C" void renderFrameStandard (int* pixels,
                          const unsigned int width,
                          const unsigned int height,
                          const float time,
                          const ISPCCamera& camera)
{
/* render image */
#if defined(EMBREE_SYCL_TUTORIAL) && !defined(EMBREE_SYCL_RT_SIMULATION)
  TutorialData ldata = data;

#if defined(USE_SPECIALIZATION_CONSTANTS)
  sycl::event event = global_gpu_queue->submit([=](sycl::handler& cgh){
    cgh.set_specialization_constant<rtc_feature_mask>(g_used_features);
    const sycl::nd_range<2> nd_range = make_nd_range(height,width);
    cgh.parallel_for(nd_range,[=](sycl::nd_item<2> item, sycl::kernel_handler kh) {
      const RTCFeatureFlags feature_mask = kh.get_specialization_constant<rtc_feature_mask>();
      const unsigned int x = item.get_global_id(1); if (x >= width ) return;
      const unsigned int y = item.get_global_id(0); if (y >= height) return;
      RayStats stats;
      renderPixelStandard(ldata,x,y,pixels,width,height,time,camera,stats,feature_mask);
    });
  });
  global_gpu_queue->wait_and_throw();
#else
  sycl::event event = global_gpu_queue->submit([=](sycl::handler& cgh) {
    const sycl::nd_range<2> nd_range = make_nd_range(height,width);
    cgh.parallel_for(nd_range,[=](sycl::nd_item<2> item) {
      const unsigned int x = item.get_global_id(1); if (x >= width ) return;
      const unsigned int y = item.get_global_id(0); if (y >= height) return;
      RayStats stats;
      const RTCFeatureFlags feature_mask = RTC_FEATURE_FLAG_ALL;
      renderPixelStandard(ldata,x,y,pixels,width,height,time,camera,stats,feature_mask);
    });
  });
  global_gpu_queue->wait_and_throw();
#endif

  const auto t0 = event.template get_profiling_info<sycl::info::event_profiling::command_start>();
  const auto t1 = event.template get_profiling_info<sycl::info::event_profiling::command_end>();
  const double dt = (t1-t0)*1E-9;
  ((ISPCCamera*)&camera)->render_time = dt;
  
#else
  const int numTilesX = (width +TILE_SIZE_X-1)/TILE_SIZE_X;
  const int numTilesY = (height+TILE_SIZE_Y-1)/TILE_SIZE_Y;
  parallel_for(size_t(0),size_t(numTilesX*numTilesY),[&](const range<size_t>& range) {
    const int threadIndex = (int)TaskScheduler::threadIndex();
    for (size_t i=range.begin(); i<range.end(); i++)
      renderTileTask((int)i,threadIndex,pixels,width,height,time,camera,numTilesX,numTilesY);
  }); 
#endif
}

/* called by the C++ code to render */
extern "C" void device_render (int* pixels,
                           const unsigned int width,
                           const unsigned int height,
                           const float time,
                           const ISPCCamera& camera)
{
  /* create scene */
  if (data.scene == nullptr) {
    data.scene = convertScene(data.ispc_scene);
    if (g_subdiv_mode) updateEdgeLevels(data.ispc_scene,camera.xfm.p);
    rtcCommitScene (data.scene);
  }

  /* create accumulator */
  if (data.accu_width != width || data.accu_height != height) {
    alignedUSMFree(data.accu);
    data.accu = (Vec3ff*) alignedUSMMalloc((width*height)*sizeof(Vec3ff),16,EMBREE_USM_SHARED_DEVICE_READ_WRITE);
    data.accu_width = width;
    data.accu_height = height;
    for (unsigned int i=0; i<width*height; i++)
      data.accu[i] = Vec3ff(0.0f);
  }

  /* reset accumulator */
  bool camera_changed = g_changed || !g_accumulate || g_animation_mode; g_changed = false;
  camera_changed |= ne(g_accu_vx,camera.xfm.l.vx); g_accu_vx = camera.xfm.l.vx;
  camera_changed |= ne(g_accu_vy,camera.xfm.l.vy); g_accu_vy = camera.xfm.l.vy;
  camera_changed |= ne(g_accu_vz,camera.xfm.l.vz); g_accu_vz = camera.xfm.l.vz;
  camera_changed |= ne(g_accu_p, camera.xfm.p);    g_accu_p  = camera.xfm.p;

  if (camera_changed)
  {
    data.accu_count=0;
    for (unsigned int i=0; i<width*height; i++)
      data.accu[i] = Vec3ff(0.0f);

    if (g_subdiv_mode) {
      updateEdgeLevels(data.ispc_scene,camera.xfm.p);
      rtcCommitScene (data.scene);
    }
  }
  else
    data.accu_count++;

  if (g_animation_mode)
      UpdateScene(g_ispc_scene, time);

} // device_render

/* called by the C++ code for cleanup */
extern "C" void device_cleanup ()
{
  TutorialData_Destructor(&data);
  
} // device_cleanup

} // namespace embree
