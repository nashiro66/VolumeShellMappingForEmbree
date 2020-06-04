% rtcSetGeometryMaxRadiusScale(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcSetGeometryMaxRadiusScale - assigns a maximal curve radius scale factor for min-width feature

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcSetGeometryMaxRadiusScale(RTCGeometry geometry, float maxRadiusScale);

#### DESCRIPTION

The `rtcSetMaxGeometryScale` function specifies a maximal scaling
factor for curve radii used by the min-width feature.

The min-width feature can increase the radius of curves, in order to
reduce aliasing and increase render times. The feature is disabled by
default and has to get enabled using the EMBREE_CURVE_MINWIDTH cmake
option.

When enabled, one has to specify a maximal curve radius scaling factor
using the [rtcSetGeometryMaxRadiusScale] function. This factor should
be a small number (e.g. 4) as the constructed BVH bounds get increased
in order to bound the curve in the worst case of maximal radii.

One also has to set the minWidthDistanceFactor in the
RTCIntersectContext when tracing a ray. This factor controls the
target size of a hair at some distance away of the ray origin.

For each control point p with radius r of a curve, the curve
intersectors first calculate a target radius r' as:

    r' = length(p-ray_org) * minWidthDistanceFactor

Typically the minWidthDistanceFactor is set by the application such
that the target curve radius projects to the width of half a pixel
(thus curve diameter is pixel sized).

The target radius r' is then clamped against the minimal bound r and
maximal bound maxRadiusScale*r to obtain the final radius r'':

    r'' = max(r, min(r', maxRadiusScale*r))

Thus curves close to the camera are rendered with a normal radii r,
and curves very far from the camera are not enlarged too much, as this
would be very expensive to render.

The min-width  feature is only implemented for the linear basis.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcInitIntersectContext]
