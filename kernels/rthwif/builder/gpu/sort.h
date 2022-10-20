// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#if defined(EMBREE_SYCL_SUPPORT)
#include "../../builder/gpu/common.h"

#define RADIX_SORT_BINS        256    

#if 0
#define RADIX_SORT_MAX_NUM_DSS 256
#define RADIX_SORT_WG_SIZE     256
#else
#define RADIX_SORT_MAX_NUM_DSS 128
#define RADIX_SORT_WG_SIZE     512
#endif

#define RADIX_ITERATIONS_64BIT 8

//todo access byte directly
namespace embree
{
  namespace gpu
  {

    __forceinline uint getNumWGsScratchSize(const ssize_t scratch_size)
    {
      return min(max( (int)floorf(scratch_size / (sizeof(uint)*RADIX_SORT_BINS))-1,(int)1),RADIX_SORT_MAX_NUM_DSS);
    }
    
    __forceinline void localAtomicBallot(uint *const histogram, const uint ID, const uint add)
    {
      uint mask = sub_group_ballot(true);
      while(mask)
      {
        const uint first = sycl::ctz(mask);
        const uint index = sub_group_broadcast(ID,first);
        const bool cmp = ID == index;
#if 0     
        const uint count = cmp ? add : 0;
        const uint reduced_count = sub_group_reduce(count, SYCL_ONEAPI::plus<uint>());
        mask &= ~sub_group_ballot(cmp);
#else
        const uint cmp_mask = sub_group_ballot(cmp);
        const uint reduced_count = sycl::popcount(cmp_mask) * add;
        mask &= ~cmp_mask;        
#endif

        sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> hist(histogram[ID]);                                                                                    
        if (get_sub_group_local_id() == first)
          hist += reduced_count;
      }        
    }
    

    __forceinline void radix_sort_single_workgroup(sycl::queue &gpu_queue, uint64_t *_input, uint64_t *_output, const uint numPrimitives,  const uint start_iteration, const uint stop_iteration, double &time)
    {
      static const uint RADIX_SORT_SINGLE_WG_SIZE = 256; 

      struct __aligned(RADIX_SORT_SINGLE_WG_SIZE/32 * sizeof(uint)) BinFlags
      {
        uint flags[RADIX_SORT_SINGLE_WG_SIZE/32];
      };
    
      {
        const sycl::nd_range<1> nd_range1(RADIX_SORT_SINGLE_WG_SIZE,sycl::range<1>(RADIX_SORT_SINGLE_WG_SIZE));          
        sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                                     sycl::accessor< uint, 1, sycl_read_write, sycl_local> histogram(sycl::range<1>(RADIX_SORT_SINGLE_WG_SIZE),cgh);
                                                     sycl::accessor< uint, 1, sycl_read_write, sycl_local> sums(sycl::range<1>(RADIX_SORT_SINGLE_WG_SIZE),cgh);
                                                     sycl::accessor< uint, 1, sycl_read_write, sycl_local> prefix_sums(sycl::range<1>(RADIX_SORT_SINGLE_WG_SIZE),cgh);
                                                     sycl::accessor< uint, 1, sycl_read_write, sycl_local> local_offset(sycl::range<1>(RADIX_SORT_SINGLE_WG_SIZE),cgh);
                                                     sycl::accessor< BinFlags, 1, sycl_read_write, sycl_local> bin_flags(sycl::range<1>(RADIX_SORT_BINS),cgh);
                                                   
                                                     cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16)                                                                    
                                                                      {                                                                                                                                            
                                                                        const uint localID     = item.get_local_id(0);
                                                                        const uint step_local  = item.get_local_range().size();
                                                                        const uint subgroupID      = get_sub_group_id();
                                                                        const uint subgroupSize    = get_sub_group_size();
                                                                        const uint subgroupLocalID = get_sub_group_local_id();

                                                                        for (uint iter = start_iteration;iter<stop_iteration;iter++)
                                                                        {
                                                                          item.barrier(sycl::access::fence_space::local_space);
                                                                          
                                                                          const uint shift = iter*8;

                                                                          const uint64_t *const input  = (iter % 2) == 0 ? _input : _output;
                                                                          uint64_t *const output = (iter % 2) == 1 ? _input : _output;
                                                                        
                                                                          // ==== bin key into global histogram =====
                                                                      
                                                                          if (localID < RADIX_SORT_BINS)
                                                                            histogram[localID] = 0;

                                                                          item.barrier(sycl::access::fence_space::local_space);
                                                                    
                                                                          for (uint ID = localID; ID < numPrimitives; ID += step_local)
                                                                          {
                                                                            const uint bin = ((uint)(input[ID] >> shift)) & (RADIX_SORT_BINS - 1);
                                                                            //localAtomicBallot(histogram.get_pointer(),bin,1);
                                                                            gpu::atomic_add_local((uint*)histogram.get_pointer() + bin,(uint)1);
                                                                          }

                                                                          item.barrier(sycl::access::fence_space::local_space);

                                                                          // ==== reduce global histogram =====    
                                                                                                                                              
                                                                          SYCL_EXT_ONEAPI::sub_group sub_group = this_sub_group();
                                                                          sub_group.barrier();

                                                                          if (localID < RADIX_SORT_BINS)
                                                                          {
                                                                            const uint count = histogram[localID];
                                                                            const uint sum = sub_group_reduce(count, std::plus<uint>());
                                                                            const uint prefix_sum = sub_group_exclusive_scan(count, std::plus<uint>());

                                                                            sums[localID] = sum;
                                                                            prefix_sums[localID] = prefix_sum;
                                                                          }
                                                                        
                                                                          item.barrier(sycl::access::fence_space::local_space);


                                                                          if (subgroupID == 0)
                                                                          {
                                                                            uint off = 0;
                                                                            for (int i = subgroupLocalID; i < RADIX_SORT_BINS; i += subgroupSize)
                                                                            {
                                                                              local_offset[i] = off + prefix_sums[i];
                                                                              off += sums[i];
                                                                            }
                                                                          }

                                                                          item.barrier(sycl::access::fence_space::local_space);                                                                          
                                                                      
                                                                          // ==== scatter key/value pairs according to global historgram =====      
                                                                                                                         
                                                                          const uint flags_bin = localID / 32;
                                                                          const uint flags_bit = 1 << (localID % 32);                                                                      
                                                                      
                                                                          for (uint blockID = 0; blockID < numPrimitives; blockID += step_local)
                                                                          {
                                                                            item.barrier(sycl::access::fence_space::local_space);
                                                                        
                                                                            const uint ID = blockID + localID;                                                                            
                                                                        
                                                                            uint binID = 0;
                                                                            uint binOffset = 0;

                                                                            if (localID < RADIX_SORT_BINS)                                                                            
                                                                              for (int i=0;i<RADIX_SORT_SINGLE_WG_SIZE/32;i++)
                                                                                bin_flags[localID].flags[i] = 0;

                                                                            item.barrier(sycl::access::fence_space::local_space);
                                                                                 
                                                                            
                                                                            uint64_t in;
                                                                            if (ID < numPrimitives)
                                                                            {
                                                                              in = input[ID];                                                                              
                                                                              binID = (in >> shift) & (RADIX_SORT_BINS - 1);                                                                            
                                                                              binOffset = local_offset[binID];
                                                                              
                                                                              sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> bflags(bin_flags[binID].flags[flags_bin]);                                                                                    
                                                                              bflags.fetch_add(flags_bit);

                                                                            }

                                                                          
                                                                            item.barrier(sycl::access::fence_space::local_space);
                                                                            
                                                                            if (ID < numPrimitives)
                                                                            {
                                                                              uint prefix = 0;
                                                                              uint count = 0;
                                                                              for (uint i = 0; i < RADIX_SORT_SINGLE_WG_SIZE / 32; i++)
                                                                              {
                                                                                const uint bits = bin_flags[binID].flags[i];
                                                                                const uint full_count    = sycl::popcount(bits);
                                                                                const uint partial_count = sycl::popcount(bits & (flags_bit - 1));
                                                                                prefix += (i  < flags_bin) ? full_count : 0;
                                                                                prefix += (i == flags_bin) ? partial_count : 0;
                                                                                count += full_count;
                                                                              }

                                                                              output[binOffset + prefix] = in;
                                                                              if (prefix == count - 1)
                                                                                local_offset[binID] += count;                                                                          
                                                                            }
                                                                          
                                                                          }                                                                       
                                                                        }
                                                                      });
                                                   
                                                   });

        {            
          try {
            gpu_queue.wait_and_throw();
          } catch (sycl::exception const& e) {
            std::cout << "Caught synchronous SYCL exception:\n"
                      << e.what() << std::endl;
            FATAL("OpenCL Exception");     		
          }
          const auto t0 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_start>();
          const auto t1 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_end>();
          const double dt = (t1-t0)*1E-6;
          time += dt;
        }
      }    
    }

    
    template<typename sort_type>
    void sort_iteration_type(sycl::queue &gpu_queue, sort_type *input, sort_type *output, const uint primitives,  uint *global_histogram, const uint iter, double &time, const uint RADIX_SORT_NUM_DSS=256, const bool sync=false)
    {
      const uint shift = iter*8;

      
      // ==== bin key into global histogram =====
      {
        const sycl::nd_range<1> nd_range1(sycl::range<1>(RADIX_SORT_WG_SIZE*RADIX_SORT_NUM_DSS),sycl::range<1>(RADIX_SORT_WG_SIZE));          
        sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                                     sycl::accessor< uint, 1, sycl_read_write, sycl_local> histogram(sycl::range<1>(RADIX_SORT_BINS),cgh);                                                 
                                                     cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16)
                                                                      {
                                                                        const uint localID     = item.get_local_id(0);
                                                                        const uint step_local  = item.get_local_range().size();
                                                                        const uint groupID     = item.get_group(0);

                                                                        const uint startID = (groupID + 0)*primitives / RADIX_SORT_NUM_DSS;
                                                                        const uint endID   = (groupID + 1)*primitives / RADIX_SORT_NUM_DSS;
                                                                                                                                                                                                            
                                                                        if (localID < RADIX_SORT_BINS)
                                                                          histogram[localID] = 0;

                                                                        item.barrier(sycl::access::fence_space::local_space);
                                                                    
                                                                        for (uint ID = startID + localID; ID < endID; ID += step_local)
                                                                        {
                                                                          const uint64_t key = input[ID];
                                                                          const uint bin = ((uint)(key >> shift)) & (RADIX_SORT_BINS - 1);
                                                                          //gpu::localAtomicBallot(histogram.get_pointer(),bin,1);
                                                                          gpu::atomic_add_local((uint*)histogram.get_pointer() + bin,(uint)1);
                                                                          
                                                                        }

                                                                        item.barrier(sycl::access::fence_space::local_space);
    
                                                                        if (localID < RADIX_SORT_BINS)
                                                                          global_histogram[RADIX_SORT_BINS*groupID + localID] = histogram[localID];
                                                                    
                                                                      });
                                                 
                                                   });
        if (sync)
        {
          try {
            gpu_queue.wait_and_throw();
          } catch (sycl::exception const& e) {
            std::cout << "Caught synchronous SYCL exception:\n"
                      << e.what() << std::endl;
            FATAL("OpenCL Exception");     		
          }        
          const auto t0 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_start>();
          const auto t1 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_end>();
          const double dt = (t1-t0)*1E-6;
          PRINT2("bin phase",(float)dt);
          time += dt;
        }
      }

      
      // ==== scatter key/value pairs according to global historgram =====
      {
        
        struct __aligned(64) BinFlags
        {
          uint flags[RADIX_SORT_WG_SIZE/32];
        };
      
        const sycl::nd_range<1> nd_range1(sycl::range<1>(RADIX_SORT_WG_SIZE*RADIX_SORT_NUM_DSS),sycl::range<1>(RADIX_SORT_WG_SIZE));          
        sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                                     sycl::accessor< uint, 1, sycl_read_write, sycl_local> local_offset(sycl::range<1>(RADIX_SORT_BINS),cgh);
                                                     sycl::accessor< uint, 1, sycl_read_write, sycl_local> sums(sycl::range<1>(RADIX_SORT_BINS),cgh);                                                     
                                                     sycl::accessor< BinFlags, 1, sycl_read_write, sycl_local> bin_flags(sycl::range<1>(RADIX_SORT_BINS),cgh);
                                                     cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16)
                                                                      {
                                                                        const uint groupID     = item.get_group(0);
                                                                        const uint localID     = item.get_local_id(0);
                                                                        const uint step_local  = item.get_local_range().size();
                                                                        const uint subgroupID      = get_sub_group_id();
                                                                        const uint subgroupLocalID = get_sub_group_local_id();
                                                                        

                                                                        const uint startID = (groupID + 0)*primitives / RADIX_SORT_NUM_DSS;
                                                                        const uint endID   = (groupID + 1)*primitives / RADIX_SORT_NUM_DSS;

                                                                        
                                                                        /* --- reduce global histogram --- */
                                                                        uint local_hist = 0;
                                                                        uint prefix_sum = 0;
                                                                        
                                                                        if (localID < RADIX_SORT_BINS)
                                                                        {
                                                                          uint t = 0;
                                                                          for (uint j = 0; j < RADIX_SORT_NUM_DSS; j++)
                                                                          {
                                                                            const uint count = global_histogram[RADIX_SORT_BINS*j + localID];
                                                                            local_hist = (j == groupID) ? t : local_hist;
                                                                            t += count;
                                                                          }
                                                                        
                                                                          const uint count = t;
                                                                          const uint sum = sub_group_reduce(count, std::plus<uint>());
                                                                          prefix_sum = sub_group_exclusive_scan(count, std::plus<uint>());

                                                                          sums[subgroupID] = sum;
                                                                        }
                                                                      
                                                                        item.barrier(sycl::access::fence_space::local_space);

                                                                        if (localID < RADIX_SORT_BINS)
                                                                        {                                                                        
                                                                          const uint sums_prefix_sum = sub_group_broadcast(sub_group_exclusive_scan(sums[subgroupLocalID], std::plus<uint>()),subgroupID);
                                                                          const uint global_hist = sums_prefix_sum + prefix_sum;
                                                                          local_offset[localID] = global_hist + local_hist;
                                                                        }

                                                                        // === barrier comes later ===
                                                                        
                                                                        const uint flags_bin = localID / 32;
                                                                        const uint flags_bit = 1 << (localID % 32);                                                                      
                                                                      
                                                                        for (uint blockID = startID; blockID < endID; blockID += step_local)
                                                                        {
                                                                        
                                                                          const uint ID = blockID + localID;
                                                                        
                                                                          uint binID = 0;
                                                                          uint binOffset = 0;

#if 0                                                                          
                                                                          if (localID < RADIX_SORT_BINS)
                                                                            for (int i=0;i<RADIX_SORT_WG_SIZE/32;i++)
                                                                              bin_flags[localID].flags[i] = 0;
#else
                                                                          uint *const bflags = (uint*)&bin_flags.get_pointer()[0];                                                                          
                                                                          for (uint i=localID;i<RADIX_SORT_BINS*RADIX_SORT_WG_SIZE/32;i+=step_local)
                                                                            bflags[i] = 0;
#endif                                                                          

                                                                          item.barrier(sycl::access::fence_space::local_space);

                                                                          sort_type key_value;
                                                                          uint64_t key;
                                                                          if (ID < endID)
                                                                          {
                                                                            key_value = input[ID];
                                                                            key = key_value;
                                                                            binID = (key >> shift) & (RADIX_SORT_BINS - 1);
                                                                            binOffset = local_offset[binID];
                                                                            sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> bflags(bin_flags[binID].flags[flags_bin]);                                                                            
                                                                            bflags += flags_bit;
                                                                          }

                                                                          item.barrier(sycl::access::fence_space::local_space);
                                                                        
                                                                          if (ID < endID)
                                                                          {
                                                                            uint prefix = 0;
                                                                            uint count = 0;
                                                                            for (uint i = 0; i < RADIX_SORT_WG_SIZE / 32; i++)
                                                                            {
                                                                              const uint bits = bin_flags[binID].flags[i];
                                                                              const uint full_count    = sycl::popcount(bits);
                                                                              const uint partial_count = sycl::popcount(bits & (flags_bit - 1));
                                                                              prefix += (i  < flags_bin) ? full_count : 0;
                                                                              prefix += (i == flags_bin) ? partial_count : 0;
                                                                              count += full_count;
                                                                            }
                                                                            output[binOffset + prefix] = key_value;
                                                                            if (prefix == count - 1)
                                                                              local_offset[binID] += count;                                                                          
                                                                          }

                                                                          item.barrier(sycl::access::fence_space::local_space);

                                                                        }
                                                                    
                                                                      });
                                                 
                                                   });

        if (sync)
        {            
          try {
            gpu_queue.wait_and_throw();
          } catch (sycl::exception const& e) {
            std::cout << "Caught synchronous SYCL exception:\n"
                      << e.what() << std::endl;
            FATAL("OpenCL Exception");     		
          }
          const auto t0 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_start>();
          const auto t1 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_end>();
          const double dt = (t1-t0)*1E-6;
          time += dt;
          PRINT2("prefix + scatter phase",(float)dt);

        }
      }
    }


    struct __aligned(64) GlobalHistograms
    {
      uint counts[RADIX_ITERATIONS_64BIT][RADIX_SORT_BINS];
    };

#define ONESWEEP_BLOCK_SIZE (8*1024)
  
    struct __aligned(64) BlockInfo
    {
      uint counts[RADIX_SORT_BINS];    
    };

  
    template<typename sort_type>
    void onesweep_sort(sycl::queue &gpu_queue, sort_type *input, sort_type *output, const uint numPrimitives, char *const scratch_mem, const uint start_iter, const uint end_iter, double &time, const uint RADIX_SORT_NUM_DSS, const uint sync = false)
    {
      static const uint LOCAL_COUNT_BIT      = (uint)1 << 30;    
      static const uint INCLUSIVE_PREFIX_BIT = (uint)1 << 31;
      static const uint COUNTS_MASK          = ~(LOCAL_COUNT_BIT | INCLUSIVE_PREFIX_BIT);
      static const uint BITS_MASK            =  (LOCAL_COUNT_BIT | INCLUSIVE_PREFIX_BIT);
    
      const uint numBlocks = (numPrimitives+ONESWEEP_BLOCK_SIZE-1)/ONESWEEP_BLOCK_SIZE;
      PRINT2(numPrimitives,numBlocks);
    
      GlobalHistograms *global_histograms = (GlobalHistograms*)(scratch_mem + 0);    
      BlockInfo        *blockInfo         =        (BlockInfo*)(scratch_mem + sizeof(GlobalHistograms));
      uint             *blockIDCounter    =        (uint*)(blockInfo + numBlocks);
      
      // === clear global histogram and block info data ===
      {
        static const uint CLEAR_WG_SIZE = 256; 
      
        const sycl::nd_range<1> nd_range1(sycl::range<1>(numBlocks*CLEAR_WG_SIZE),sycl::range<1>(CLEAR_WG_SIZE));          
        sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                                     cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16)
                                                                      {
                                                                        const uint globalID    = item.get_global_id(0);                                                                      
                                                                        const uint localID     = item.get_local_id(0);
                                                                        const uint groupID     = item.get_group(0);
                                                                        blockInfo[groupID].counts[localID] = 0;                                                                      
                                                                        if (groupID == 0)
                                                                          for (uint i=0;i<RADIX_ITERATIONS_64BIT;i++)
                                                                            global_histograms->counts[i][localID] = 0;

                                                                        if (globalID == 0)
                                                                          *blockIDCounter = 0;
                                                                      });
                                                 
                                                   });
        if (sync)
        {
          gpu::waitOnQueueAndCatchException(gpu_queue);
          const auto t0 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_start>();
          const auto t1 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_end>();
          const double dt = (t1-t0)*1E-6;
          PRINT2("clear histograms",(float)dt);
          time += dt;
        }
      }

      // ==== bin keys into global histograms =====
      {
        const sycl::nd_range<1> nd_range1(sycl::range<1>(RADIX_SORT_WG_SIZE*RADIX_SORT_NUM_DSS),sycl::range<1>(RADIX_SORT_WG_SIZE));          
        sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                                     sycl::accessor< uint, 1, sycl_read_write, sycl_local> _local_histograms(sycl::range<1>(RADIX_SORT_BINS*RADIX_ITERATIONS_64BIT),cgh);                                                 
                                                     cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16)
                                                                      {
                                                                        const uint localID     = item.get_local_id(0);
                                                                        const uint step_local  = item.get_local_range().size();
                                                                        const uint groupID     = item.get_group(0);

                                                                        const uint startID = (groupID + 0)*numPrimitives / RADIX_SORT_NUM_DSS;
                                                                        const uint endID   = (groupID + 1)*numPrimitives / RADIX_SORT_NUM_DSS;

                                                                        uint *local_histograms = (uint*)_local_histograms.get_pointer();
                                                                     
                                                                        for (uint i=localID;i<RADIX_SORT_BINS*RADIX_ITERATIONS_64BIT;i+=step_local)
                                                                          local_histograms[i] = 0;

                                                                        item.barrier(sycl::access::fence_space::local_space);
                                                                    
                                                                        for (uint ID = startID + localID; ID < endID; ID += step_local)
                                                                        {
                                                                          const uint64_t key = input[ID];                                                                        
                                                                          for (uint r = start_iter; r < end_iter; r++)
                                                                          {
                                                                            const uint shift = r*8;
                                                                            const uint bin = ((uint)(key >> shift)) & (RADIX_SORT_BINS - 1);
                                                                            gpu::atomic_add_local(local_histograms + RADIX_SORT_BINS * r + bin,(uint)1);
                                                                            //gpu::localAtomicBallot(local_histograms + RADIX_SORT_BINS * r,bin,1);
                                                                          }
                                                                        }

                                                                        item.barrier(sycl::access::fence_space::local_space);
                                                                      
                                                                        if (localID < RADIX_SORT_BINS)
                                                                          for (uint r = 0; r < RADIX_ITERATIONS_64BIT; r++)                                                                        
                                                                            gpu::atomic_add_global(&global_histograms->counts[r][localID],local_histograms[RADIX_SORT_BINS * r + localID]);
                                                                    
                                                                      });
                                                 
                                                   });
        if (sync)
        {
          gpu::waitOnQueueAndCatchException(gpu_queue);
          const auto t0 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_start>();
          const auto t1 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_end>();
          const double dt = (t1-t0)*1E-6;
          PRINT2("create global histograms",(float)dt);
          time += dt;
        }
      }

    
      // for (uint r=0;r<8;r++)
      // {
      //   uint sum = 0;
      //   for (uint i=0;i<RADIX_SORT_BINS;i++)
      //   {
      //     sum += global_histograms->counts[r][i];
      //   }
      //   PRINT2(r,sum);
      // }

      // ==== compute prefix sum for global histograms =====
      {
        const sycl::nd_range<1> nd_range1(sycl::range<1>(RADIX_ITERATIONS_64BIT*RADIX_SORT_BINS),sycl::range<1>(RADIX_SORT_BINS));          
        sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                                     sycl::accessor< uint, 1, sycl_read_write, sycl_local> sums(sycl::range<1>(RADIX_SORT_BINS),cgh);
                                                     cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16)
                                                                      {
                                                                        const uint localID         = item.get_local_id(0);
                                                                        const uint groupID         = item.get_group(0);                                                                      
                                                                        const uint subgroupID      = get_sub_group_id();
                                                                        const uint subgroupLocalID = get_sub_group_local_id();

                                                                        const uint count = global_histograms->counts[groupID][localID];
                                                                        const uint sum = sub_group_reduce(count, std::plus<uint>());
                                                                        const uint prefix_sum = sub_group_exclusive_scan(count, std::plus<uint>());
                                                                        sums[subgroupID] = sum;
                                                                      
                                                                        item.barrier(sycl::access::fence_space::local_space);
                                                                      
                                                                        const uint sums_prefix_sum = sub_group_broadcast(sub_group_exclusive_scan(sums[subgroupLocalID], std::plus<uint>()),subgroupID);                                                                      
                                                                        global_histograms->counts[groupID][localID] = sums_prefix_sum + prefix_sum;
                                                                      
                                                                      });
                                                 
                                                   });
        if (sync)
        {      
          gpu::waitOnQueueAndCatchException(gpu_queue);      
          const auto t0 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_start>();
          const auto t1 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_end>();
          const double dt = (t1-t0)*1E-6;
          PRINT2("compute prefix sums",(float)dt);
          time += dt;
        }
      }

      // === scatter iteration ===
      struct __aligned(RADIX_SORT_WG_SIZE/32 * sizeof(uint)) BinFlags
      {
        uint flags[RADIX_SORT_WG_SIZE/32];
      };
    
      for (uint iter = start_iter; iter<end_iter; iter++)
      {
        // === clear block info data ===
        {
          static const uint CLEAR_WG_SIZE = 256; 
      
          const sycl::nd_range<1> nd_range1(sycl::range<1>(numBlocks*CLEAR_WG_SIZE),sycl::range<1>(CLEAR_WG_SIZE));          
          sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                                       cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16)
                                                                        {
                                                                          const uint localID     = item.get_local_id(0);
                                                                          const uint groupID     = item.get_group(0);
                                                                          blockInfo[groupID].counts[localID] = 0;                                                                      
                                                                        });
                                                 
                                                     });
          if (sync)
          {          
            gpu::waitOnQueueAndCatchException(gpu_queue);
            const auto t0 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_start>();
            const auto t1 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_end>();
            const double dt = (t1-t0)*1E-6;
            PRINT2("clear blocks",(float)dt);
            time += dt;
          }
        }

        
        sort_type *iter_input  = ((iter-start_iter) % 2) == 0 ? input : output;
        sort_type *iter_output = ((iter-start_iter) % 2) == 0 ? output : input;
        
        
        // === scan over primitives ===        
        static const uint SCATTER_WG_SIZE = RADIX_SORT_WG_SIZE; // needs to be >= 256
        const sycl::nd_range<1> nd_range1(sycl::range<1>(numBlocks*SCATTER_WG_SIZE),sycl::range<1>(SCATTER_WG_SIZE));          
        sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                                     sycl::accessor< uint, 1, sycl_read_write, sycl_local> _local_histogram(sycl::range<1>(RADIX_SORT_BINS),cgh);
                                                     sycl::accessor< uint, 1, sycl_read_write, sycl_local> _global_prefix_sum(sycl::range<1>(RADIX_SORT_BINS),cgh);
                                                     sycl::accessor< BinFlags, 1, sycl_read_write, sycl_local> bin_flags(sycl::range<1>(RADIX_SORT_BINS),cgh);
                                                     sycl::accessor< uint      ,  0, sycl_read_write, sycl_local> _wgID(cgh);

                                                     cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16)
                                                                      {
                                                                        const uint localID     = item.get_local_id(0);
                                                                        const uint localSize   = item.get_local_range().size();
                                                                        const uint numGroups   = item.get_group_range(0);
                                                                        
                                                                        //const uint blockID         = item.get_group(0);                                                                      
                                                                        //const uint subgroupID      = get_sub_group_id();
                                                                        //const uint subgroupLocalID = get_sub_group_local_id();

                                                                        uint *local_histogram   = (uint*)_local_histogram.get_pointer();
                                                                        uint *global_prefix_sum = (uint*)_global_prefix_sum.get_pointer();

                                                                        if (localID < RADIX_SORT_BINS)
                                                                          local_histogram[localID] = 0;

                                                                        uint &wgID = *_wgID.get_pointer();
                                                                        if (localID == 0)
                                                                        {
                                                                          // get work group ID
                                                                          wgID = gpu::atomic_add_global(blockIDCounter,(uint)1);
                                                                          // if we are the last block, reset work group ID
                                                                          if (wgID == numGroups-1)
                                                                            *blockIDCounter = 0;
                                                                        }
                                                                        
                                                                        item.barrier(sycl::access::fence_space::local_space);
                                                                        const uint blockID = wgID;
                                                                        
                                                                        const uint shift = iter*8;                                                                        
                                                                        const uint startID = blockID * ONESWEEP_BLOCK_SIZE;
                                                                        const uint endID   = min(startID + ONESWEEP_BLOCK_SIZE,numPrimitives);
                                                                        for (uint ID = startID + localID; ID < endID; ID+=localSize)
                                                                        {
                                                                          const uint64_t key = iter_input[ID];
                                                                          const uint bin = ((uint)(key >> shift)) & (RADIX_SORT_BINS - 1);
                                                                          gpu::atomic_add_local(local_histogram + bin,(uint)1);
                                                                        }

                                                                        item.barrier(sycl::access::fence_space::local_space);

                                                                        const uint global_bin_prefix_sum = (localID < RADIX_SORT_BINS) ? global_histograms->counts[iter][localID] : 0;
                                                                        
                                                                        if (localID < RADIX_SORT_BINS)
                                                                        {
                                                                          const uint local_bin_count = local_histogram[localID];
                                                                          // === write per block counts ===
                                                                          sycl::atomic_ref<uint, sycl::memory_order::acq_rel, sycl::memory_scope::device,sycl::access::address_space::global_space> global_write(blockInfo[blockID].counts[localID]);
                                                                          uint bits = LOCAL_COUNT_BIT;
                                                                          bits |= blockID == 0 ? INCLUSIVE_PREFIX_BIT : 0;
                                                                          
                                                                          global_write.store( local_bin_count | bits );
                                                                          
                                                                          // === look back to get prefix sum ===
                                                                          uint sum = 0;                                                                          
                                                                          if (blockID > 0)
                                                                          {
                                                                            int lookback_blockID = blockID-1;
                                                                            
                                                                            while(1)
                                                                            {
                                                                              sycl::atomic_ref<uint, sycl::memory_order::acq_rel, sycl::memory_scope::device,sycl::access::address_space::global_space> global_read(blockInfo[lookback_blockID].counts[localID]);
                                                                              const uint prev_count = global_read.load();

                                                                              if ((prev_count & BITS_MASK) == 0) continue; // polling

                                                                              sum += prev_count & COUNTS_MASK;
                                                                              
                                                                              if (prev_count & INCLUSIVE_PREFIX_BIT) break;

                                                                              lookback_blockID--;
                                                                            }
                                                                          }

                                                                          global_prefix_sum[localID] = global_bin_prefix_sum + sum;
                                                                          
                                                                          const uint inclusive_sum = local_bin_count + sum;
                                                                          global_write.store( inclusive_sum | INCLUSIVE_PREFIX_BIT |  LOCAL_COUNT_BIT);
                                                                        }

                                                                        item.barrier(sycl::access::fence_space::local_space);
                                                                          
                                                                        //sycl::atomic_ref<uint, sycl::memory_order::acq_rel, sycl::memory_scope::device,sycl::access::address_space::global_space> global_counts();
                                                                        //scratch_mem_counter.store(total_offset);
                                                                        
                                                                        const uint flags_bin = localID / 32;
                                                                        const uint flags_bit = 1 << (localID % 32);                                                                      

                                                                        for (uint chunkID = startID; chunkID < endID; chunkID += localSize)
                                                                        {
                                                                        
                                                                          const uint ID = chunkID + localID;
                                                                        
                                                                          uint binID = 0;
                                                                          uint binOffset = 0;

                                                                          if (localID < RADIX_SORT_BINS)
                                                                            for (int i=0;i<RADIX_SORT_WG_SIZE/32;i++)
                                                                              bin_flags[localID].flags[i] = 0;

                                                                          item.barrier(sycl::access::fence_space::local_space);

                                                                          sort_type key_value;
                                                                          uint64_t key;
                                                                          if (ID < endID)
                                                                          {
                                                                            key_value = iter_input[ID];
                                                                            key = key_value;
                                                                            binID = (key >> shift) & (RADIX_SORT_BINS - 1);
                                                                            binOffset = global_prefix_sum[binID];
                                                                            sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> bflags(bin_flags[binID].flags[flags_bin]);                                                                            
                                                                            bflags += flags_bit;
                                                                          }

                                                                          item.barrier(sycl::access::fence_space::local_space);
                                                                        
                                                                          if (ID < endID)
                                                                          {
                                                                            uint prefix = 0;
                                                                            uint count = 0;
                                                                            for (uint i = 0; i < RADIX_SORT_WG_SIZE / 32; i++)
                                                                            {
                                                                              const uint bits = bin_flags[binID].flags[i];
                                                                              const uint full_count    = sycl::popcount(bits);
                                                                              const uint partial_count = sycl::popcount(bits & (flags_bit - 1));
                                                                              prefix += (i  < flags_bin) ? full_count : 0;
                                                                              prefix += (i == flags_bin) ? partial_count : 0;
                                                                              count += full_count;
                                                                            }
                                                                            iter_output[binOffset + prefix] = key_value;
                                                                            if (prefix == count - 1)
                                                                              global_prefix_sum[binID] += count;                                                                          
                                                                          }

                                                                          item.barrier(sycl::access::fence_space::local_space);

                                                                        }
                                                                        
                                                                        
                                                                        
                                                                      });
                                                 
                                                   });
        if (sync)
        {        
          gpu::waitOnQueueAndCatchException(gpu_queue);
      
          const auto t0 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_start>();
          const auto t1 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_end>();
          const double dt = (t1-t0)*1E-6;
          PRINT2("compute prefix sums",(float)dt);
          time += dt;
        }
      }    
    }
    
    
  };
};

#endif
