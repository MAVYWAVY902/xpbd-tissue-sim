#ifndef __GPU_RESOURCE_HPP
#define __GPU_RESOURCE_HPP

#include <cuda_runtime_api.h>
#include <stdio.h>
#include <cassert>

#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, char *file, int line)
{
   if (code != cudaSuccess)
   {
      fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
      assert(0);
   }
}

namespace Sim
{

/** A class for managing memory on the GPU. 
 * Derived classes will be specialized for specific Simulation objects.
*/
// class GPUResource 
// {
//     public:
//     explicit GPUResource()
//     {}

//     virtual ~GPUResource()
//     {}

//     virtual void allocate() = 0;

//     virtual void copyToDevice() const = 0;

//     virtual void copyFromDevice() const = 0;
// };

class HostReadableGPUResource
{
    public:
    virtual void allocate() = 0;

    virtual void copyToDevice() const = 0;
};

class HostWritableGPUResource
{
    public:
    virtual void allocate() = 0;

    virtual void copyFromDevice() = 0;
};

} // namespace Sim

#endif // __GPU_RESOURCE_HPP