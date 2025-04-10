#ifndef __GPU_RESOURCE_HPP
#define __GPU_RESOURCE_HPP

#include <cuda_runtime_api.h>
#include <iostream>
#include <cassert>

#define CHECK_CUDA_ERROR(val) checkCudaError((val), #val, __FILE__, __LINE__)
inline void checkCudaError(cudaError_t err, const char* const func, const char* const file,
           const int line)
{
    if (err != cudaSuccess)
    {
        std::cerr << "CUDA Runtime Error at: " << file << ":" << line
                  << std::endl;
        std::cerr << cudaGetErrorString(err) << " " << func << std::endl;
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

    /** Copies all pieces of data (i.e. the entire struct) over to the GPU.
     * This is usually done once after initializing and allocating the GPUResource.
     * For data that is constant throughout the simulation, we do not need to do this more than once.
    */
    virtual void fullCopyToDevice() const = 0;

    /** Copies only "dynamic" pieces of data (i.e. quantities that change throughout the course of the sim) over to the GPU.
     * This is avoids unnecessary memory copy overhead for copying data that is constant throughout the sim.
    */
    virtual void partialCopyToDevice() const = 0;

    // virtual void copyToDevice() const = 0;
};

class HostWritableGPUResource
{
    public:
    virtual void allocate() = 0;

    virtual void copyFromDevice() = 0;
};

} // namespace Sim

#endif // __GPU_RESOURCE_HPP