#ifndef __ARRAY_GPU_RESOURCE_HPP
#define __ARRAY_GPU_RESOURCE_HPP

#include "gpu/GPUResource.hpp"

#include <iostream>

namespace Sim
{

template <typename T>
class ArrayGPUResource : public HostReadableGPUResource, public HostWritableGPUResource
{
    public:
    explicit ArrayGPUResource(T* arr, int num_elements)
        : _arr(arr), _num_elements(num_elements)
    {
    }

    virtual ~ArrayGPUResource()
    {
        cudaFree(_d_arr);
    }

    virtual void allocate() override
    {
        _arr_size = _num_elements * sizeof(T);
        cudaMalloc((void**)&_d_arr, _arr_size);
    }

    virtual void copyToDevice() const override
    {
        cudaMemcpy(_d_arr, _arr, _arr_size, cudaMemcpyHostToDevice);
    }

    virtual void copyFromDevice() override
    {
        cudaMemcpy(_arr, _d_arr, _arr_size, cudaMemcpyDeviceToHost);
        std::cout <<"copyFromDevice[0]: " << _arr[0].penetration_dist << std::endl;
    }

    T* gpuArr() { return _d_arr; }

    // TODO: remove this - not needed since we should keep track of host memory already
    T* arr() { return _arr; }

    private:
    T* _arr;
    int _num_elements;
    size_t _arr_size;

    T* _d_arr;

};

} // namespace Sim

#endif // __ARRAY_GPU_RESOURCE_HPP