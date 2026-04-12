#ifndef __WRITABLE_ARRAY_GPU_RESOURCE_HPP
#define __WRITABLE_ARRAY_GPU_RESOURCE_HPP

#include "gpu/resource/GPUResource.hpp"

#include <iostream>

namespace Sim
{

template <typename T>
class WritableArrayGPUResource : public HostReadableGPUResource, public HostWritableGPUResource
{
    public:
    explicit WritableArrayGPUResource(T* arr, int num_elements)
        : _arr(arr), _num_elements(num_elements)
    {
    }

    virtual ~WritableArrayGPUResource()
    {
        cudaFree(_d_arr);
    }

    virtual void allocate() override
    {
        _arr_size = _num_elements * sizeof(T);
        // CHECK_CUDA_ERROR(cudaHostRegister(_arr, _arr_size, cudaHostRegisterMapped));
        // CHECK_CUDA_ERROR(cudaHostGetDevicePointer((void**)&_d_arr, _arr, 0));
        cudaMalloc((void**)&_d_arr, _arr_size);
    }

    virtual void fullCopyToDevice() const override
    {
        cudaMemcpy(_d_arr, _arr, _arr_size, cudaMemcpyHostToDevice);
    }

    virtual void partialCopyToDevice() const override
    {
        fullCopyToDevice();
    }

    virtual void copyFromDevice() override
    {
        cudaMemcpy(_arr, _d_arr, _arr_size, cudaMemcpyDeviceToHost);
    }

    T* gpuArr() { return _d_arr; }

    private:
    T* _arr;
    int _num_elements;
    size_t _arr_size;

    T* _d_arr;

};

} // namespace Sim

#endif // __WRITABLE_ARRAY_GPU_RESOURCE_HPP