#ifndef __VECTOR_GPU_RESOURCE_HPP
#define __VECTOR_GPU_RESOURCE_HPP

#include "gpu/resource/GPUResource.hpp"

#include <iostream>
#include <vector>

namespace Sim
{

template <typename T>
class VectorGPUResource : public HostReadableGPUResource
{
    public:
    explicit VectorGPUResource(const std::vector<T>* vec)
        : _vec(vec)
    {
    }

    VectorGPUResource()
        : _vec(nullptr)
    {
    }

    virtual ~VectorGPUResource()
    {
        cudaFree(_d_arr);
    }

    void setVecPtr(const std::vector<T>* vec)
    {
        _vec = vec;
    }

    virtual void allocate() override
    {
        // if we have already allocated memory and the vector's capacity has not changed, no need to reallocate
        if (_vec->capacity() * sizeof(T) == _allocated_size)
            return;

        // free current device memory, if allocated already
        if (_d_arr)
            cudaFree(_d_arr);

        // allocate conservatively according to the vector's capacity, not size
        // that way we can tolerate some resizing of the vector
        _allocated_size = _vec->capacity() * sizeof(T);
        cudaMalloc((void**)&_d_arr, _allocated_size);
    }

    virtual void fullCopyToDevice() const override
    {
        // NOTE: since the vector's size can change, we must call allocate() before copyToDevice() to make sure we have allocated enough mem
        assert(_vec->capacity() * sizeof(T) == _allocated_size);

        // when we copy, only copy up to vector size (even though we've allocated extra space on device)
        // this requires us to pass the number of "active" vector elements we have to the GPU
        size_t copy_size = _vec->size() * sizeof(T);
        cudaMemcpy(_d_arr, _vec->data(), copy_size, cudaMemcpyHostToDevice);
    }

    virtual void partialCopyToDevice() const override
    {
        fullCopyToDevice();
    }

    T* gpuArr() const { return _d_arr; }

    private:
    const std::vector<T>* _vec;
    size_t _allocated_size;

    T* _d_arr;

};


} // namespace Sim

#endif // __VECTOR_GPU_RESOURCE_HPP