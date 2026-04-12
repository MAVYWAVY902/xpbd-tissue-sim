#ifndef __MONITORED_VECTOR_GPU_RESOURCE_HPP
#define __MONITORED_VECTOR_GPU_RESOURCE_HPP

#include "common/MonitoredVector.hpp"
#include "gpu/resource/GPUResource.hpp"

#include <iostream>

namespace Sim
{

template <typename T>
class MonitoredVectorGPUResource : public HostReadableGPUResource
{
    public:
    explicit MonitoredVectorGPUResource(const MonitoredVector<T>* vec)
        : _vec(vec)
    {
    }

    MonitoredVectorGPUResource()
        : _vec(nullptr)
    {
    }

    virtual ~MonitoredVectorGPUResource()
    {
        cudaFree(_d_arr);
    }

    void setVecPtr(const MonitoredVector<T>* vec)
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

        // only copy when the state of the vector has changed
        if (!_vec->stateChanged())
            return;

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
    const MonitoredVector<T>* _vec;
    size_t _allocated_size;

    T* _d_arr;

};


} // namespace Sim

#endif // __MONITORED_VECTOR_GPU_RESOURCE_HPP