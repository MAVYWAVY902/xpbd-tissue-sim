#include <iostream>
#include <math.h>

#include <chrono>

__global__
void add(int n, float* x, float* y)
{
    int index = blockIdx.x * blockDim.x + threadIdx.x;
    int stride = blockDim.x * gridDim.x;
    for (int i = index; i < n; i += stride)
    {
        y[i] = x[i] + y[i];
    }
}

int main(void)
{
    int N = 1 << 26;
    float *x, *y;

    int blockSize = 256;
    int numBlocks = (N + blockSize - 1) / blockSize;

    std::cout << numBlocks << std::endl;


    // allocate CUDA Unified Memory
    cudaMallocManaged(&x, N*sizeof(float));
    cudaMallocManaged(&y, N*sizeof(float));

    // initialize x and y arrays
    for (int i = 0; i < N; i++)
    {
        x[i] = 1.0f;
        y[i] = 2.0f;
    }

    auto start = std::chrono::high_resolution_clock::now();

    // run kernel on GPU
    add<<<numBlocks, blockSize>>>(N, x, y);

    // wait for GPU
    cudaDeviceSynchronize();

    

    auto end = std::chrono::high_resolution_clock::now();
    auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);

    std::cout << "Elapsed time: " << nanosec.count() << " ns" << std::endl;

    // allocate CUDA Unified Memory
    cudaMallocManaged(&x, N*sizeof(float));
    cudaMallocManaged(&y, N*sizeof(float));

    // initialize x and y arrays
    for (int i = 0; i < N; i++)
    {
        x[i] = 1.0f;
        y[i] = 2.0f;
    }

    start = std::chrono::high_resolution_clock::now();

    // run kernel on GPU
    add<<<1, blockSize>>>(N, x, y);

    // wait for GPU
    cudaDeviceSynchronize();

    end = std::chrono::high_resolution_clock::now();
    nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);

    std::cout << "Elapsed time: " << nanosec.count() << " ns" << std::endl;

    // free memory
    cudaFree(x);
    cudaFree(y);

    return 0;
}