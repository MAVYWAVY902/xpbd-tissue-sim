#include <iostream>
#include <math.h>

#include <chrono>
#include <thread>

#include "utils/CudaHelperMath.h"

#include <Eigen/Dense>

struct DeviceSphereSDF
{
    float3 position;
    float radius;
};

__device__ float sphere_sdf_distance(const DeviceSphereSDF* sphere_sdf, const float* x)
{
    const float lx = x[0] - sphere_sdf->position.x;
    const float ly = x[1] - sphere_sdf->position.y;
    const float lz = x[2] - sphere_sdf->position.z;
    return sqrt(lx*lx + ly*ly + lz*lz) - sphere_sdf->radius;
}

__host__ float sphere_sdf_distance_host(const DeviceSphereSDF* sphere_sdf, const float* x)
{
    const float lx = x[0] - sphere_sdf->position.x;
    const float ly = x[1] - sphere_sdf->position.y;
    const float lz = x[2] - sphere_sdf->position.z;
    return std::sqrt(lx*lx + ly*ly + lz*lz) - sphere_sdf->radius;
}

__global__ void sphereMeshCollisionDetection(const DeviceSphereSDF* sphere_sdf, const float* vertices, int num_vertices, const int* faces, int num_faces)
{
    int face_index = blockIdx.x * blockDim.x + threadIdx.x;
    const int* face = faces + 3*face_index;
    float x1[3], x2[3], x3[3];
    x1[0] = (vertices + 3*face[0])[0];  x1[1] = (vertices + 3*face[0])[1]; x1[2] = (vertices + 3*face[0])[2];
    x2[0] = (vertices + 3*face[1])[0];  x2[1] = (vertices + 3*face[1])[1]; x2[2] = (vertices + 3*face[1])[2];
    x3[0] = (vertices + 3*face[2])[0];  x3[1] = (vertices + 3*face[2])[1]; x3[2] = (vertices + 3*face[2])[2];

    float min_p[3];
    min_p[0] = x1[0]; min_p[1] = x1[1]; min_p[2] = x1[2];
    float min_dist = 1000;

    const int num_samples = 21;
    for (int i = 0; i <= num_samples; i++)
    {
        for (int j = 0; j <= num_samples - i; j++)
        {
            const float u = (float)i / num_samples;
            const float v = (float)j / num_samples;
            const float w = 1 - u - v;
            float p[3];
            p[0] = u*x1[0] + v*x2[0] + w*x3[0];
            p[1] = u*x1[1] + v*x2[1] + w*x3[1];
            p[2] = u*x2[2] + v*x2[2] + w*x3[2];
            const float dist = sphere_sdf_distance(sphere_sdf, p);

            if (dist < min_dist)
            {
                min_dist = dist;
                min_p[0] = p[0]; min_p[1] = p[1]; min_p[2] = p[2];
            }
        }
    }
}

__global__ void sphereMeshCollisionDetectionParallel(const DeviceSphereSDF* sphere_sdf, const float* vertices, int num_vertices, const int* faces, int num_faces)
{
    int face_index = blockIdx.x;
    const int* face = faces + 3*face_index;

    __shared__ float x1[3], x2[3], x3[3];
    x1[0] = (vertices + 3*face[0])[0];  x1[1] = (vertices + 3*face[0])[1]; x1[2] = (vertices + 3*face[0])[2];
    x2[0] = (vertices + 3*face[1])[0];  x2[1] = (vertices + 3*face[1])[1]; x2[2] = (vertices + 3*face[1])[2];
    x3[0] = (vertices + 3*face[2])[0];  x3[1] = (vertices + 3*face[2])[1]; x3[2] = (vertices + 3*face[2])[2];
    // const float* x1 = vertices + 3*face[0];
    // const float* x2 = vertices + 3*face[1];
    // const float* x3 = vertices + 3*face[2];

    // map thread index to iteration
    const int N = 22;
    const int n = ceilf(0.5 * (2*N + 1) - sqrtf( (2*N + 1)*(2*N + 1) - 8*(threadIdx.x + 1)));
    const int i = min(n - 1, N - 1);    // clamp i to be still in the triangle
    const int j = threadIdx.x - (2*N - i + 1) * i / 2;

    // calculate barycentric coords u,v,w
    const float u = (float)i / (N - 1);
    const float v = (float)j / (N - 1);
    const float w = 1 - u - v;

    // find the distance
    float p[3];
    p[0] = u*x1[0] + v*x2[0] + w*x3[0];
    p[1] = u*x1[1] + v*x2[1] + w*x3[1];
    p[2] = u*x2[2] + v*x2[2] + w*x3[2];
    const float dist = sphere_sdf_distance(sphere_sdf, p);
}


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
    int num_vertices = 10000;
    Eigen::Matrix<float, 3, -1> vertices = Eigen::Matrix<float,3,-1>::Zero(3,num_vertices);
    for (int i = 0; i < num_vertices; i++)
    {
        vertices(0,i) = i*0.1;
        vertices(1,i) = i*0.15;
        vertices(2,i) = i*0.21;
    } 

    int num_faces = 100000;
    Eigen::Matrix<int, 3, -1> faces = Eigen::Matrix<int, 3, -1>::Zero(3, num_faces);
    for (int i = 0; i < num_faces; i++)
    {
        faces(0,i) = i/20;
        faces(1,i) = i/20+1;
        faces(2,i) = i/20+2;
    }

    std::cout << "created faces and vertices!" << std::endl;

    // allocate vertices and faces on device
    float* d_vertices;
    int* d_faces;
    cudaMalloc(&d_vertices, vertices.size() * sizeof(float));
    cudaMemcpy(d_vertices, vertices.data(), vertices.size() * sizeof(float), cudaMemcpyHostToDevice);
    cudaMalloc(&d_faces, faces.size() * sizeof(int));
    cudaMemcpy(d_faces, faces.data(), faces.size() * sizeof(int), cudaMemcpyHostToDevice);

    // allocate SDF on device
    DeviceSphereSDF sdf;
    sdf.position = make_float3(0.0, 0.0, 0.0);
    sdf.radius = 1.0;
    DeviceSphereSDF* d_sdf;
    cudaMalloc(&d_sdf, sizeof(DeviceSphereSDF));
    cudaMemcpy(d_sdf, &sdf, sizeof(DeviceSphereSDF), cudaMemcpyHostToDevice);

    int block_size = 256;
    int num_blocks;

    /////////////////////////////////////////////////////////////////////
    auto start2 = std::chrono::high_resolution_clock::now();

    num_blocks = (num_faces + block_size - 1) / block_size;

    sphereMeshCollisionDetection<<<num_blocks, block_size>>>(d_sdf, d_vertices, num_vertices, d_faces, num_faces);

    cudaDeviceSynchronize();

    auto end2 = std::chrono::high_resolution_clock::now();
    auto nanosec2 = std::chrono::duration_cast<std::chrono::nanoseconds>(end2 - start2);
    std::cout << "Elapsed time on GPU: " << nanosec2.count() << " ns" << std::endl;
    /////////////////////////////////////////////////////////////////////
    // auto start = std::chrono::high_resolution_clock::now();

    // num_blocks = num_faces;
    // sphereMeshCollisionDetectionParallel<<<num_blocks, block_size>>>(d_sdf, d_vertices, num_vertices, d_faces, num_faces);

    // // wait for GPU
    // cudaDeviceSynchronize();

    // auto end = std::chrono::high_resolution_clock::now();
    // auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    // std::cout << "Elapsed time on GPU (more parallel): " << nanosec.count() << " ns" << std::endl;

    ///////////////////////////////////////////////////////////////////
    cudaFree(d_sdf);
    cudaFree(d_vertices);
    cudaFree(d_faces);

    auto start3 = std::chrono::high_resolution_clock::now();
    for (int face_index = 0; face_index < num_faces; face_index++)
    {
        const int* face = faces.data() + 3*face_index;
        const float* x1 = vertices.data() + 3*face[0];
        const float* x2 = vertices.data() + 3*face[1];
        const float* x3 = vertices.data() + 3*face[2];

        const int num_samples = 16;
        for (int i = 0; i <= num_samples; i++)
        {
            for (int j = 0; j <= num_samples; j++)
            {
                const float u = (float)i / num_samples;
                const float v = (float)j / num_samples;
                const float w = 1 - u - v;
                float p[3];
                p[0] = u*x1[0] + v*x2[0] + w*x3[0];
                p[1] = u*x1[1] + v*x2[1] + w*x3[1];
                p[2] = u*x2[2] + v*x2[2] + w*x3[2];
                const float dist = sphere_sdf_distance_host(&sdf, p);
            }
        }
    }
    auto end3 = std::chrono::high_resolution_clock::now();
    auto nanosec3 = std::chrono::duration_cast<std::chrono::nanoseconds>(end3 - start3);
    std::cout << "Elapsed time on CPU: " << nanosec3.count() << " ns" << std::endl;

    

    return 0;
}