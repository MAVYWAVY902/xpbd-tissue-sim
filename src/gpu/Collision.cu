#include "gpu/Collision.cuh"

#include "gpu/SphereSDFGPUResource.hpp"
#include "gpu/BoxSDFGPUResource.hpp"
#include "gpu/CylinderSDFGPUResource.hpp"

__host__ void launchCollisionKernel(const Sim::HostReadableGPUResource* sdf_resource, const Sim::MeshGPUResource* mesh_resource, int num_vertices, int num_faces)
{
    const int block_size = 256;
    const int num_blocks = (num_faces + block_size - 1) / block_size;

    // spawn GPU kernel depending on the type of SDF being collided with
    if (const Sim::SphereSDFGPUResource* sphere_sdf_resource = dynamic_cast<const Sim::SphereSDFGPUResource*>(sdf_resource))
    {
        sphereMeshCollisionDetection<<<num_blocks, block_size>>>(sphere_sdf_resource->gpuSDF(),
                                                                 mesh_resource->gpuVertices(),
                                                                 num_vertices,
                                                                 mesh_resource->gpuFaces(),
                                                                 num_faces);
    }
    else if (const Sim::BoxSDFGPUResource* box_sdf_resource = dynamic_cast<const Sim::BoxSDFGPUResource*>(sdf_resource))
    {

    }
    else if (const Sim::CylinderSDFGPUResource* cyl_sdf_resource = dynamic_cast<const Sim::CylinderSDFGPUResource*>(sdf_resource))
    {

    }
}

__device__ float sphere_sdf_distance(const Sim::GPUSphereSDF* sphere_sdf, const float* x)
{
    const float lx = x[0] - sphere_sdf->position.x;
    const float ly = x[1] - sphere_sdf->position.y;
    const float lz = x[2] - sphere_sdf->position.z;
    return sqrt(lx*lx + ly*ly + lz*lz) - sphere_sdf->radius;
}

__global__ void sphereMeshCollisionDetection(const Sim::GPUSphereSDF* sphere_sdf, const float* vertices, int num_vertices, const int* faces, int num_faces)
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

__global__ void sphereMeshCollisionDetectionParallel(const Sim::GPUSphereSDF* sphere_sdf, const float* vertices, int num_vertices, const int* faces, int num_faces)
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