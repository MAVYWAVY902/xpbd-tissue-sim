#include <iostream>
#include <math.h>

#include <chrono>
#include <thread>

#include "utils/CudaHelperMath.h"

#include "common/types.hpp"

#include "utils/MeshUtils.hpp"
#include "geometry/Mesh.hpp"
#include "geometry/SphereSDF.hpp"
#include "simobject/RigidPrimitives.hpp"
#include "gpu/GPUStructs.hpp"
#include "gpu/GPUResource.hpp"
#include "gpu/SphereSDFGPUResource.hpp"
#include "gpu/BoxSDFGPUResource.hpp"
#include "gpu/MeshGPUResource.hpp"
#include "gpu/CylinderSDFGPUResource.hpp"
#include "gpu/ArrayGPUResource.hpp"

struct GPUCollision
{
    float penetration_dist;
    float3 bary_coords;
};

__device__ float sphere_sdf_distance(const Sim::GPUSphereSDF* sphere_sdf, const float3& x)
{
    return length(x - sphere_sdf->position) - sphere_sdf->radius;
}

__device__ float3 sphere_sdf_gradient(const Sim::GPUSphereSDF* sphere_sdf, const float3& x)
{
    float3 grad =  normalize(x - sphere_sdf->position);
    // printf("x: %f, %f, %f\n", x.x, x.y, x.z);
    // printf("grad: %f, %f, %f\n", grad.x, grad.y, grad.z);
    return grad;
}

__global__ void sphereMeshCollisionDetection(const Sim::GPUSphereSDF* sphere_sdf, const float* vertices, int num_vertices, const int* faces, int num_faces, GPUCollision* collisions)
{
    int face_index = blockIdx.x * blockDim.x + threadIdx.x;
    if (face_index >= num_faces)    return;
    const int* face = faces + 3*face_index;
    float3 x1, x2, x3;
    x1.x = (vertices + 3*face[0])[0];  x1.y = (vertices + 3*face[0])[1]; x1.z = (vertices + 3*face[0])[2];
    x2.x = (vertices + 3*face[1])[0];  x2.y = (vertices + 3*face[1])[1]; x2.z = (vertices + 3*face[1])[2];
    x3.x = (vertices + 3*face[2])[0];  x3.y = (vertices + 3*face[2])[1]; x3.z = (vertices + 3*face[2])[2];

    // printf("f: %i, %i, %i\n", faces[0], faces[1], faces[2]);
    // printf("x1: %f, %f, %f\n", x1.x, x1.y, x1.z);
    // printf("x2: %f, %f, %f\n", x2.x, x2.y, x2.z);
    // printf("x3: %f, %f, %f\n", x3.x, x3.y, x3.z);
    // printf("x1: %f, %f, %f\n", vertices[0], vertices[1], vertices[2]);
    // printf("x2: %f, %f, %f\n", vertices[3], vertices[4], vertices[5]);
    // printf("x3: %f, %f, %f\n", vertices[6], vertices[7], vertices[8]);

    float3 min_p = x1;
    float3 min_bary_coords = make_float3(1.0, 0.0, 0.0);
    float min_dist = 1000;

    const int num_samples = 16;
    for (int i = 0; i <= num_samples; i++)
    {
        for (int j = 0; j <= num_samples - i; j++)
        {
            const float u = (float)i / num_samples;
            const float v = (float)j / num_samples;
            const float w = 1 - u - v;
            float3 p = u*x1 + v*x2 + w*x3;
            // printf("u: %f, v: %f, w: %f ", u, v, w);
            // printf("p: %f,%f,%f\n", p.x, p.y, p.z);
            const float dist = sphere_sdf_distance(sphere_sdf, p);

            if (dist < min_dist)
            {
                min_dist = dist;
                min_p = p;
                min_bary_coords = make_float3(u, v, w);
            }
        }
    }

    // printf("d: %f\n", min_dist);
    // printf("f: %i min_p: %f, %f, %f\n", face_index, min_p.x, min_p.y, min_p.z);
    // TODO: stream compaction
    // float3 grad = sphere_sdf_gradient(sphere_sdf, min_p);
    collisions[face_index].penetration_dist = min_dist;
    // collisions[face_index].normal = sphere_sdf_gradient(sphere_sdf, min_p);
    collisions[face_index].bary_coords = min_bary_coords;
    // collisions[face_index].surface_point = min_p - collisions[face_index].normal * min_dist;
    
}

__global__ void sphereMeshCollisionDetectionParallel(const Sim::GPUSphereSDF* sphere_sdf, const float* vertices, int num_vertices, const int* faces, int num_faces, GPUCollision* collisions)
{
    int face_index = blockIdx.x;
    __shared__ int3 face;
    face = make_int3(faces[3*face_index], faces[3*face_index+1], faces[3*face_index+2]);
    __shared__ float3 x1, x2, x3;
    x1.x = (vertices + 3*face.x)[0];  x1.y = (vertices + 3*face.x)[1]; x1.z = (vertices + 3*face.x)[2];
    x2.x = (vertices + 3*face.y)[0];  x2.y = (vertices + 3*face.y)[1]; x2.z = (vertices + 3*face.y)[2];
    x3.x = (vertices + 3*face.z)[0];  x3.y = (vertices + 3*face.z)[1]; x3.z = (vertices + 3*face.z)[2];
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
    float3 p = u*x1 + v*x2 + w*x3;
    const float dist = sphere_sdf_distance(sphere_sdf, p);
}


__host__ void launchCollisionKernel(const Sim::HostReadableGPUResource* sdf_resource, const Sim::MeshGPUResource* mesh_resource, int num_vertices, int num_faces, Sim::ArrayGPUResource<GPUCollision>* collisions_resource)
{
    const int block_size = 256;
    // const int num_blocks = (num_faces + block_size - 1) / block_size;
    const int num_blocks = num_faces;

    // spawn GPU kernel depending on the type of SDF being collided with
    if (const Sim::SphereSDFGPUResource* sphere_sdf_resource = dynamic_cast<const Sim::SphereSDFGPUResource*>(sdf_resource))
    {
        sphereMeshCollisionDetection<<<num_blocks, block_size>>>(sphere_sdf_resource->gpuSDF(),
                                                                 mesh_resource->gpuVertices(),
                                                                 num_vertices,
                                                                 mesh_resource->gpuFaces(),
                                                                 num_faces,
                                                                 collisions_resource->gpuArr());
        // sphereMeshCollisionDetectionParallel<<<num_blocks, block_size>>>(sphere_sdf_resource->gpuSDF(),
        //                                                          mesh_resource->gpuVertices(),
        //                                                          num_vertices,
        //                                                          mesh_resource->gpuFaces(),
        //                                                          num_faces,
        //                                                          collision_resource->gpuArr());
        gpuErrchk(cudaPeekAtLastError());
        // remove later, but here for testing
        // gpuErrchk(cudaDeviceSynchronize());
    }
    else if (const Sim::BoxSDFGPUResource* box_sdf_resource = dynamic_cast<const Sim::BoxSDFGPUResource*>(sdf_resource))
    {

    }
    else if (const Sim::CylinderSDFGPUResource* cyl_sdf_resource = dynamic_cast<const Sim::CylinderSDFGPUResource*>(sdf_resource))
    {

    }

    
}

int main(void)
{
    gmsh::initialize();

    Geometry::TetMesh mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/cube/cube16.msh");
    mesh.resize(1.0);
    mesh.moveTogether(Vec3r(-0.5, -0.5, 0.499));
    // mesh.moveTogether(Vec3r(0, 0, 0.5));
    mesh.createGPUResource();
    const Sim::MeshGPUResource* mesh_gpu_resource = dynamic_cast<const Sim::MeshGPUResource*>(mesh.gpuResource());
    mesh_gpu_resource->copyToDevice();

    Sim::RigidSphere sphere(nullptr, "sphere1", Vec3r(0,0,0), Vec4r(0,0,0,1), 0.5, 100);
    Geometry::SphereSDF sdf(&sphere);
    sdf.createGPUResource();
    sdf.gpuResource()->copyToDevice();

    GPUCollision* collisions = new GPUCollision[mesh.numFaces()];
    std::unique_ptr<Sim::ArrayGPUResource<GPUCollision>> collisions_resource = std::make_unique<Sim::ArrayGPUResource<GPUCollision>>(collisions, mesh.numFaces());
    collisions_resource->allocate();

    // allocate SDF on device
    int block_size = 256;
    int num_blocks;
    num_blocks = (mesh.numFaces() + block_size - 1) / block_size;

    // warm up
    launchCollisionKernel(sdf.gpuResource(), mesh_gpu_resource, mesh.numVertices(), mesh.numFaces(), collisions_resource.get());

    cudaDeviceSynchronize();

    std::array<int, 100> nanosecs;
    for (int i = 0; i < 100; i++)
    {
        /////////////////////////////////////////////////////////////////////
        auto start2 = std::chrono::high_resolution_clock::now();

        mesh_gpu_resource->copyVerticesToDevice();
        sdf.gpuResource()->copyToDevice();

        launchCollisionKernel(sdf.gpuResource(), mesh_gpu_resource, mesh.numVertices(), mesh.numFaces(), collisions_resource.get());

        cudaDeviceSynchronize();

        collisions_resource->copyFromDevice();
        

        auto end2 = std::chrono::high_resolution_clock::now();
        auto nanosec2 = std::chrono::duration_cast<std::chrono::nanoseconds>(end2 - start2);
        // std::cout << "Elapsed time on GPU: " << nanosec2.count() << " ns" << std::endl;
        nanosecs[i] = nanosec2.count();
        /////////////////////////////////////////////////////////////////////
    }
    for (const auto& t : nanosecs)
    {
        std::cout << "Elapsed time on GPU: " << t << " ns\n";
    }

    for (int i = 0; i < mesh.numFaces(); i++)
    {
        if (collisions[i].penetration_dist < 0)
            std::cout << "dist " << i << ": " << collisions[i].penetration_dist << std::endl;
    }
    // auto start = std::chrono::high_resolution_clock::now();

    // num_blocks = num_faces;
    // sphereMeshCollisionDetectionParallel<<<num_blocks, block_size>>>(d_sdf, d_vertices, num_vertices, d_faces, num_faces);

    // // wait for GPU
    // cudaDeviceSynchronize();

    // auto end = std::chrono::high_resolution_clock::now();
    // auto nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    // std::cout << "Elapsed time on GPU (more parallel): " << nanosec.count() << " ns" << std::endl;

    ///////////////////////////////////////////////////////////////////

    auto start3 = std::chrono::high_resolution_clock::now();
    for (int face_index = 0; face_index < mesh.numFaces(); face_index++)
    {
        const Vec3i face = mesh.face(face_index);
        const Vec3r x1 = mesh.vertex(face[0]);
        const Vec3r x2 = mesh.vertex(face[1]);
        const Vec3r x3 = mesh.vertex(face[2]);
        // const float* x1 = mesh.vertices().data() + 3*face[0];
        // const float* x2 = mesh.vertices().data() + 3*face[1];
        // const float* x3 = mesh.vertices().data() + 3*face[2];

        Vec3r min_bary_coords(1, 0, 0);
        Vec3r min_p = x1;
        float min_dist = 1000;

        const int num_samples = 16;
        for (int i = 0; i <= num_samples; i++)
        {
            for (int j = 0; j <= num_samples; j++)
            {
                const float u = (float)i / num_samples;
                const float v = (float)j / num_samples;
                const float w = 1 - u - v;
                const Vec3r p = u*x1 + v*x2 + w*x3;
                const float dist = sdf.evaluate(p);
                
                if (dist < min_dist)
                {
                    min_dist = dist;
                    min_p = p;
                    min_bary_coords = Vec3r(u,v,w);
                }

            }
        }
    }
    auto end3 = std::chrono::high_resolution_clock::now();
    auto nanosec3 = std::chrono::duration_cast<std::chrono::nanoseconds>(end3 - start3);
    std::cout << "Elapsed time on CPU: " << nanosec3.count()/1000000 << " ms" << std::endl;

    delete[] collisions;
    

    return 0;
}