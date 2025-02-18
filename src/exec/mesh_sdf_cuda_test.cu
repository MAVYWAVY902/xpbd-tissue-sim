#include <iostream>
#include <math.h>

#include <chrono>
#include <thread>

#include "utils/CudaHelperMath.h"

#include "common/types.hpp"

#include "utils/MeshUtils.hpp"
#include "geometry/Mesh.hpp"
#include "geometry/MeshSDF.hpp"
#include "simobject/RigidPrimitives.hpp"
#include "simobject/RigidMeshObject.hpp"
#include "gpu/GPUStructs.hpp"
#include "gpu/GPUResource.hpp"
#include "gpu/SphereSDFGPUResource.hpp"
#include "gpu/BoxSDFGPUResource.hpp"
#include "gpu/MeshGPUResource.hpp"
#include "gpu/CylinderSDFGPUResource.hpp"
#include "gpu/ArrayGPUResource.hpp"
#include "gpu/MeshSDFGPUResource.hpp"

#include "config/RigidMeshObjectConfig.hpp"

#include <Mesh2SDF/array3.hpp>

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


__device__ float mesh_sdf_distance(const Sim::GPUMeshSDF* mesh_sdf, const float3& x)
{
    const float3 grid_ijk = (x - mesh_sdf->grid_bbox_min) / mesh_sdf->grid_cell_size;
    int3 ijk0 = make_int3(floorf(grid_ijk));
    int3 ijk1 = ijk0 + 1;
    const float3 ijkd = grid_ijk - make_float3(ijk0);

    // clamp to the grid border
    ijk0.x = clamp(ijk0.x, 0, mesh_sdf->grid_dims.x-1);
    ijk0.y = clamp(ijk0.y, 0, mesh_sdf->grid_dims.y-1);
    ijk0.z = clamp(ijk0.z, 0, mesh_sdf->grid_dims.z-1);

    ijk1.x = clamp(ijk1.x, 0, mesh_sdf->grid_dims.x-1);
    ijk1.y = clamp(ijk1.y, 0, mesh_sdf->grid_dims.y-1);
    ijk1.z = clamp(ijk1.z, 0, mesh_sdf->grid_dims.z-1);

    // printf("i0: %i j0: %i k0: %i\n", ijk0.x, ijk0.y, ijk0.z);
    // printf("i1: %i j1: %i k1: %i\n", ijk1.x, ijk1.y, ijk1.z);
    // printf("id: %f jd: %f kd: %f\n", ijkd.x, ijkd.y, ijkd.z);

    // get distance from trilinear interpolation
    const char* devPtr = (const char*)mesh_sdf->dev_dist_grid_ptr.ptr;
    size_t pitch = mesh_sdf->dev_dist_grid_ptr.pitch;
    size_t slice_pitch = pitch * mesh_sdf->grid_dims.z;

    const char* slice = devPtr + ijk0.z * slice_pitch;
    const float* row = (const float*)(slice + ijk0.y * pitch);
    const float c000 = row[ijk0.x];
    const float c100 = row[ijk1.x];

    row = (const float*)(slice + ijk1.y * pitch);
    const float c010 = row[ijk0.x];
    const float c110 = row[ijk1.x];

    slice = devPtr + ijk1.z * slice_pitch;
    row = (const float*)(slice + ijk0.y * pitch);
    const float c001 = row[ijk0.x];
    const float c101 = row[ijk1.x];
    
    row = (const float*)(slice + ijk1.y * pitch);
    const float c011 = row[ijk0.x];
    const float c111 = row[ijk1.x];

    // printf("c000: %f\n", c000);
    // printf("c100: %f\n", c100);
    // printf("c010: %f\n", c010);
    // printf("c110: %f\n", c110);
    // printf("c001: %f\n", c001);
    // printf("c101: %f\n", c101);
    // printf("c011: %f\n", c011);
    // printf("c111: %f\n", c111);

    const float c00 = c000*(1-ijkd.x) + c100*ijkd.x;
    const float c01 = c001*(1-ijkd.x) + c101*ijkd.x;
    const float c10 = c010*(1-ijkd.x) + c110*ijkd.x;
    const float c11 = c011*(1-ijkd.x) + c111*ijkd.x;
    const float c0 = c00*(1-ijkd.y) + c10*ijkd.y;
    const float c1 = c01*(1-ijkd.y) + c11*ijkd.y;
    const float c = c0*(1-ijkd.z) + c1*ijkd.z;

    // printf("c00: %f\n", c00);
    // printf("c01: %f\n", c01);
    // printf("c10: %f\n", c10);
    // printf("c11: %f\n", c11);
    // printf("c0: %f\n", c0);
    // printf("c1: %f\n", c1);
    // printf("c: %f\n", c);

    return c;
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

    collisions[face_index].penetration_dist = min_dist;
    collisions[face_index].bary_coords = min_bary_coords;
    
}

__global__ void meshMeshCollisionDetection(const Sim::GPUMeshSDF* mesh_sdf, const float* vertices, int num_vertices, const int* faces, int num_faces, GPUCollision* collisions)
{
    int face_index = blockIdx.x * blockDim.x + threadIdx.x;
    if (face_index >= num_faces)    return;
    const int* face = faces + 3*face_index;
    float3 x1, x2, x3;
    x1.x = (vertices + 3*face[0])[0];  x1.y = (vertices + 3*face[0])[1]; x1.z = (vertices + 3*face[0])[2];
    x2.x = (vertices + 3*face[1])[0];  x2.y = (vertices + 3*face[1])[1]; x2.z = (vertices + 3*face[1])[2];
    x3.x = (vertices + 3*face[2])[0];  x3.y = (vertices + 3*face[2])[1]; x3.z = (vertices + 3*face[2])[2];

    float3 min_p = x1;
    float3 min_bary_coords = make_float3(1.0, 0.0, 0.0);
    float min_dist = 1000;

    const int num_samples = 8;
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
            const float dist = mesh_sdf_distance(mesh_sdf, p);

            if (dist < min_dist)
            {
                min_dist = dist;
                min_p = p;
                min_bary_coords = make_float3(u, v, w);
            }
        }
    }

    collisions[face_index].penetration_dist = min_dist;
    collisions[face_index].bary_coords = min_bary_coords;
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
        CHECK_CUDA_ERROR(cudaPeekAtLastError());
        // remove later, but here for testing
        // CHECK_CUDA_ERROR(cudaDeviceSynchronize());
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
    mesh_gpu_resource->fullCopyToDevice();

    // TODO: fix whatever is going on here
    RigidMeshObjectConfig rigid_mesh_obj_config("rigid_mesh_obj", Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), 100, true, true,
                                                "../resource/general/stanford_bunny_lowpoly.stl", std::optional<Real>(1), std::optional<Vec3r>(),
                                                 false, true, true, Vec4r(0.5, 0.5, 0.5, 0.5), std::optional<std::string>());
    Sim::RigidMeshObject rigid_mesh_obj(nullptr, &rigid_mesh_obj_config);
    rigid_mesh_obj.setup(); // call setup() to load the mesh from file

    Geometry::MeshSDF rigid_mesh_sdf(&rigid_mesh_obj, &rigid_mesh_obj_config);
    rigid_mesh_sdf.createGPUResource();

    const Sim::MeshSDFGPUResource* mesh_sdf_gpu_resource = dynamic_cast<const Sim::MeshSDFGPUResource*>(rigid_mesh_sdf.gpuResource());
    mesh_sdf_gpu_resource->fullCopyToDevice();

    GPUCollision* collisions = new GPUCollision[mesh.numFaces()];
    std::unique_ptr<Sim::ArrayGPUResource<GPUCollision>> collisions_resource = std::make_unique<Sim::ArrayGPUResource<GPUCollision>>(collisions, mesh.numFaces());
    collisions_resource->allocate();

    // allocate SDF on device
    int block_size = 256;
    int num_blocks;
    num_blocks = (mesh.numFaces() + block_size - 1) / block_size;

    // warm up
    meshMeshCollisionDetection<<<num_blocks,block_size>>>(mesh_sdf_gpu_resource->gpuSDF(), mesh_gpu_resource->gpuVertices(), mesh.numVertices(), mesh_gpu_resource->gpuFaces(), mesh.numFaces(), collisions_resource->gpuArr());
    
    cudaDeviceSynchronize();

    std::array<int, 100> nanosecs;
    for (int i = 0; i < 100; i++)
    {
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
        /////////////////////////////////////////////////////////////////////
        auto start2 = std::chrono::high_resolution_clock::now();

        mesh_sdf_gpu_resource->partialCopyToDevice();
        mesh_gpu_resource->partialCopyToDevice();
        

        meshMeshCollisionDetection<<<num_blocks,block_size>>>(mesh_sdf_gpu_resource->gpuSDF(), mesh_gpu_resource->gpuVertices(), mesh.numVertices(), mesh_gpu_resource->gpuFaces(), mesh.numFaces(), collisions_resource->gpuArr());

        // cudaDeviceSynchronize();

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

    ///////////////////////////////////////////////////////////////////////////////////
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

        const int num_samples = 8;
        for (int i = 0; i <= num_samples; i++)
        {
            for (int j = 0; j <= num_samples; j++)
            {
                const float u = (float)i / num_samples;
                const float v = (float)j / num_samples;
                const float w = 1 - u - v;
                const Vec3r p = u*x1 + v*x2 + w*x3;
                const float dist = rigid_mesh_sdf.evaluate(p);
                
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

    return 0;
}