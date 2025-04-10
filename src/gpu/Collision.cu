#include "gpu/Collision.cuh"

#include "gpu/resource/WritableArrayGPUResource.hpp"
#include "gpu/resource/SphereSDFGPUResource.hpp"
#include "gpu/resource/BoxSDFGPUResource.hpp"
#include "gpu/resource/CylinderSDFGPUResource.hpp"
#include "gpu/resource/MeshSDFGPUResource.hpp"

#include "utils/CudaHelperMath.h"

#include <cuda_profiler_api.h>

__host__ void launchCollisionKernel(const Sim::HostReadableGPUResource* sdf_resource, const Sim::MeshGPUResource* mesh_resource, int num_vertices, int num_faces, Sim::WritableArrayGPUResource<Sim::GPUCollision>* collision_resource)
{
    const int block_size = 256;
    const int num_blocks = (num_faces + block_size - 1) / block_size;
    // const int num_blocks = num_faces;

    // spawn GPU kernel depending on the type of SDF being collided with
    if (const Sim::SphereSDFGPUResource* sphere_sdf_resource = dynamic_cast<const Sim::SphereSDFGPUResource*>(sdf_resource))
    {
        sphereMeshCollisionDetection<<<num_blocks, block_size>>>(sphere_sdf_resource->gpuSDF(),
                                                                 mesh_resource->gpuVertices(),
                                                                 num_vertices,
                                                                 mesh_resource->gpuFaces(),
                                                                 num_faces,
                                                                 collision_resource->gpuArr());
        // sphereMeshCollisionDetectionParallel<<<num_blocks, block_size>>>(sphere_sdf_resource->gpuSDF(),
        //                                                          mesh_resource->gpuVertices(),
        //                                                          num_vertices,
        //                                                          mesh_resource->gpuFaces(),
        //                                                          num_faces,
        //                                                          collision_resource->gpuArr());
        // CHECK_CUDA_ERROR(cudaPeekAtLastError());
        // remove later, but here for testing
        // CHECK_CUDA_ERROR(cudaDeviceSynchronize());
    }
    else if (const Sim::BoxSDFGPUResource* box_sdf_resource = dynamic_cast<const Sim::BoxSDFGPUResource*>(sdf_resource))
    {
        boxMeshCollisionDetection<<<num_blocks, block_size>>>(box_sdf_resource->gpuSDF(),
                                                                 mesh_resource->gpuVertices(),
                                                                 num_vertices,
                                                                 mesh_resource->gpuFaces(),
                                                                 num_faces,
                                                                 collision_resource->gpuArr());
        CHECK_CUDA_ERROR(cudaPeekAtLastError());
    }
    else if (const Sim::CylinderSDFGPUResource* cyl_sdf_resource = dynamic_cast<const Sim::CylinderSDFGPUResource*>(sdf_resource))
    {
        cylinderMeshCollisionDetection<<<num_blocks, block_size>>>(cyl_sdf_resource->gpuSDF(),
                                                                 mesh_resource->gpuVertices(),
                                                                 num_vertices,
                                                                 mesh_resource->gpuFaces(),
                                                                 num_faces,
                                                                 collision_resource->gpuArr());
        CHECK_CUDA_ERROR(cudaPeekAtLastError());
    }
    else if (const Sim::MeshSDFGPUResource* mesh_sdf_resource = dynamic_cast<const Sim::MeshSDFGPUResource*>(sdf_resource))
    {
        meshMeshCollisionDetection<<<num_blocks, block_size>>>(mesh_sdf_resource->gpuSDF(),
                                                                 mesh_resource->gpuVertices(),
                                                                 num_vertices,
                                                                 mesh_resource->gpuFaces(),
                                                                 num_faces,
                                                                 collision_resource->gpuArr());
        CHECK_CUDA_ERROR(cudaPeekAtLastError());
    }

    
}

__device__ void global_to_body(const float3& x, const float3& body_position, const float4& body_orientation, float3& x_body)
{
    const float3 diff = x - body_position;
    const float3 neg_quat3 = make_float3(-body_orientation.x, -body_orientation.y, -body_orientation.z);
    x_body = 2*dot(diff,neg_quat3)*neg_quat3 + (2*body_orientation.w*body_orientation.w - 1)*diff + 2*body_orientation.w*cross(neg_quat3, diff);
}

__device__ float sphere_sdf_distance(const Sim::GPUSphereSDF* sphere_sdf, const float3& x)
{
    return length(x - sphere_sdf->position) - sphere_sdf->radius;
}

__device__ float3 sphere_sdf_gradient(const Sim::GPUSphereSDF* sphere_sdf, const float3& x)
{
    float3 grad =  normalize(x - sphere_sdf->position);
    return grad;
}

__device__ float box_sdf_distance(const Sim::GPUBoxSDF* box_sdf, const float3& x)
{
    float3 x_body;
    global_to_body(x, box_sdf->position, box_sdf->orientation, x_body);

    const float3 q = fabs(x_body) - 0.5 * box_sdf->size;
    const float q0 = fmaxf(q.x, 0.0);
    const float q1 = fmaxf(q.y, 0.0);
    const float q2 = fmaxf(q.z, 0.0);

    const float max_q = fmaxf(fmaxf(q.x, q.y), q.z);

    const float dist_when_outside = sqrtf(q0*q0 + q1*q1 + q2*q2);
    const float dist_when_inside = fminf(max_q, 0.0);

    return dist_when_outside + dist_when_inside;
}

__device__ float cyl_sdf_distance(const Sim::GPUCylinderSDF* cyl_sdf, const float3& x)
{
    float3 x_body;
    global_to_body(x, cyl_sdf->position, cyl_sdf->orientation, x_body);

    const float xy_dist = sqrtf(x_body.x*x_body.x + x_body.y*x_body.y) - cyl_sdf->radius;
    const float z_dist = fabs(x_body.z) - 0.5 * cyl_sdf->height;

    const float outside_dist_xy = fmaxf(xy_dist, 0.0);
    const float outside_dist_z = fmaxf(z_dist, 0.0);

    const float dist_when_outside = sqrtf(outside_dist_xy*outside_dist_xy + outside_dist_z*outside_dist_z);
    const float dist_when_inside = fminf(fmaxf(xy_dist, z_dist), 0.0);

    return dist_when_inside + dist_when_outside;
}

__device__ float mesh_sdf_distance(const Sim::GPUMeshSDF* mesh_sdf, const float3& x)
{
    float3 x_body;
    global_to_body(x, mesh_sdf->position, mesh_sdf->orientation, x_body);

    const float3 grid_ijk = (x_body - mesh_sdf->grid_bbox_min) / mesh_sdf->grid_cell_size;
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

    const float c00 = c000*(1-ijkd.x) + c100*ijkd.x;
    const float c01 = c001*(1-ijkd.x) + c101*ijkd.x;
    const float c10 = c010*(1-ijkd.x) + c110*ijkd.x;
    const float c11 = c011*(1-ijkd.x) + c111*ijkd.x;
    const float c0 = c00*(1-ijkd.y) + c10*ijkd.y;
    const float c1 = c01*(1-ijkd.y) + c11*ijkd.y;
    const float c = c0*(1-ijkd.z) + c1*ijkd.z;
    return c;
}

__global__ void sphereMeshCollisionDetection(const Sim::GPUSphereSDF* sphere_sdf, const float* vertices, int num_vertices, const int* faces, int num_faces, Sim::GPUCollision* collisions)
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
    collisions[face_index].penetration_dist = min_dist;
    // collisions[face_index].normal = sphere_sdf_gradient(sphere_sdf, min_p);
    collisions[face_index].bary_coords = min_bary_coords;
    // collisions[face_index].surface_point = min_p - collisions[face_index].normal * min_dist;
}

__global__ void sphereMeshCollisionDetectionParallel(const Sim::GPUSphereSDF* sphere_sdf, const float* vertices, int num_vertices, const int* faces, int num_faces, Sim::GPUCollision* collisions)
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

__global__ void boxMeshCollisionDetection(const Sim::GPUBoxSDF* box_sdf, const float* vertices, int num_vertices, const int* faces, int num_faces, Sim::GPUCollision* collisions)
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

    const int num_samples = 16;
    for (int i = 0; i <= num_samples; i++)
    {
        for (int j = 0; j <= num_samples - i; j++)
        {
            const float u = (float)i / num_samples;
            const float v = (float)j / num_samples;
            const float w = 1 - u - v;
            float3 p = u*x1 + v*x2 + w*x3;
            const float dist = box_sdf_distance(box_sdf, p);

            if (dist < min_dist)
            {
                min_dist = dist;
                min_p = p;
                min_bary_coords = make_float3(u, v, w);
            }
        }
    }

    // TODO: stream compaction
    collisions[face_index].penetration_dist = min_dist;
    collisions[face_index].bary_coords = min_bary_coords;
}

__global__ void cylinderMeshCollisionDetection(const Sim::GPUCylinderSDF* cyl_sdf, const float* vertices, int num_vertices, const int* faces, int num_faces, Sim::GPUCollision* collisions)
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

    const int num_samples = 16;
    for (int i = 0; i <= num_samples; i++)
    {
        for (int j = 0; j <= num_samples - i; j++)
        {
            const float u = (float)i / num_samples;
            const float v = (float)j / num_samples;
            const float w = 1 - u - v;
            float3 p = u*x1 + v*x2 + w*x3;
            const float dist = cyl_sdf_distance(cyl_sdf, p);

            if (dist < min_dist)
            {
                min_dist = dist;
                min_p = p;
                min_bary_coords = make_float3(u, v, w);
            }
        }
    }

    // TODO: stream compaction
    collisions[face_index].penetration_dist = min_dist;
    collisions[face_index].bary_coords = min_bary_coords;
}

__global__ void meshMeshCollisionDetection(const Sim::GPUMeshSDF* mesh_sdf, const float* vertices, int num_vertices, const int* faces, int num_faces, Sim::GPUCollision* collisions)
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