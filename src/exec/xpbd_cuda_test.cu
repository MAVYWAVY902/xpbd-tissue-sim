#include <iostream>
#include <math.h>

#include <chrono>
#include <thread>

#include "utils/CudaHelperMath.h"

#include "common/types.hpp"

#include "utils/MeshUtils.hpp"
#include "geometry/Mesh.hpp"
#include "geometry/SphereSDF.hpp"
#include "simobject/XPBDMeshObject.hpp"
#include "config/XPBDMeshObjectConfig.hpp"

#include "gpu/GPUStructs.hpp"
#include "gpu/GPUResource.hpp"
#include "gpu/TetMeshGPUResource.hpp"
#include "gpu/ArrayGPUResource.hpp"
#include "gpu/WritableArrayGPUResource.hpp"

// computes A * B^T and stores result in C
__host__ __device__ void Mat3MulTranspose(const float* A, const float* B, float* C)
{
    C[0] = A[0]*B[0] + A[3]*B[3] + A[6]*B[6];
    C[1] = A[1]*B[0] + A[4]*B[3] + A[7]*B[6];
    C[2] = A[2]*B[0] + A[5]*B[3] + A[8]*B[6];

    C[3] = A[0]*B[1] + A[3]*B[4] + A[6]*B[7];
    C[4] = A[1]*B[1] + A[4]*B[4] + A[7]*B[7];
    C[5] = A[2]*B[1] + A[5]*B[4] + A[8]*B[7];

    C[6] = A[0]*B[2] + A[3]*B[5] + A[6]*B[8];
    C[7] = A[1]*B[2] + A[4]*B[5] + A[7]*B[8];
    C[8] = A[2]*B[2] + A[5]*B[5] + A[8]*B[8];
}

// computes A * B and stores result in C
__host__ __device__ void Mat3Mul(const float* A, const float* B, float* C)
{
    C[0] = A[0]*B[0] + A[3]*B[1] + A[6]*B[2];
    C[1] = A[1]*B[0] + A[4]*B[1] + A[7]*B[2];
    C[2] = A[2]*B[0] + A[5]*B[1] + A[8]*B[2];

    C[3] = A[0]*B[3] + A[3]*B[4] + A[6]*B[5];
    C[4] = A[1]*B[3] + A[4]*B[4] + A[7]*B[5];
    C[5] = A[2]*B[3] + A[5]*B[4] + A[8]*B[5];

    C[6] = A[0]*B[6] + A[3]*B[7] + A[6]*B[8];
    C[7] = A[1]*B[6] + A[4]*B[7] + A[7]*B[8];
    C[8] = A[2]*B[6] + A[5]*B[7] + A[8]*B[8];
}

__host__ __device__ void Vec3Cross(const float* v1, const float* v2, float* v3)
{
    v3[0] = v1[1]*v2[2] - v1[2]*v2[1];
    v3[1] = v1[2]*v2[0] - v1[0]*v2[2];
    v3[2] = v1[0]*v2[1] - v1[1]*v2[0];
}

__host__ __device__ void ElementJacobiSolve(int elem_index, const int* elements, int num_elements, const float* vertices, const float* masses, const float* volumes, const float* Qs, float lambda, float mu, float dt, float* coord_updates)
{
    // extract quantities for element
    const int* elem = elements + 4*elem_index;

    float3 x1, x2, x3, x4;
    x1.x = (vertices + 3*elem[0])[0];  x1.y = (vertices + 3*elem[0])[1]; x1.z = (vertices + 3*elem[0])[2];
    x2.x = (vertices + 3*elem[1])[0];  x2.y = (vertices + 3*elem[1])[1]; x2.z = (vertices + 3*elem[1])[2];
    x3.x = (vertices + 3*elem[2])[0];  x3.y = (vertices + 3*elem[2])[1]; x3.z = (vertices + 3*elem[2])[2];
    x4.x = (vertices + 3*elem[3])[0];  x4.y = (vertices + 3*elem[3])[1]; x4.z = (vertices + 3*elem[3])[2];

    // float inv_m1 = 1.0/masses[3*elem[0]];
    // float inv_m2 = 1.0/masses[3*elem[1]];
    // float inv_m3 = 1.0/masses[3*elem[2]];
    // float inv_m4 = 1.0/masses[3*elem[3]];

    float inv_m[4];
    for (int i = 0; i < 4; i++) { inv_m[i] = 1.0/masses[3*elem[i]]; }

    const float* Q = Qs + 9*elem_index;
    const float gamma = mu / lambda;

    // compute F
    float X[9];
    X[0] = x1.x - x4.x; X[1] = x1.y - x4.y; X[2] = x1.z - x4.z;
    X[3] = x2.x - x4.x; X[4] = x2.y - x4.y; X[5] = x2.z - x4.z;
    X[6] = x3.x - x4.x; X[7] = x3.y - x4.y; X[8] = x3.z - x4.z;

    float F[9];
    Mat3Mul(X, Q, F);

    // compute hydrostatic constraint and its gradient
    float C_h_grad[12];

    // C_h = det(F) - (1 + gamma)
    const float C_h = F[0]*F[4]*F[8] - F[0]*F[7]*F[5] - F[3]*F[1]*F[8] + F[3]*F[7]*F[2] + F[6]*F[1]*F[5] - F[6]*F[4]*F[2] - (1+gamma);

    const float alpha_h = 1.0/(lambda * volumes[elem_index]);

    float F_cross[9];
    Vec3Cross(F+3, F+6, F_cross);   // 2nd column of F crossed with 3rd column
    Vec3Cross(F+6, F, F_cross+3);   // 3rd column of F crossed with 1st column
    Vec3Cross(F, F+3, F_cross+6);   // 1st column of F crossed with 2nd column

    Mat3MulTranspose(F_cross, Q, C_h_grad);
    C_h_grad[9] = -C_h_grad[0] - C_h_grad[3] - C_h_grad[6];
    C_h_grad[10] = -C_h_grad[1] - C_h_grad[4] - C_h_grad[7];
    C_h_grad[11] = -C_h_grad[2] - C_h_grad[5] - C_h_grad[8];


    // compute deviatoric constraint and its gradient
    float C_d_grad[12];

    // C_d = frob(F)
    const float C_d = sqrtf(F[0]*F[0] + F[1]*F[1] + F[2]*F[2] + F[3]*F[3] + F[4]*F[4] + F[5]*F[5] + F[6]*F[6] + F[7]*F[7] + F[8]*F[8]);
    const float inv_C_d = 1.0/C_d;

    const float alpha_d = 1.0/(mu * volumes[elem_index]);

    Mat3MulTranspose(F, Q, C_d_grad);
    for (int i = 0; i < 9; i++) { C_d_grad[i] *= inv_C_d; }
    C_d_grad[9] =  -C_d_grad[0] - C_d_grad[3] - C_d_grad[6];
    C_d_grad[10] = -C_d_grad[1] - C_d_grad[4] - C_d_grad[7];
    C_d_grad[11] = -C_d_grad[2] - C_d_grad[5] - C_d_grad[8];


    // solve the 2x2 system
    float A[4];
    A[0] = alpha_h / (dt * dt);
    A[1] = 0;
    A[2] = 0;
    A[3] = alpha_d / (dt * dt);
    for (int i = 0; i < 4; i++)
    {
        A[0] += inv_m[i] * (C_h_grad[3*i]*C_h_grad[3*i] + C_h_grad[3*i+1]*C_h_grad[3*i+1] + C_h_grad[3*i+2]*C_h_grad[3*i+2]);
        A[3] += inv_m[i] * (C_d_grad[3*i]*C_d_grad[3*i] + C_d_grad[3*i+1]*C_d_grad[3*i+1] + C_d_grad[3*i+2]*C_d_grad[3*i+2]);
        A[1] += inv_m[i] * (C_h_grad[3*i]*C_d_grad[3*i] + C_h_grad[3*i+1]*C_d_grad[3*i+1] + C_h_grad[3*i+2]*C_d_grad[3*i+2]);
    }
    A[2] = A[1];

    float k[2];
    k[0] = -C_h; // TODO: include current lambda (assuming its 0 right now)
    k[1] = -C_d; // TODO: include current lambda

    const float detA = A[0]*A[3] - A[1]*A[2];
    const float dlam_h = (k[0]*A[3] - k[1]*A[2]) / detA;
    const float dlam_d = (k[1]*A[0] - k[0]*A[1]) / detA;

    // compute the coordinate updates
    for (int i = 0; i < 4; i++)
    {
        coord_updates[12*elem_index + 3*i] =   inv_m[i] * (C_h_grad[3*i] * dlam_h + C_d_grad[3*i] * dlam_d);
        coord_updates[12*elem_index + 3*i+1] = inv_m[i] * (C_h_grad[3*i+1] * dlam_h + C_d_grad[3*i+1] * dlam_d);
        coord_updates[12*elem_index + 3*i+2] = inv_m[i] * (C_h_grad[3*i+2] * dlam_h + C_d_grad[3*i+2] * dlam_d);
    }
}

__global__ void XPBDJacobiSolve(const int* elements, int num_elements, const float* vertices, const float* masses, const float* volumes, const float* Qs, float lambda, float mu, float dt, float* coord_updates)
{
    int elem_index = blockIdx.x * blockDim.x + threadIdx.x;
    if (elem_index >= num_elements)    return;

    for (int gi = 0; gi < 1; gi++)
    {
        ElementJacobiSolve(elem_index, elements, num_elements, vertices, masses, volumes, Qs, lambda, mu, dt, coord_updates);
    }

}

int main(void)
{
    gmsh::initialize();

    // MeshUtils::createBeamObj("../resource/cube/cube32.obj", 1, 1, 1, 32);
    // MeshUtils::convertToSTL("../resource/cube/cube32.obj");
    // MeshUtils::convertSTLtoMSH("../resource/cube/cube32.stl");

    Geometry::TetMesh mesh = MeshUtils::loadTetMeshFromGmshFile("../resource/cube/cube16.msh");
    mesh.resize(1.0);
    mesh.moveTogether(Vec3r(-0.5, -0.5, 0.499));
    // mesh.moveTogether(Vec3r(0, 0, 0.5));
    mesh.createGPUResource();
    const Sim::TetMeshGPUResource* mesh_gpu_resource = dynamic_cast<const Sim::TetMeshGPUResource*>(mesh.gpuResource());
    mesh_gpu_resource->fullCopyToDevice();


    float density = 1000;
    float lambda = 1e7;
    float mu = 1e6;

    // precompute quantities
    std::vector<float> volumes(mesh.numElements());
    std::vector<float> Qs(mesh.numElements()*9);
    std::vector<float> masses(mesh.numVertices());
    for (int i = 0; i < mesh.numElements(); i++)
    {
        const Eigen::Vector4i elem = mesh.element(i);
        Eigen::Matrix3f X;
        X.col(0) = mesh.vertex(elem[0]) - mesh.vertex(elem[3]);
        X.col(1) = mesh.vertex(elem[1]) - mesh.vertex(elem[3]);
        X.col(2) = mesh.vertex(elem[2]) - mesh.vertex(elem[3]);

        Eigen::Matrix3f Q = X.inverse();
        for (int k = 0; k < 9; k++) { Qs[9*i+k] = Q.data()[k]; }

        volumes[i] = std::abs(X.determinant()/6.0);

        float element_mass = volumes[i] * density;
        for (int k = 0; k < 4; k++) { masses[elem[k]] += element_mass / 4.0; }
    }

    Sim::ArrayGPUResource<float> volumes_resource(volumes.data(), volumes.size());
    volumes_resource.allocate();
    volumes_resource.fullCopyToDevice();
    Sim::ArrayGPUResource<float> Qs_resource(Qs.data(), Qs.size());
    Qs_resource.allocate();
    Qs_resource.fullCopyToDevice();
    Sim::ArrayGPUResource<float> masses_resource(masses.data(), masses.size());
    masses_resource.allocate();
    masses_resource.fullCopyToDevice();

    std::vector<float> coord_updates(mesh.numElements()*12);
    Sim::WritableArrayGPUResource<float> coord_updates_resource(coord_updates.data(), coord_updates.size());
    coord_updates_resource.allocate();

    // allocate SDF on device
    int block_size = 256;
    int num_blocks;
    num_blocks = (mesh.numElements() + block_size - 1) / block_size;

    std::array<int, 100> nanosecs;
    for (int i = 0; i < 100; i++)
    {
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
        /////////////////////////////////////////////////////////////////////
        auto start2 = std::chrono::high_resolution_clock::now();

        mesh_gpu_resource->partialCopyToDevice();
        

        XPBDJacobiSolve<<<num_blocks, block_size>>>(mesh_gpu_resource->gpuElements(), mesh.numElements(), mesh_gpu_resource->gpuVertices(),
                                                     masses_resource.gpuArr(), volumes_resource.gpuArr(), Qs_resource.gpuArr(),
                                                    lambda, mu, 1e-3,
                                                    coord_updates_resource.gpuArr());
        CHECK_CUDA_ERROR(cudaPeekAtLastError());

        CHECK_CUDA_ERROR(cudaDeviceSynchronize());
        

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

    /////////////////////////////////////////////////////////////////////////

    auto start3 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < mesh.numElements(); i++)
    {
        ElementJacobiSolve(i, mesh.elements().data(), mesh.numElements(), mesh.vertices().data(),
                            masses.data(), volumes.data(), Qs.data(),
                            lambda, mu, 1e-3,
                            coord_updates.data());
    }
    auto end3 = std::chrono::high_resolution_clock::now();
    auto nanosec3 = std::chrono::duration_cast<std::chrono::nanoseconds>(end3 - start3);
    std::cout << "Elapsed time on CPU: " << nanosec3.count()/1000000 << " ms" << std::endl;

    ///////////////////////////////////////////////////////////////////
    

    return 0;
}