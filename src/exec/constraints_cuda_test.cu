#include <iostream>
#include <math.h>
#include <chrono>
#include <thread>

#include "utils/CudaHelperMath.h"

#include "common/types.hpp"

#include "config/SimulationConfig.hpp"
#include "config/XPBDMeshObjectConfig.hpp"

#include "simulation/Simulation.hpp"

#include "simobject/XPBDMeshObject.hpp"

#include "gpu/GPUStructs.hpp"
#include "gpu/ArrayGPUResource.hpp"

//////////////////////////////////////////////////////////////////////////////
// Helper functions
//////////////////////////////////////////////////////////////////////////////


// computes A * B^T and stores result in C
__host__ __device__ void MyMat3MulTranspose(const float* A, const float* B, float* C)
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
__host__ __device__ void MyMat3Mul(const float* A, const float* B, float* C)
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

__host__ __device__ void MyVec3Cross(const float* v1, const float* v2, float* v3)
{
    v3[0] = v1[1]*v2[2] - v1[2]*v2[1];
    v3[1] = v1[2]*v2[0] - v1[0]*v2[2];
    v3[2] = v1[0]*v2[1] - v1[1]*v2[0];
}

////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////

struct GPUPositionReference
{
    float* ptr; // pointer to the vertex in GPU memory
    float inv_mass; // inverse mass of the vertex
    int index; // vertex index
};

struct GPUElementConstraint
{
    __host__ static void computeQandVolume(const Eigen::Vector3f& v0, const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, const Eigen::Vector3f& v3, float* Q, float* rest_volume)
    {
        Eigen::Matrix3f X;
        X.col(0) = v0 - v3;
        X.col(1) = v1 - v3;
        X.col(2) = v2 - v3;

        Eigen::Matrix3f Q_mat = X.inverse();
        for (int k = 0; k < 9; k++) { Q[k] = Q_mat.data()[k]; }

        *rest_volume = std::abs(X.determinant()/6.0);
    }

    __device__ static void computeF(const float* x, const float* Q, float* F)
    {
        float X[9];
        X[0] = x[0] - x[9]; X[1] = x[1] - x[10]; X[2] = x[2] - x[11];
        X[3] = x[3] - x[9]; X[4] = x[4] - x[10]; X[5] = x[5] - x[11];
        X[6] = x[6] - x[9]; X[7] = x[7] - x[10]; X[8] = x[8] - x[11];

        MyMat3Mul(X, Q, F);
    }
};

struct GPUHydrostaticConstraint
{
    GPUPositionReference positions[4];
    float Q[9];
    float rest_volume;
    float alpha;
    float gamma;

    __host__ GPUHydrostaticConstraint(Sim::XPBDMeshObject* xpbd_obj, int v0, int v1, int v2, int v3)
    {
        float* gpu_vertices = xpbd_obj->gpuResource()->meshGpuResource().gpuVertices();
        positions[0].ptr = gpu_vertices + v0*3;
        positions[0].inv_mass = 1 / xpbd_obj->vertexMass(v0);
        positions[0].index = v0;

        positions[1].ptr = gpu_vertices + v1*3;
        positions[1].inv_mass = 1 / xpbd_obj->vertexMass(v1);
        positions[1].index = v1;

        positions[2].ptr = gpu_vertices + v2*3;
        positions[2].inv_mass = 1 / xpbd_obj->vertexMass(v2);
        positions[2].index = v2;

        positions[3].ptr = gpu_vertices + v3*3;
        positions[3].inv_mass = 1 / xpbd_obj->vertexMass(v3);
        positions[3].index = v3;

        // compute Q and rest volume
        const Geometry::Mesh* mesh = xpbd_obj->mesh();
        GPUElementConstraint::computeQandVolume(mesh->vertex(v0), mesh->vertex(v1), mesh->vertex(v2), mesh->vertex(v3), Q, &rest_volume);

        alpha = 1/(xpbd_obj->material().lambda() * rest_volume);            // set alpha after the ElementConstraint constructor because we need the element volume
        gamma = xpbd_obj->material().mu() / xpbd_obj->material().lambda(); 
    }

    constexpr __host__ __device__ static int numPositions() { return 4; }

    __device__ void _loadVertices(float* x)
    {
        x[0] = positions[0].ptr[0];  x[1] = positions[0].ptr[1];  x[2] = positions[0].ptr[2];
        x[3] = positions[1].ptr[0];  x[4] = positions[1].ptr[1];  x[5] = positions[1].ptr[2];
        x[6] = positions[2].ptr[0];  x[7] = positions[2].ptr[1];  x[8] = positions[2].ptr[2];
        x[9] = positions[3].ptr[0];  x[10] = positions[3].ptr[1]; x[11] = positions[3].ptr[2];
    }

    __device__ void evaluate(float* C)
    {
        float x[12];
        _loadVertices(x);

        float F[9];
        GPUElementConstraint::computeF(x, Q, F);

        _evaluate(C, F);
        
    }

    __device__ void _evaluate(float* C, float* F)
    {
        *C = F[0]*F[4]*F[8] - F[0]*F[7]*F[5] - F[3]*F[1]*F[8] + F[3]*F[7]*F[2] + F[6]*F[1]*F[5] - F[6]*F[4]*F[2] - (1+gamma);
    }

    __device__ void gradient(float* delC)
    {
        float x[12];
        _loadVertices(x);

        float F[9];
        GPUElementConstraint::computeF(x, Q, F);

        _gradient(delC, F);
    }

    __device__ void _gradient(float* delC, float* F)
    {   
        float F_cross[9];
        MyVec3Cross(F+3, F+6, F_cross);   // 2nd column of F crossed with 3rd column
        MyVec3Cross(F+6, F, F_cross+3);   // 3rd column of F crossed with 1st column
        MyVec3Cross(F, F+3, F_cross+6);   // 1st column of F crossed with 2nd column

        MyMat3MulTranspose(F_cross, Q, delC);
        delC[9]  = -delC[0] - delC[3] - delC[6];
        delC[10] = -delC[1] - delC[4] - delC[7];
        delC[11] = -delC[2] - delC[5] - delC[8];
    }

    __device__ void evaluateWithGradient(float* C, float* delC)
    {
        float x[12];
        _loadVertices(x);

        float F[9];
        GPUElementConstraint::computeF(x, Q, F);

        _evaluate(C, F);
        _gradient(delC, F);
    }
};

struct GPUDeviatoricConstraint
{
    GPUPositionReference positions[4];
    float Q[9];
    float rest_volume;
    float alpha;

    __host__ GPUDeviatoricConstraint(Sim::XPBDMeshObject* xpbd_obj, int v0, int v1, int v2, int v3)
    {
        float* gpu_vertices = xpbd_obj->gpuResource()->meshGpuResource().gpuVertices();
        positions[0].ptr = gpu_vertices + v0*3;
        positions[0].inv_mass = 1 / xpbd_obj->vertexMass(v0);
        positions[0].index = v0;

        positions[1].ptr = gpu_vertices + v1*3;
        positions[1].inv_mass = 1 / xpbd_obj->vertexMass(v1);
        positions[1].index = v1;

        positions[2].ptr = gpu_vertices + v2*3;
        positions[2].inv_mass = 1 / xpbd_obj->vertexMass(v2);
        positions[2].index = v2;

        positions[3].ptr = gpu_vertices + v3*3;
        positions[3].inv_mass = 1 / xpbd_obj->vertexMass(v3);
        positions[3].index = v3;

        // compute Q and rest volume
        const Geometry::Mesh* mesh = xpbd_obj->mesh();
        float Q[9];
        float rest_volume;
        GPUElementConstraint::computeQandVolume(mesh->vertex(v0), mesh->vertex(v1), mesh->vertex(v2), mesh->vertex(v3), Q, &rest_volume);

        alpha = 1/(xpbd_obj->material().mu() * rest_volume);
    }

    constexpr __host__ __device__ static int numPositions() { return 4; } 

    __device__ void _loadVertices(float* x)
    {
        x[0] = positions[0].ptr[0];  x[1] = positions[0].ptr[1];  x[2] = positions[0].ptr[2];
        x[3] = positions[1].ptr[0];  x[4] = positions[1].ptr[1];  x[5] = positions[1].ptr[2];
        x[6] = positions[2].ptr[0];  x[7] = positions[2].ptr[1];  x[8] = positions[2].ptr[2];
        x[9] = positions[3].ptr[0];  x[10] = positions[3].ptr[1]; x[11] = positions[3].ptr[2];
    }

    __device__ void evaluate(float* C)
    {
        float x[12];
        _loadVertices(x);

        float F[9];
        GPUElementConstraint::computeF(x, Q, F);

        _evaluate(C, F);
    }

    __device__ void _evaluate(float* C, float* F)
    {
        *C = sqrtf(F[0]*F[0] + F[1]*F[1] + F[2]*F[2] + F[3]*F[3] + F[4]*F[4] + F[5]*F[5] + F[6]*F[6] + F[7]*F[7] + F[8]*F[8]);
    }

    __device__ void gradient(float* delC)
    {
        float x[12];
        _loadVertices(x);

        float F[9];
        GPUElementConstraint::computeF(x, Q, F);

        float C;
        _evaluate(&C, F);
        _gradient(delC, &C, F);
    }

    __device__ void _gradient(float* delC, float* C, float* F)
    {
        float inv_C_d = 1.0 / *C;

        MyMat3MulTranspose(F, Q, delC);
        for (int i = 0; i < 9; i++) { delC[i] *= inv_C_d; }
        delC[9] =  -delC[0] - delC[3] - delC[6];
        delC[10] = -delC[1] - delC[4] - delC[7];
        delC[11] = -delC[2] - delC[5] - delC[8];
    }

    __device__ void evaluateWithGradient(float* C, float* delC)
    {
        float x[12];
        _loadVertices(x);

        float F[9];
        GPUElementConstraint::computeF(x, Q, F);

        _evaluate(C, F);
        _gradient(delC, C, F);
    }
};

template<class Constraint>
struct GPUConstraintProjector
{
    float dt;
    float lambda;
    Constraint constraint;

    __host__ GPUConstraintProjector(const Constraint& constraint_, float dt_)
        : constraint(constraint_), dt(dt_)
    {

    } 
    __host__ GPUConstraintProjector(Constraint&& constraint_, float dt_)
        : constraint(std::move(constraint_)), dt(dt_)
    {
    }

    __device__ void initialize()
    {
        lambda = 0;
    }

    __device__ void project(float* new_vertices)
    {
        float C;
        float delC[Constraint::numPositions()*3];

        // evaluate constraint and its gradient
        constraint->evaluateWithGradient(&C, delC);

        float alpha_tilde = constraint->alpha / (dt*dt);

        // compute LHS of lambda upate - delC^T * M^-1 * delC
        float LHS = alpha_tilde;
        for (int i = 0; i < constraint->numPositions(); i++)
        {
            LHS += constraint->positions[i].inv_mass * (delC[3*i]*delC[3*i] + delC[3*i+1]*delC[3*i+1] + delC[3*i+2]*delC[3*i+2]);
        }

        // compute RHS of lambda update - -C - alpha_tilde * lambda
        float RHS = -C - alpha_tilde * lambda;

        // compute lambda update
        float dlam = RHS / LHS;
        lambda += dlam;

        // update positions
        for (int i = 0; i < 4; i++)
        {
            float* v_ptr = new_vertices + constraint->positions[i].index;
            atomicAdd(v_ptr,     constraint->positions[i].inv_mass * delC[3*i] * dlam);
            atomicAdd(v_ptr + 1, constraint->positions[i].inv_mass * delC[3*i+1] * dlam);
            atomicAdd(v_ptr + 2, constraint->positions[i].inv_mass * delC[3*i+2] * dlam);
        }
    }
};

template<class Constraint1, class Constraint2>
struct GPUCombinedConstraintProjector
{
    float dt;
    float lambda[2];
    Constraint1 constraint1;
    Constraint2 constraint2;

    __host__ GPUCombinedConstraintProjector(const Constraint1& constraint1_, const Constraint2& constraint2_, float dt_)
        : constraint1(constraint1_), constraint2(constraint2_), dt(dt_)
    {

    }
    
    __host__ GPUCombinedConstraintProjector(Constraint1&& constraint1_, Constraint2&& constraint2_, float dt_)
        : constraint1(std::move(constraint1_)), constraint2(std::move(constraint2_)), dt(dt_)
    {

    } 

    __device__ void initialize()
    {
        lambda[0] = 0;
        lambda[1] = 0;
    }

    __device__ void project(float* new_vertices)
    {
        float C[2];
        float delC[2*(Constraint1::numPositions()*3)];  // FOR NOW: we assume that both constraints share the same exact positions

        // evaluate constraint and its gradient
        constraint1.evaluateWithGradient(C, delC);
        constraint2.evaluateWithGradient(C + 1, delC + constraint1.numPositions()*3);

        float alpha_tilde[2] = {constraint1.alpha / (dt*dt), constraint2.alpha / (dt*dt)};

        // compute LHS of lambda upate - delC^T * M^-1 * delC
        float LHS[4];

        for (int ci = 0; ci < 2; ci++)
        {
            for (int cj = 0; cj < 2; cj++)
            {
                if (ci == cj) LHS[cj*2 + ci] = alpha_tilde[ci];
                else LHS[cj*2 + ci] = 0;
            }
        }

        for (int ci = 0; ci < 2; ci++)
        {
            float* delC_i = delC + ci*constraint1.numPositions()*3;
            for (int cj = ci; cj < 2; cj++)
            {
                float* delC_j = delC + cj*constraint1.numPositions()*3;

                for (int i = 0; i < constraint1.numPositions(); i++)
                {
                    LHS[cj*2 + ci] += constraint1.positions[i].inv_mass * (delC_i[3*i]*delC_j[3*i] + delC_i[3*i+1]*delC_j[3*i+1] + delC_i[3*i+2]*delC_j[3*i+2]);
                }

                LHS[ci*2 + cj] = LHS[cj*2 + ci];
            }
            
        }

        // compute RHS of lambda update - -C - alpha_tilde * lambda
        float RHS[2];
        for (int ci = 0; ci < 2; ci++)
        {
            RHS[ci] = -C[ci] - alpha_tilde[ci] * lambda[ci];
        }

        // compute lambda update - solve 2x2 system
        float dlam[2];
        const float det = LHS[0]*LHS[3] - LHS[1]*LHS[2];

        dlam[0] = (RHS[0]*LHS[3] - RHS[1]*LHS[2]) / det;
        dlam[1] = (RHS[1]*LHS[0] - RHS[0]*LHS[1]) / det;

        // update lambda
        lambda[0] += dlam[0];
        lambda[1] += dlam[1];

        // update positions
        float* delC_c2 = delC + constraint1.numPositions()*3;
        for (int i = 0; i < 4; i++)
        {
            float* v_ptr = new_vertices + constraint1.positions[i].index;
            atomicAdd(v_ptr,     constraint1.positions[i].inv_mass * (delC[3*i] * dlam[0] + delC_c2[3*i] * dlam[1]));
            atomicAdd(v_ptr + 1, constraint1.positions[i].inv_mass * (delC[3*i+1] * dlam[0] + delC_c2[3*i+1] * dlam[1]));
            atomicAdd(v_ptr + 2, constraint1.positions[i].inv_mass * (delC[3*i+2] * dlam[0] + delC_c2[3*i+2] * dlam[1]));
        }
    }
};

template <> void GPUCombinedConstraintProjector<GPUHydrostaticConstraint, GPUDeviatoricConstraint>::project(float* new_vertices)
{
    GPUHydrostaticConstraint l_hyd = constraint1;
    GPUDeviatoricConstraint l_dev = constraint2;

    float C[2];
    float delC[24];  // FOR NOW: we assume that both constraints share the same exact positions

    // evaluate constraint and its gradient
    float x[12];
    l_hyd._loadVertices(x);
    // for (int i = 0; i < 12; i++) {x[i] = 0;}

    float Q[9];
    for (int i = 0; i < 9; i++) { Q[i] = l_hyd.Q[i]; }

    float inv_m[4];
    for (int i = 0; i < 4; i++) { inv_m[i] = l_hyd.positions[i].inv_mass; }

    float alpha_tilde[2] = {l_hyd.alpha / (dt*dt), l_dev.alpha / (dt*dt)};

    float F[9];
    GPUElementConstraint::computeF(x, Q, F);

    l_hyd._evaluate(C, F);
    l_hyd._gradient(delC, F);
    l_dev._evaluate(C+1, F);
    l_dev._gradient(delC + 12, C+1, F);

    // compute LHS of lambda upate - delC^T * M^-1 * delC
    float LHS[4];

    for (int ci = 0; ci < 2; ci++)
    {
        for (int cj = 0; cj < 2; cj++)
        {
            if (ci == cj) LHS[cj*2 + ci] = alpha_tilde[ci];
            else LHS[cj*2 + ci] = 0;
        }
    }

    for (int ci = 0; ci < 2; ci++)
    {
        float* delC_i = delC + ci*12;
        for (int cj = ci; cj < 2; cj++)
        {
            float* delC_j = delC + cj*12;

            for (int i = 0; i < 4; i++)
            {
                LHS[cj*2 + ci] += inv_m[i] * (delC_i[3*i]*delC_j[3*i] + delC_i[3*i+1]*delC_j[3*i+1] + delC_i[3*i+2]*delC_j[3*i+2]);
            }

            LHS[ci*2 + cj] = LHS[cj*2 + ci];
        }
        
    }

    // compute RHS of lambda update - -C - alpha_tilde * lambda
    float RHS[2];
    for (int ci = 0; ci < 2; ci++)
    {
        RHS[ci] = -C[ci] - alpha_tilde[ci] * lambda[ci];
    }

    // compute lambda update - solve 2x2 system
    float dlam[2];
    const float det = LHS[0]*LHS[3] - LHS[1]*LHS[2];

    dlam[0] = (RHS[0]*LHS[3] - RHS[1]*LHS[2]) / det;
    dlam[1] = (RHS[1]*LHS[0] - RHS[0]*LHS[1]) / det;

    // update lambda
    lambda[0] += dlam[0];
    lambda[1] += dlam[1];

    // update positions
    float* delC_c2 = delC + 12;
    for (int i = 0; i < 4; i++)
    {
        float* v_ptr = new_vertices + l_hyd.positions[i].index;
        atomicAdd(v_ptr,     inv_m[i] * (delC[3*i] * dlam[0] + delC_c2[3*i] * dlam[1]));
        atomicAdd(v_ptr + 1, inv_m[i] * (delC[3*i+1] * dlam[0] + delC_c2[3*i+1] * dlam[1]));
        atomicAdd(v_ptr + 2, inv_m[i] * (delC[3*i+2] * dlam[0] + delC_c2[3*i+2] * dlam[1]));
    }
}


template<class ConstraintProjector>
__global__ void ProjectConstraints(ConstraintProjector* projectors, int num_projectors, float* new_vertices)
{
    int proj_index = blockIdx.x * blockDim.x + threadIdx.x;
    if (proj_index >= num_projectors)   return;

    // projectors[proj_index].initialize();
    projectors[proj_index].project(new_vertices);
}

__global__ void MyCopyVertices(const float* src_vertices, float* dst_vertices, int num_vertices)
{
    int coord_index = blockIdx.x * blockDim.x + threadIdx.x;
    if (coord_index >= 3*num_vertices) return;

    dst_vertices[coord_index] = src_vertices[coord_index];
}

int main(void)
{
    std::vector<std::unique_ptr<ObjectConfig> > object_configs;
    object_configs.push_back(std::make_unique<XPBDMeshObjectConfig>(
        "obj1", Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), false,
        "../resource/general/single.msh", 1, Vec3r(1,1,1), false, true, true, Vec4r(0,0,0,1),
        1000, 3e6, 0.48, 0.4, 0.1,
        1, XPBDSolverType::GAUSS_SEIDEL, XPBDConstraintType::STABLE_NEOHOOKEAN_COMBINED, false, false, 0, XPBDResidualPolicy::NEVER
    ));

    SimulationConfig sim_config("dummy_sim", "", 1e-3, 10, 9.81, SimulationMode::AFAP, Visualization::NONE, 0, 0);
    Sim::Simulation sim(std::move(sim_config));
    // sim.setup();
    XPBDMeshObjectConfig config("obj1", Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), false,
                                "../resource/cube/cube16.msh", 1, Vec3r(1,1,1), false, true, true, Vec4r(0,0,0,1),
                                1000, 3e6, 0.48, 0.4, 0.1,
                                1, XPBDSolverType::GAUSS_SEIDEL, XPBDConstraintType::STABLE_NEOHOOKEAN_COMBINED, false, false, 0, XPBDResidualPolicy::NEVER);
    Sim::XPBDMeshObject xpbd_obj(&sim, &config);
    
    xpbd_obj.setup();
    xpbd_obj.createGPUResource();

    // create GPU constraints for each element
    typedef GPUCombinedConstraintProjector<GPUHydrostaticConstraint, GPUDeviatoricConstraint> Projector;
    std::vector<Projector> projectors;
    for (int i = 0; i < xpbd_obj.tetMesh()->numElements(); i++)
    {
        const Eigen::Vector4i& elem = xpbd_obj.tetMesh()->element(i);
        GPUHydrostaticConstraint hyd(&xpbd_obj, elem[0], elem[1], elem[2], elem[3]);
        GPUDeviatoricConstraint dev(&xpbd_obj, elem[0], elem[1], elem[2], elem[3]);
        Projector combined_projector(std::move(hyd), std::move(dev), 1e-3);
        projectors.push_back(std::move(combined_projector));
    }

    // allocate space on device
    Sim::ArrayGPUResource<Projector> projectors_resource(projectors.data(), projectors.size());
    projectors_resource.allocate();
    projectors_resource.fullCopyToDevice();
    // GPUConstraintProjector<GPUDeviatoricConstraint> dev_projector(std::move(dev), 1e-3);

    // allocate space for new vertices
    std::vector<float> new_vertices(xpbd_obj.mesh()->numVertices()*3);
    Sim::ArrayGPUResource<float> new_vertices_resource(new_vertices.data(), new_vertices.size());
    new_vertices_resource.allocate();
    new_vertices_resource.fullCopyToDevice();

    // launch kernels
    int block_size = 256;
    int num_blocks = (projectors.size() + block_size - 1) / block_size;
    int num_vertex_blocks = (xpbd_obj.mesh()->numVertices()*3 + block_size - 1) / block_size;
    ProjectConstraints<<<num_blocks, block_size>>>(projectors_resource.gpuArr(), projectors.size(), new_vertices_resource.gpuArr());

    CHECK_CUDA_ERROR(cudaPeekAtLastError());

    CHECK_CUDA_ERROR(cudaDeviceSynchronize());




    const Sim::TetMeshGPUResource& mesh_gpu_resource = xpbd_obj.gpuResource()->meshGpuResource();
    std::array<int, 100> nanosecs;
    for (int i = 0; i < 100; i++)
    {
        auto start2 = std::chrono::high_resolution_clock::now();

        mesh_gpu_resource.partialCopyToDevice();
        // CopyVertices<<<num_vertex_blocks, block_size>>>(new_vertices_resource.gpuArr(), mesh_gpu_resource->gpuVertices(), xpbd_obj->mesh()->numVertices());

        for(int gi = 0; gi < 4; gi++)
        {
            ProjectConstraints<<<num_blocks, block_size>>>(projectors_resource.gpuArr(), projectors.size(), new_vertices_resource.gpuArr());
            // MyCopyVertices<<<num_vertex_blocks, block_size>>>(new_vertices_resource.gpuArr(), mesh_gpu_resource.gpuVertices(), xpbd_obj.mesh()->numVertices());
        }
        CHECK_CUDA_ERROR(cudaPeekAtLastError());

        CHECK_CUDA_ERROR(cudaDeviceSynchronize());
        

        auto end2 = std::chrono::high_resolution_clock::now();
        auto nanosec2 = std::chrono::duration_cast<std::chrono::nanoseconds>(end2 - start2);
        nanosecs[i] = nanosec2.count();
    }
    for (const auto& t : nanosecs)
    {
        std::cout << "Elapsed time on GPU: " << t << " ns\n";
    }

}