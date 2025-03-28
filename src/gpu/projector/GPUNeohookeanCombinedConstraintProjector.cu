#include "gpu/projector/GPUNeohookeanCombinedConstraintProjector.cuh"

template <>
__host__ GPUCombinedConstraintProjector<GPUDeviatoricConstraint, GPUHydrostaticConstraint>::GPUCombinedConstraintProjector(const GPUHydrostaticConstraint& constraint1_, const GPUDeviatoricConstraint& constraint2_, float dt_)
    : dt(dt_), alpha_h(constraint1_.alpha), alpha_d(constraint2_.alpha), gamma(constraint1_.gamma)
{
    for (int i = 0; i < 4; i++)
    {
        positions[i].index = constraint1_.positions[i].index;
        positions[i].inv_mass = constraint1_.positions[i].inv_mass;
    }
    std::cout << "GPUNeohookeanCombinedConstraintProjector const& constructor!" << std::endl;
    // printf("indices: %i, %i, %i, %i\n", positions[0].index, positions[1].index, positions[2].index, positions[3].index);

    for (int i = 0; i < 9; i++)
    {
        Q[i] = constraint1_.Q[i];
    }
}
    
template <>
__host__ GPUCombinedConstraintProjector<GPUDeviatoricConstraint, GPUHydrostaticConstraint>::GPUCombinedConstraintProjector(GPUHydrostaticConstraint&& constraint1_, GPUDeviatoricConstraint&& constraint2_, float dt_)
    : dt(dt_), alpha_h(constraint1_.alpha), alpha_d(constraint2_.alpha), gamma(constraint1_.gamma)
{
    for (int i = 0; i < 4; i++)
    {
        positions[i].index = constraint1_.positions[i].index;
        positions[i].inv_mass = constraint1_.positions[i].inv_mass;
    }
    std::cout << "GPUNeohookeanCombinedConstraintProjector && constructor!" << std::endl;
    // printf("indices: %i, %i, %i, %i\n", positions[0].index, positions[1].index, positions[2].index, positions[3].index);

    for (int i = 0; i < 9; i++)
    {
        Q[i] = constraint1_.Q[i];
    }
}

template<>
__device__ GPUCombinedConstraintProjector<GPUDeviatoricConstraint, GPUHydrostaticConstraint>::GPUCombinedConstraintProjector()
{

}

template<>
__device__ void GPUCombinedConstraintProjector<GPUDeviatoricConstraint, GPUHydrostaticConstraint>::initialize()
{
    lambda[0] = 0;
    lambda[1] = 0;
}

template<>
__device__ void GPUCombinedConstraintProjector<GPUDeviatoricConstraint, GPUHydrostaticConstraint>::project(const float* vertices, float* new_vertices)
{
    // printf("indices: %i, %i, %i, %i\n", positions[0].index, positions[1].index, positions[2].index, positions[3].index);
    float x[12];
    *reinterpret_cast<float3*>(x) = *reinterpret_cast<const float3*>(vertices + 3*positions[0].index);
    *reinterpret_cast<float3*>(x+3) = *reinterpret_cast<const float3*>(vertices + 3*positions[1].index);
    *reinterpret_cast<float3*>(x+6) = *reinterpret_cast<const float3*>(vertices + 3*positions[2].index);
    *reinterpret_cast<float3*>(x+9) = *reinterpret_cast<const float3*>(vertices + 3*positions[3].index);
    // x[0] = 0; x[1] = 0; x[2] = 0; x[3] = 0; x[4] = 0; x[5] = 0;
    // x[6] = 0; x[7] = 0; x[8] = 0; x[9] = 0; x[10] = 0; x[11] = 0;

    // compute F
    float X[9];
    X[0] = x[0] - x[9]; X[1] = x[1] - x[10]; X[2] = x[2] - x[11];
    X[3] = x[3] - x[9]; X[4] = x[4] - x[10]; X[5] = x[5] - x[11];
    X[6] = x[6] - x[9]; X[7] = x[7] - x[10]; X[8] = x[8] - x[11];
    
    float F[9];
    Mat3Mul(X, Q, F);

    // compute hydrostatic constraint and its gradient
    float C_h_grad[12];

    // C_h = det(F) - (1 + gamma)
    const float C_h = F[0]*F[4]*F[8] - F[0]*F[7]*F[5] - F[3]*F[1]*F[8] + F[3]*F[7]*F[2] + F[6]*F[1]*F[5] - F[6]*F[4]*F[2] - (1+gamma);

    float F_cross[9];
    Vec3Cross(F+3, F+6, F_cross);   // 2nd column of F crossed with 3rd column
    Vec3Cross(F+6, F, F_cross+3);   // 3rd column of F crossed with 1st column
    Vec3Cross(F, F+3, F_cross+6);   // 1st column of F crossed with 2nd column

    Mat3MulTranspose(F_cross, Q, C_h_grad);
    C_h_grad[9] = -C_h_grad[0] - C_h_grad[3] - C_h_grad[6];
    C_h_grad[10] = -C_h_grad[1] - C_h_grad[4] - C_h_grad[7];
    C_h_grad[11] = -C_h_grad[2] - C_h_grad[5] - C_h_grad[8];

    // printf("Ch_grad: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", C_h_grad[0], C_h_grad[1], C_h_grad[2], C_h_grad[3], C_h_grad[4], C_h_grad[5], C_h_grad[6], C_h_grad[7], C_h_grad[8], C_h_grad[9], C_h_grad[10], C_h_grad[11]);

    // compute deviatoric constraint and its gradient
    float C_d_grad[12];

    // C_d = frob(F)
    const float C_d = sqrtf(F[0]*F[0] + F[1]*F[1] + F[2]*F[2] + F[3]*F[3] + F[4]*F[4] + F[5]*F[5] + F[6]*F[6] + F[7]*F[7] + F[8]*F[8]);
    const float inv_C_d = 1.0/C_d;

    Mat3MulTranspose(F, Q, C_d_grad);
    for (int i = 0; i < 9; i++) { C_d_grad[i] *= inv_C_d; }
    C_d_grad[9] =  -C_d_grad[0] - C_d_grad[3] - C_d_grad[6];
    C_d_grad[10] = -C_d_grad[1] - C_d_grad[4] - C_d_grad[7];
    C_d_grad[11] = -C_d_grad[2] - C_d_grad[5] - C_d_grad[8];

    // printf("Cd_grad: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", C_d_grad[0], C_d_grad[1], C_d_grad[2], C_d_grad[3], C_d_grad[4], C_d_grad[5], C_d_grad[6], C_d_grad[7], C_d_grad[8], C_d_grad[9], C_d_grad[10], C_d_grad[11]);

    // solve the 2x2 system
    float A[4];
    A[0] = alpha_h / (dt * dt);
    A[1] = 0;
    A[2] = 0;
    A[3] = alpha_d / (dt * dt);
    for (int i = 0; i < 4; i++)
    {
        A[0] += positions[i].inv_mass * (C_h_grad[3*i]*C_h_grad[3*i] + C_h_grad[3*i+1]*C_h_grad[3*i+1] + C_h_grad[3*i+2]*C_h_grad[3*i+2]);
        A[3] += positions[i].inv_mass * (C_d_grad[3*i]*C_d_grad[3*i] + C_d_grad[3*i+1]*C_d_grad[3*i+1] + C_d_grad[3*i+2]*C_d_grad[3*i+2]);
        A[1] += positions[i].inv_mass * (C_h_grad[3*i]*C_d_grad[3*i] + C_h_grad[3*i+1]*C_d_grad[3*i+1] + C_h_grad[3*i+2]*C_d_grad[3*i+2]);
    }
    A[2] = A[1];

    float k[2];
    k[0] = -C_h - alpha_h * lambda[0]; // TODO: include current lambda (assuming its 0 right now)
    k[1] = -C_d - alpha_d * lambda[1]; // TODO: include current lambda

    const float detA = A[0]*A[3] - A[1]*A[2];
    const float dlam_h = (k[0]*A[3] - k[1]*A[2]) / detA;
    const float dlam_d = (k[1]*A[0] - k[0]*A[1]) / detA;

    lambda[0] += dlam_h;
    lambda[1] += dlam_d;

    // printf("elem: %i  dlam_h: %.10f  dlam_d: %.10f\n", elem_index, dlam_h, dlam_d);
    // const float dlam_h = 0;
    // const float dlam_d = 0;

    // // compute the coordinate updates
    for (int i = 0; i < 4; i++)
    {
        float pos_update_x = positions[i].inv_mass * (C_h_grad[3*i] * dlam_h + C_d_grad[3*i] * dlam_d);
        float pos_update_y = positions[i].inv_mass * (C_h_grad[3*i+1] * dlam_h + C_d_grad[3*i+1] * dlam_d);
        float pos_update_z = positions[i].inv_mass * (C_h_grad[3*i+2] * dlam_h + C_d_grad[3*i+2] * dlam_d);
        // printf("pos update %i: %f, %f, %f\n", i, pos_update_x, pos_update_y, pos_update_z);
        float* v_ptr = new_vertices + 3*positions[i].index;
        atomicAdd(v_ptr, pos_update_x);
        atomicAdd(v_ptr + 1, pos_update_y);
        atomicAdd(v_ptr + 2, pos_update_z);
        // coord_updates[12*elem_index + 3*i] =   inv_m[i] * (C_h_grad[3*i] * dlam_h + C_d_grad[3*i] * dlam_d);
        // coord_updates[12*elem_index + 3*i+1] = inv_m[i] * (C_h_grad[3*i+1] * dlam_h + C_d_grad[3*i+1] * dlam_d);
        // coord_updates[12*elem_index + 3*i+2] = inv_m[i] * (C_h_grad[3*i+2] * dlam_h + C_d_grad[3*i+2] * dlam_d);
    }
}