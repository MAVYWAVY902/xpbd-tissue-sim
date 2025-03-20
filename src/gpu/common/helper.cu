#include "gpu/common/helper.cuh"

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

// computes cross product between 2 vectors
__host__ __device__ void Vec3Cross(const float* v1, const float* v2, float* v3)
{
    v3[0] = v1[1]*v2[2] - v1[2]*v2[1];
    v3[1] = v1[2]*v2[0] - v1[0]*v2[2];
    v3[2] = v1[0]*v2[1] - v1[1]*v2[0];
}

__host__ __device__ float Vec3Dot(const float* v1, const float* v2)
{
    return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}

// computes the rest state matrix Q and the volume for a tetrahedral element defined by (v0, v1, v2, v3)
__host__ void computeQandVolume(const Eigen::Vector3f& v0, const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, const Eigen::Vector3f& v3, float* Q, float* rest_volume)
{
    Eigen::Matrix3f X;
    X.col(0) = v0 - v3;
    X.col(1) = v1 - v3;
    X.col(2) = v2 - v3;

    Eigen::Matrix3f Q_mat = X.inverse();
    for (int k = 0; k < 9; k++) { Q[k] = Q_mat.data()[k]; }

    *rest_volume = std::abs(X.determinant()/6.0);
}

// computes the deformation gradient F given the current nodal positions x and the rest state matrix Q
__device__ void computeF(const float* x, const float* Q, float* F)
{
    float X[9];
    X[0] = x[0] - x[9]; X[1] = x[1] - x[10]; X[2] = x[2] - x[11];
    X[3] = x[3] - x[9]; X[4] = x[4] - x[10]; X[5] = x[5] - x[11];
    X[6] = x[6] - x[9]; X[7] = x[7] - x[10]; X[8] = x[8] - x[11];

    Mat3Mul(X, Q, F);
}