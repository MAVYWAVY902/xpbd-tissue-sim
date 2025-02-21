#ifndef __TYPES_HPP
#define __TYPES_HPP

#include <Eigen/Dense>

#ifdef HAVE_CUDA
typedef float Real;
#define CUDA_BLOCK_SIZE 256
#else
typedef double Real;
#endif

typedef Eigen::Vector<Real, 3> Vec3r;
typedef Eigen::Vector<int, 3> Vec3i;
typedef Eigen::Vector<Real, 4> Vec4r;
typedef Eigen::Vector<Real, -1> VecXr;

typedef Eigen::Matrix<Real, 3, 3> Mat3r;
typedef Eigen::Matrix<Real, -1, -1> MatXr;

#endif // __TYPES_HPP