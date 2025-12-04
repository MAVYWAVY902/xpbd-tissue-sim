#include "solver/constraint/InterObjectDeformableCollisionConstraint.hpp"

#include "utils/MathUtils.hpp"
#include <iostream>

namespace Solver
{

InterObjectDeformableCollisionConstraint::InterObjectDeformableCollisionConstraint(int v, Real* p, Real m,
                                                                                    int fv1, Real* fp1, Real fm1,
                                                                                    int fv2, Real* fp2, Real fm2,
                                                                                    int fv3, Real* fp3, Real fm3)
    : Constraint(std::vector<PositionReference>({
    PositionReference(v, p, m),
    PositionReference(fv1, fp1, fm1),
    PositionReference(fv2, fp2, fm2),
    PositionReference(fv3, fp3, fm3)}), 0)  // Much smaller compliance = stiffer/harder constraint (reduced from 1e-8)
{

}

void InterObjectDeformableCollisionConstraint::evaluate(Real* C) const
{
    Eigen::Map<const Vec3r> q(_positions[0].position_ptr);
    Eigen::Map<const Vec3r> p1(_positions[1].position_ptr);
    Eigen::Map<const Vec3r> p2(_positions[2].position_ptr);
    Eigen::Map<const Vec3r> p3(_positions[3].position_ptr);

    const Vec3r a = (p2 - p1).cross(p3 - p1);
    Real a_norm = a.norm();

    // C = signed distance from vertex q to triangle plane
    // C > 0: separated (no collision)
    // C < 0: penetrating (collision!)
    // Adding small bias (1e-5) for numerical stability and to create a small "contact zone"
    *C = (q - p1).dot(a) / a_norm + 1e-5;
    
    // DEBUG: Print inter-object collision constraint evaluation (limit output frequency)
    static int collision_debug_count = 0;
    // if (collision_debug_count++ % 900 == 0) {  // Print every 900 evaluations
    //     std::cout << "[INTER-OBJECT collision DEBUG] Deformable-deformable eval #" << collision_debug_count
    //               << ": point=(" << q.transpose() << ")"
    //               << " triangle_center=(" << ((p1 + p2 + p3) / 3.0).transpose() << ")"
    //               << " C=" << *C << " (C>0 means separated, C<=0 means penetration)\n";
    // }
}

void InterObjectDeformableCollisionConstraint::gradient(Real* delC) const
{
    Eigen::Map<const Vec3r> q(_positions[0].position_ptr);
    Eigen::Map<const Vec3r> p1(_positions[1].position_ptr);
    Eigen::Map<const Vec3r> p2(_positions[2].position_ptr);
    Eigen::Map<const Vec3r> p3(_positions[3].position_ptr);

    const Vec3r a = (p2 - p1).cross(p3 - p1);
    Real a_norm = a.norm();

    const Vec3r gq = a/a_norm;

    const Mat3r I_aaT = Mat3r::Identity()/a_norm - a*a.transpose() / (a_norm * a_norm * a_norm);
    const Vec3r gp1 = -a.transpose()/a_norm + (q-p1).transpose() * I_aaT * MathUtils::Skew3(p3 - p2);
    const Vec3r gp2 = (q-p1).transpose() * I_aaT * MathUtils::Skew3(p1 - p3);
    const Vec3r gp3 = (q-p2).transpose() * I_aaT * MathUtils::Skew3(p2 - p1);

    delC[0] = gq[0];
    delC[1] = gq[1];
    delC[2] = gq[2];

    delC[3] = gp1[0];
    delC[4] = gp1[1];
    delC[5] = gp1[2];
    
    delC[6] = gp2[0];
    delC[7] = gp2[1];
    delC[8] = gp2[2];

    delC[9] = gp3[0];
    delC[10] = gp3[1];
    delC[11] = gp3[2];
}

void InterObjectDeformableCollisionConstraint::evaluateWithGradient(Real* C, Real* grad) const
{
    evaluate(C);
    gradient(grad);
}

#ifdef HAVE_CUDA
InterObjectDeformableCollisionConstraint::GPUConstraintType InterObjectDeformableCollisionConstraint::createGPUConstraint() const
{
    GPUConstraintType gpu_constraint = GPUConstraintType(_positions[0].index, _positions[0].inv_mass,
                                                            _positions[1].index, _positions[1].inv_mass,
                                                            _positions[2].index, _positions[2].inv_mass,
                                                            _u, _v, _w,
                                                            _p, _collision_normal);
    return gpu_constraint;
}
#endif

// Friction support can be added later if needed for inter-object collisions
// Currently commented out to keep the implementation simple

} // namespace Solver
