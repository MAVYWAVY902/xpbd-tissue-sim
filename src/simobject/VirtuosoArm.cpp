#include "simobject/VirtuosoArm.hpp"

#include "utils/GeometryUtils.hpp"

#include <algorithm>

namespace Sim 
{

VirtuosoArm::VirtuosoArm(const Simulation* sim, const VirtuosoArmConfig* config)
    : Object(sim, config)
{
    _it_dia = config->innerTubeDiameter();
    _it_translation = config->innerTubeInitialTranslation();
    _it_rotation = config->innerTubeInitialRotation() * 3.1415/180.0;   // convert to radians

    _ot_dia = config->outerTubeDiameter();
    _ot_r_curvature = config->outerTubeRadiusOfCurvature();
    _ot_translation = config->outerTubeInitialTranslation();
    _ot_rotation = config->outerTubeInitialRotation() * 3.1415/180.0;   // convert to radians
    _ot_distal_straight_length = config->outerTubeDistalStraightLength();

    _endoscope_position = config->endoscopeInitialPosition();
    Eigen::Vector3d initial_rot_xyz = config->endoscopeInitialRotation() * 3.1415 / 180.0;
    _endoscope_rotation = GeometryUtils::quatToMat(GeometryUtils::eulXYZ2Quat(initial_rot_xyz[0], initial_rot_xyz[1], initial_rot_xyz[2]));

    _recomputeCoordinateFrames();

}

std::string VirtuosoArm::toString(const int indent) const
{
    // TODO: better toString
    return Object::toString(indent);
}

Eigen::Vector3d VirtuosoArm::tipPosition() const
{
    // if (_stale_frames)
    //     _recomputeCoordinateFrames();
    
    return _it_end_frame.origin();
}

void VirtuosoArm::setTipPosition(const Eigen::Vector3d& new_position)
{
    _differentialInverseKinematics(new_position - tipPosition());
    _stale_frames = true;
}

void VirtuosoArm::setup()
{
    // nothing for now
}

void VirtuosoArm::update()
{
    if (_stale_frames)
    {
        _recomputeCoordinateFrames();
    }
}

void VirtuosoArm::velocityUpdate()
{
    // nothing for now
}

/** Returns the axis-aligned bounding-box (AABB) for this Object in global simulation coordinates. */
Geometry::AABB VirtuosoArm::boundingBox() const
{
    return Geometry::AABB(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
}

void VirtuosoArm::_recomputeCoordinateFrames()
{
    // compute endoscope (base) frame based on current position and orientation
    _endoscope_frame = Geometry::CoordinateFrame(Geometry::TransformationMatrix(_endoscope_rotation, _endoscope_position));

    // compute outer tube base frmae
    // consists of rotating about the current z-axis accordint to outer tube rotation
    Geometry::TransformationMatrix T_rot_z(GeometryUtils::Rz(_ot_rotation), Eigen::Vector3d::Zero());
    _ot_base_frame = _endoscope_frame * T_rot_z;

    // compute frame for end of curved section of outer tube
    // consists of rotating around the body y-axis by the angle swept by the outer tube curve
    double swept_angle = std::max(_ot_translation - _ot_distal_straight_length, 0.0) / _ot_r_curvature; 
    const Eigen::Vector3d end_of_xz_curve(-_ot_r_curvature*std::cos(swept_angle) + _ot_r_curvature, 0, _ot_r_curvature*std::sin(swept_angle));
    Geometry::TransformationMatrix T_curve_end(GeometryUtils::Ry(swept_angle), end_of_xz_curve);

    _ot_curve_end_frame = _ot_base_frame * T_curve_end;

    // compute frame for end of outer tube
    // consists of simply moving along the current z-axis by the length of the distal straight section
    Geometry::TransformationMatrix T_curve_end_to_ot_end(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, std::min(_ot_distal_straight_length, _ot_translation)));
    _ot_end_frame = _ot_curve_end_frame * T_curve_end_to_ot_end;

    // compute frame for end of inner tube
    // consists of rotating around the current z-axis by the inner tube rotation and moving along the current z-axis by the exposed length of the inner tube
    Geometry::TransformationMatrix T_ot_end_to_it_end(GeometryUtils::Rz(_it_rotation), Eigen::Vector3d(0, 0, std::max(0.0, _it_translation - _ot_translation)));
    _it_end_frame = _ot_end_frame * T_ot_end_to_it_end;

    _stale_frames = false;

    // std::cout << "Curve end: \n" << _ot_curve_end_frame.transform().asMatrix() << std::endl;
    // std::cout << "OT end: \n" << _ot_end_frame.transform().asMatrix() << std::endl;
    // std::cout << "Tip Transform: \n" << _it_end_frame.transform().asMatrix() << std::endl;

}

void VirtuosoArm::_differentialInverseKinematics(const Eigen::Vector3d& dx)
{
    _recomputeCoordinateFrames();
    const Eigen::Vector3d target_position = tipPosition() + dx;

    for (int i = 0; i < 10; i++)
    {
        const Eigen::Vector3d pos_err = target_position - tipPosition();
        if (pos_err.norm() < 1e-10)
            break;

        const Eigen::Matrix3d J_a = _3DOFAnalyticalHybridJacobian();
        const Eigen::Vector3d dq = J_a.colPivHouseholderQr().solve(pos_err);
        _ot_rotation += dq[0];
        _ot_translation += dq[1];
        _it_translation += dq[2];

        // enforce joint limits
        _ot_translation = std::clamp(_ot_translation, 0.0, 40.0e-3);
        _it_translation = std::clamp(_it_translation, 0.0, 50.0e-3);

        _recomputeCoordinateFrames();
    }

    std::cout << "Tip pos: " << tipPosition()[0] << ", " << tipPosition()[1] << ", " << tipPosition()[2] << std::endl;
    std::cout << "q: " << _ot_rotation << ", " << _ot_translation << ", " << _it_translation << std::endl;

    _stale_frames = true;
}

Eigen::Matrix<double,6,3> VirtuosoArm::_3DOFSpatialJacobian()
{
    // make sure coordinate frames are up to date
    if (_stale_frames)      
        _recomputeCoordinateFrames();

    // because there are max() and min() expressions used, we have a couple different cases for the derivatives
    // when outer tube translation >= length of the outer tube straight section, we have some curve in the outer tube
    double dalpha_d1, dmin_d1;
    if (_ot_translation >= _ot_distal_straight_length)
    {
        dalpha_d1 = 1/_ot_r_curvature;    // partial of alpha (outer tube curve swept angle) w.r.t outer tube translation
        dmin_d1 = 0;                // partial of min(length of OT straight section, outer tube translation) w.r.t. outer tube translation
    }
    else
    {                                
        dalpha_d1 = 0;
        dmin_d1 = 1;
    }

    // when outer tube translation >= inner tube translation, no inner tube is showing
    double dmax_d1, dmax_d2;
    if (_ot_translation >= _it_translation)
    {
        dmax_d1 = 0;    // partial of max(0, inner tube translation - outer tube translation) w.r.t. outer tube translation
        dmax_d2 = 0;    // partial of max(0, inner tube translation - outer tube translation) w.r.t. inner tube translation
    }             
        
    else    
    {
        dmax_d1 = -1;
        dmax_d2 = 1;
    }

    // now just implement dT/dq
    const double alpha = std::max(_ot_translation - _ot_distal_straight_length, 0.0) / _ot_r_curvature;    // angle swept by the outer tube curve
    const double straight_length = std::min(_ot_distal_straight_length, _ot_translation) + std::max(0.0, _it_translation - _ot_translation); // length of the straight section of the combined outer + inner tube
    // precompute some sin and cos
    const double ct1 = std::cos(_ot_rotation);
    const double st1 = std::sin(_ot_rotation);
    const double ca = std::cos(alpha);
    const double sa = std::sin(alpha);

    Eigen::Matrix4d dT_d_ot_rot, dT_d_ot_trans, dT_d_it_trans;
    dT_d_ot_rot << -st1*ca, -ct1, -st1*sa, -st1*sa*straight_length - st1*(-_ot_r_curvature*ca + _ot_r_curvature),
                   ct1*ca, -st1, ct1*sa,  ct1*sa*straight_length + ct1*(-_ot_r_curvature*ca + _ot_r_curvature),
                    0, 0, 0, 0,
                    0, 0, 0, 0;

    dT_d_ot_trans << ct1*-sa*dalpha_d1, 0, ct1*ca*dalpha_d1, ct1*ca*dalpha_d1*straight_length + ct1*sa*(dmin_d1 + dmax_d1) + ct1*(-_ot_r_curvature*-sa*dalpha_d1),
                    st1*-sa*dalpha_d1, 0, st1*ca*dalpha_d1, st1*ca*dalpha_d1*straight_length + st1*sa*(dmin_d1 + dmax_d1) + st1*(-_ot_r_curvature*-sa*dalpha_d1),
                    -ca*dalpha_d1, 0, -sa*dalpha_d1, -sa*dalpha_d1*straight_length + ca*(dmin_d1 + dmax_d1) + _ot_r_curvature*ca*dalpha_d1,
                    0, 0, 0, 0;

    dT_d_it_trans << 0, 0, 0, ct1*sa*dmax_d2,
                     0, 0, 0, st1*sa*dmax_d2,
                     0, 0, 0, ca*dmax_d2,
                     0, 0, 0, 0;

    // and assemble them into the spatial Jacobian
    const Geometry::TransformationMatrix T_inv = _it_end_frame.transform().inverse();
    
    Eigen::Matrix<double,6,3> J_s;
    J_s.col(0) = GeometryUtils::Vee_SE3(_endoscope_frame.transform().asMatrix() * dT_d_ot_rot * T_inv.asMatrix());
    J_s.col(1) = GeometryUtils::Vee_SE3(_endoscope_frame.transform().asMatrix() * dT_d_ot_trans * T_inv.asMatrix());
    J_s.col(2) = GeometryUtils::Vee_SE3(_endoscope_frame.transform().asMatrix() * dT_d_it_trans * T_inv.asMatrix());

    // std::cout << "J_s:\n" << J_s << std::endl;
    return J_s;
}

Eigen::Matrix3d VirtuosoArm::_3DOFAnalyticalHybridJacobian()
{
    // make sure coordinate frames are up to date
    if (_stale_frames)      
        _recomputeCoordinateFrames();

    Eigen::Matrix<double,3,6> C;
    C << Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Identity();

    // std::cout << "C:\n" << C << std::endl;

    Eigen::Matrix<double,6,6> R;
    R << _it_end_frame.transform().rotMat(), Eigen::Matrix3d::Zero(), Eigen::Matrix3d::Zero(), _it_end_frame.transform().rotMat();

    // std::cout << "R:\n" << R << std::endl;

    Eigen::Matrix<double,6,3> J_b = _it_end_frame.transform().inverse().adjoint() * _3DOFSpatialJacobian();
    // std::cout << "J_b:\n" << J_b << std::endl;
    Eigen::Matrix3d J_a = C * R * J_b;

    // std::cout << "J_a:\n" << J_a << std::endl;
    return J_a;
}

} // namespace Sim