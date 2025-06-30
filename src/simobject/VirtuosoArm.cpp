#include "simobject/VirtuosoArm.hpp"
#include "simobject/XPBDMeshObject.hpp"

#include "config/simobject/VirtuosoArmConfig.hpp"

#include "utils/GeometryUtils.hpp"
#include "utils/MathUtils.hpp"

#include <math.h>
#include <algorithm>

namespace Sim 
{

VirtuosoArm::VirtuosoArm(const Simulation* sim, const ConfigType* config)
    : Object(sim, config), _ot_frames(), _it_frames()
{
    _it_outer_dia = config->innerTubeOuterDiameter();
    _it_inner_dia = config->innerTubeInnerDiameter();
    _it_translation = config->innerTubeInitialTranslation();
    _it_rotation = config->innerTubeInitialRotation() * M_PI/180.0;   // convert to radians

    _ot_outer_dia = config->outerTubeOuterDiameter();
    _ot_inner_dia = config->outerTubeInnerDiameter();
    _ot_r_curvature = config->outerTubeRadiusOfCurvature();
    _ot_translation = config->outerTubeInitialTranslation();
    _ot_rotation = config->outerTubeInitialRotation() * M_PI/180.0;   // convert to radians
    _ot_distal_straight_length = config->outerTubeDistalStraightLength();

    _tool_state = 0;  // default tool state is off
    _tool_type = config->toolType();
    _tool_manipulated_object = nullptr;

    _arm_base_position = config->baseInitialPosition();
    Vec3r initial_rot_xyz = config->baseInitialRotation() * M_PI / 180.0;
    _arm_base_rotation = GeometryUtils::quatToMat(GeometryUtils::eulXYZ2Quat(initial_rot_xyz[0], initial_rot_xyz[1], initial_rot_xyz[2]));

    // _recomputeCoordinateFrames();
    _recomputeCoordinateFramesStaticsModel();

}

std::string VirtuosoArm::toString(const int indent) const
{
    // TODO: better toString
    return Object::toString(indent);
}

Vec3r VirtuosoArm::tipPosition() const
{
    
    return innerTubeEndFrame().origin();
}

void VirtuosoArm::setTipPosition(const Vec3r& new_position)
{
    // _jacobianDifferentialInverseKinematics(new_position - tipPosition());
    _hybridDifferentialInverseKinematics(new_position - tipPosition());
    _stale_frames = true;
}

void VirtuosoArm::setTipForce(const Vec3r& new_tip_force)
{
    _tip_force = new_tip_force;
    _recomputeCoordinateFramesStaticsModel();
}

void VirtuosoArm::setTipMoment(const Vec3r& new_tip_moment)
{
    _tip_moment = new_tip_moment;
    _recomputeCoordinateFramesStaticsModel();
}

void VirtuosoArm::setTipForceAndMoment(const Vec3r& new_tip_force, const Vec3r& new_tip_moment)
{
    _tip_force = new_tip_force;
    _tip_moment = new_tip_moment;
    _recomputeCoordinateFramesStaticsModel();
}

void VirtuosoArm::setJointState(double ot_rotation, double ot_translation, double it_rotation, double it_translation, int tool)
{
    _ot_rotation = ot_rotation;
    _ot_translation = ot_translation;
    _it_rotation = it_rotation;
    _it_translation = it_translation;
    _tool_state = tool;

    _recomputeCoordinateFrames();
}

void VirtuosoArm::setup()
{
    // nothing for now
}

void VirtuosoArm::update()
{
    if (_stale_frames)
    {
        // _recomputeCoordinateFrames();
        _recomputeCoordinateFramesStaticsModel();
    }

    _toolAction();
}

void VirtuosoArm::velocityUpdate()
{
    // nothing for now
}

/** Returns the axis-aligned bounding-box (AABB) for this Object in global simulation coordinates. */
Geometry::AABB VirtuosoArm::boundingBox() const
{
    return Geometry::AABB(Vec3r::Zero(), Vec3r::Zero());
}

void VirtuosoArm::_toolAction()
{
    // if the manipulated object has not been given, do nothing
    if (!_tool_manipulated_object)
        return;

    if (_tool_type == ToolType::SPATULA)
        _spatulaToolAction();
    else if (_tool_type == ToolType::GRASPER)
        _grasperToolAction();
    else if (_tool_type == ToolType::CAUTERY)
        _cauteryToolAction();

    _last_tool_state = _tool_state;
}

void VirtuosoArm::_spatulaToolAction()
{
    // don't need to do anything for the spatula
}

void VirtuosoArm::_grasperToolAction()
{
    // if tool state has changed from 0 to 1, start grasping vertices inside grasping radius
    if (_tool_state == 1 && _last_tool_state == 0)
    {
        std::map<int, Vec3r> vertices_to_grasp;

        // quick and dirty way to find all vertices in a sphere
        for (int theta = 0; theta < 360; theta+=30)
        {
            for (int phi = 0; phi < 360; phi+=30)
            {
                for (double p = 0; p < GRASPING_RADIUS; p+=GRASPING_RADIUS/5.0)
                {
                    const double x = _tool_position[0] + p*std::sin(phi*M_PI/180)*std::cos(theta*M_PI/180);
                    const double y = _tool_position[1] + p*std::sin(phi*M_PI/180)*std::sin(theta*M_PI/180);
                    const double z = _tool_position[2] + p*std::cos(phi*M_PI/180);
                    int v = _tool_manipulated_object->mesh()->getClosestVertex(Vec3r(x, y, z));

                    // make sure v is inside grasping sphere
                    if ((_tool_position - _tool_manipulated_object->mesh()->vertex(v)).norm() <= GRASPING_RADIUS)
                        if (!_tool_manipulated_object->vertexFixed(v))
                        {
                            const Vec3r attachment_offset = (_tool_manipulated_object->mesh()->vertex(v) - _tool_position) * 0.9;
                            vertices_to_grasp[v] = attachment_offset;
                        }
                }
            }
        }

        for (const auto& [v, offset] : vertices_to_grasp)
        {
            _tool_manipulated_object->addAttachmentConstraint(v, &_tool_position, offset);
            
            _grasped_vertices.push_back(v);
        }
    }

    // if tool state has changed from 1 to 0, stop grasping
    else if (_tool_state == 0 && _last_tool_state == 1)
    {
        _tool_manipulated_object->clearAttachmentConstraints();
        _grasped_vertices.clear();
    }

    Vec3r total_force = Vec3r::Zero();
    for (const auto& v : _grasped_vertices)
    {
        total_force += _tool_manipulated_object->elasticForceAtVertex(v);
    }

    // smooth forces
    // Vec3r new_force = total_force*0.5 + _last_force*0.5;

    // const Vec3r force = 1000*(_initial_grasp_pos - _tip_cursor->position());

    std::cout << "TOTAL GRASPED FORCE: " << total_force[0] << ", " << total_force[1] << ", " << total_force[2] << std::endl;

    // transform force from global coordinates into haptic input frame
    Vec3r test_force(10,0,0);
    setTipForce(test_force);
    
}

void VirtuosoArm::_cauteryToolAction()
{
    // do nothing (for now)
}

void VirtuosoArm::_recomputeCoordinateFrames()
{
    // compute arm (base) frame based on current position and orientation
    _arm_base_frame = Geometry::CoordinateFrame(Geometry::TransformationMatrix(_arm_base_rotation, _arm_base_position));

    // compute outer tube base frame
    // consists of rotating about the current z-axis accordint to outer tube rotation
    Geometry::TransformationMatrix T_rot_z(GeometryUtils::Rz(_ot_rotation), Vec3r::Zero());
    Geometry::CoordinateFrame ot_base_frame = _arm_base_frame * T_rot_z;

    Real swept_angle = std::max(_ot_translation - _ot_distal_straight_length, Real(0.0)) / _ot_r_curvature; 
    // frames for the curved section of the outer tube
    for (int i = 0; i < NUM_OT_CURVE_FRAMES; i++)
    {
        Real cur_angle = i * swept_angle / (NUM_OT_CURVE_FRAMES - 1);
        const Vec3r curve_point(0.0, -_ot_r_curvature +_ot_r_curvature*std::cos(cur_angle), _ot_r_curvature*std::sin(cur_angle));
        Geometry::TransformationMatrix T(GeometryUtils::Rx(cur_angle), curve_point);
        _ot_frames[i] = ot_base_frame * T;
    }

    // frames for the straight section of the outer tube
    for (int i = 0; i < NUM_OT_STRAIGHT_FRAMES; i++)
    {
        // relative transformation along z-axis
        Geometry::TransformationMatrix T(Mat3r::Identity(), Vec3r(0,0, std::min(_ot_translation, _ot_distal_straight_length) / NUM_OT_STRAIGHT_FRAMES));
        _ot_frames[NUM_OT_CURVE_FRAMES + i] = _ot_frames[NUM_OT_CURVE_FRAMES + i -1] * T;
    }

    // frames for the inner tube
    // unrotate about z-axis for outer tube rotation, then rotate about z-axis with inner tube rotation to get to the beginning of the exposed part of the inner tube
    Geometry::TransformationMatrix T_rot_z_outer_to_inner(GeometryUtils::Rz(_it_rotation-_ot_rotation), Vec3r::Zero());
    _it_frames[0] = _ot_frames.back() * T_rot_z_outer_to_inner;
    const Real it_length = std::max(Real(0.0), _it_translation - _ot_translation);      // exposed length of the inner tube
    for (int i = 1; i < NUM_IT_FRAMES; i++)
    {
        // relative transformation matrix along z-axis
        Geometry::TransformationMatrix T(Mat3r::Identity(), Vec3r(0, 0, it_length / (NUM_IT_FRAMES - 1)));
        _it_frames[i] = _it_frames[i-1] * T;
    }

    _stale_frames = false;

    // for now, the tool position is just the inner tube tip position with no offset
    _tool_position = _it_frames.back().origin();

}

void VirtuosoArm::_recomputeCoordinateFramesStaticsModel()
{
    // compute arm base frame based on current position and orientation
    _arm_base_frame = Geometry::CoordinateFrame(Geometry::TransformationMatrix(_arm_base_rotation, _arm_base_position));

    // compute outer tube base frame
    // consists of rotating about the current z-axis accordint to outer tube rotation
    Geometry::TransformationMatrix T_rot_z(GeometryUtils::Rz(_ot_rotation), Vec3r::Zero());
    Geometry::CoordinateFrame ot_base_frame = _arm_base_frame * T_rot_z;
    
    // calculate internal forces and moments carried at the base of the tube collection
    const Vec3r& tip_position = innerTubeEndFrame().origin();
    const Vec3r base_moment = _tip_moment + tip_position.cross(_tip_force);
    const Vec3r base_force = _tip_force;


    // EVERYTHING FROM NOW ON WILL USE MM
    // compute cross section properties
    Real E = 60e9/1000/1000;    // Young's Modulus (N/mm^2)
    Real G = E / (2*(1+0.3));   // Shear modulus (N/mm^2)

    Real ot_outer_dia_mm = _ot_outer_dia * 1000;
    Real ot_inner_dia_mm = _ot_inner_dia * 1000;
    Real it_outer_dia_mm = _it_outer_dia * 1000;
    Real it_inner_dia_mm = _it_inner_dia * 1000;
    Real I_ot = M_PI/4 * (ot_outer_dia_mm*ot_outer_dia_mm*ot_outer_dia_mm*ot_outer_dia_mm/16 - ot_inner_dia_mm*ot_inner_dia_mm*ot_inner_dia_mm*ot_inner_dia_mm/16);
    Real I_it = M_PI/4 * (it_outer_dia_mm*it_outer_dia_mm*it_outer_dia_mm*it_outer_dia_mm/16 - it_inner_dia_mm*it_inner_dia_mm*it_inner_dia_mm*it_inner_dia_mm/16);
    Real J_ot = 2*I_ot;
    Real J_it = 2*I_it;

    // integrate the curved section of the outer tube
    Real curved_length = (_ot_translation - _ot_distal_straight_length)*1000; // in mm

    TubeIntegrationState ot_base_state;
    ot_base_state.position = ot_base_frame.origin()*1000;   // put position in mm
    ot_base_state.orientation = ot_base_frame.transform().rotMat();
    ot_base_state.internal_force = base_force;
    ot_base_state.internal_moment = base_moment*1000; // put it in N-mm
    ot_base_state.torsional_displacement = _ot_rotation;

    TubeIntegrationState ot_curve_end_state;
    if (curved_length > 0)
    {
        // std::cout << "Outer tube curved section length > 0 (=" << curved_length << " mm)" << std::endl;
        const Vec3r K_inv(1/(E*I_ot + E*I_it), 1/(E*I_ot + E*I_it), 1/(G*J_ot));
        const Vec3r precurvature(1/_ot_r_curvature/1000, 0, 0);

        std::vector<Real> ot_curve_s(NUM_OT_CURVE_FRAMES);  // distance along tube for outer curve frames (in mm)
        for (int i = 0; i < NUM_OT_CURVE_FRAMES; i++)
            ot_curve_s[i] = i* (curved_length / (NUM_OT_CURVE_FRAMES-1));

        std::vector<TubeIntegrationState::VecType> ot_curve_states = _integrateTubeRK4(ot_base_state, ot_curve_s, K_inv, precurvature);
        
        for (int i = 0; i < NUM_OT_CURVE_FRAMES; i++)
        {
            TubeIntegrationState state = TubeIntegrationState::fromVec(ot_curve_states[i]);
            _ot_frames[i].setTransform(Geometry::TransformationMatrix(state.orientation, state.position/1000).asMatrix()); // convert position back to m
        }

        ot_curve_end_state = TubeIntegrationState::fromVec(ot_curve_states.back());
    }
    else
    {
        for (int i = 0; i < NUM_OT_CURVE_FRAMES; i++)
        {
            _ot_frames[i].setTransform(ot_base_frame.transform().asMatrix());
        }
        ot_curve_end_state = ot_base_state;
    }

    // integrate straight section of outer tube
    Real straight_length = std::min(_ot_translation, _ot_distal_straight_length)*1000; // length of outer tube straight section (in mm)
    TubeIntegrationState ot_end_state;
    if (straight_length > 0)
    {
        // std::cout << "Outer tube straight section length > 0 mm (=" << straight_length << " mm)" << std::endl;
        const Vec3r ot_K_inv(1/(E*I_ot + E*I_it), 1/(E*I_ot + E*I_it), 1/(G*J_ot));
        std::vector<Real> ot_straight_s(NUM_OT_STRAIGHT_FRAMES+1);
        for (int i = 0; i < NUM_OT_STRAIGHT_FRAMES+1; i++)
            ot_straight_s[i] = curved_length + i*straight_length / NUM_OT_STRAIGHT_FRAMES;
        std::vector<TubeIntegrationState::VecType> ot_straight_states = _integrateTubeRK4(ot_curve_end_state, ot_straight_s, ot_K_inv, Vec3r(0,0,0));
        

        for (int i = 0; i < NUM_OT_STRAIGHT_FRAMES; i++)
        {
            TubeIntegrationState state = TubeIntegrationState::fromVec(ot_straight_states[i+1]);
            _ot_frames[NUM_OT_CURVE_FRAMES + i].setTransform(Geometry::TransformationMatrix(state.orientation, state.position/1000).asMatrix()); // convert position back to m
        }

        ot_end_state = TubeIntegrationState::fromVec(ot_straight_states.back());
    }
    else
    {
        for (int i = 0; i < NUM_OT_STRAIGHT_FRAMES; i++)
        {
            _ot_frames[NUM_OT_CURVE_FRAMES + i].setTransform(_ot_frames[NUM_OT_CURVE_FRAMES-1].transform().asMatrix());
        }
        
        ot_end_state = ot_curve_end_state;
    }

    // integrate inner tube
    const Real it_length = std::max(Real(0.0), _it_translation - _ot_translation) * 1000;      // exposed length of the inner tube in mm
    TubeIntegrationState it_start_state = ot_end_state;
    it_start_state.orientation *= GeometryUtils::Rz(_it_rotation - it_start_state.torsional_displacement);
    it_start_state.torsional_displacement = _it_rotation;

    if (it_length > 0)
    {
        // std::cout << "Inner tube length > 0... (=" << it_length << " mm)" << std::endl;
        
        const Vec3r it_K_inv(1/(E*I_it), 1/(E*I_it), 1/(G*J_it));
        std::vector<Real> it_s(NUM_IT_FRAMES);
        for (int i = 0; i < NUM_IT_FRAMES; i++)     
        {
            it_s[i] = _ot_translation*1000 + i*it_length / (NUM_IT_FRAMES - 1);
        }
        
        std::vector<TubeIntegrationState::VecType> it_states = _integrateTubeRK4(it_start_state, it_s, it_K_inv, Vec3r(0,0,0));

        for (int i = 0; i < NUM_IT_FRAMES; i++)
        {
            TubeIntegrationState state = TubeIntegrationState::fromVec(it_states[i]);
            _it_frames[i].setTransform(Geometry::TransformationMatrix(state.orientation, state.position/1000).asMatrix()); // convert position back to m
        }

    }
    else
    {
        Geometry::TransformationMatrix it_start_transform(it_start_state.orientation, it_start_state.position/1000); // convert position back to m
        for (int i = 0; i < NUM_IT_FRAMES; i++)
        {
            _it_frames[i].setTransform(it_start_transform.asMatrix());
        }
    }
    
    _stale_frames = false;

    // for now, the tool position is just the inner tube tip position with no offset
    _tool_position = _it_frames.back().origin();
}

std::vector<VirtuosoArm::TubeIntegrationState::VecType> VirtuosoArm::_integrateTubeRK4(const VirtuosoArm::TubeIntegrationState& tube_base_state, const std::vector<Real>& s, const Vec3r& K_inv, const Vec3r& u_star) const
{
    std::vector<TubeIntegrationState::VecType> tube_states(s.size());
    tube_states[0] = TubeIntegrationState::toVec(tube_base_state);

    auto ode_func = [](Real /*s*/, const TubeIntegrationState::VecType& state_vec, const TubeIntegrationParams& params) -> TubeIntegrationState::VecType {
        
        const TubeIntegrationState state = TubeIntegrationState::fromVec(state_vec);
        TubeIntegrationState state_dot;

        // compute arc length derivates of state
        state_dot.position = state.orientation.col(2);

        const Vec3r u = params.precurvature + params.K_inv.asDiagonal() * state.orientation.transpose() * state.internal_moment;
        state_dot.orientation = state.orientation * Skew3(u);
        state_dot.internal_force = Vec3r::Zero();
        state_dot.internal_moment = -state_dot.position.cross( state.internal_force );
        state_dot.torsional_displacement = u[2];

        return TubeIntegrationState::toVec(state_dot);
    };

    TubeIntegrationParams params;
    params.precurvature = u_star;
    params.K_inv = K_inv;

    for (unsigned i = 1; i < s.size(); i++)
    {
        // using RK4 integration scheme, compute next state
        const Real h = s[i] - s[i-1];
        TubeIntegrationState::VecType next_state = RK4<TubeIntegrationState::VecType, TubeIntegrationParams>(s[i-1], h, tube_states[i-1], params, ode_func);
        tube_states[i] = next_state;

        // std::cout << "Next position: " << next_state[0] << ", " << next_state[1] << ", " << next_state[2] << std::endl;
    }

    return tube_states;
}

Geometry::TransformationMatrix VirtuosoArm::_computeTipTransform(Real ot_rot, Real ot_trans, Real it_rot, Real it_trans)
{
    Geometry::CoordinateFrame arm_base_frame = Geometry::CoordinateFrame(Geometry::TransformationMatrix(_arm_base_rotation, _arm_base_position));

    // compute outer tube base frmae
    // consists of rotating about the current z-axis accordint to outer tube rotation
    Geometry::TransformationMatrix T_rot_z(GeometryUtils::Rz(ot_rot), Vec3r::Zero());
    Geometry::CoordinateFrame ot_base_frame = arm_base_frame * T_rot_z;

    Real swept_angle = std::max(ot_trans - _ot_distal_straight_length, Real(0.0)) / _ot_r_curvature; 
    // frames for the curved section of the outer tube
    const Vec3r curve_point(0.0, -_ot_r_curvature +_ot_r_curvature*std::cos(swept_angle), _ot_r_curvature*std::sin(swept_angle));
    Geometry::TransformationMatrix T_curve(GeometryUtils::Rx(swept_angle), curve_point);
    Geometry::CoordinateFrame ot_curve_end_frame = ot_base_frame * T_curve;

    // frames for the straight section of the outer tube
    // relative transformation along z-axis
    Geometry::TransformationMatrix T_z_trans_ot(Mat3r::Identity(), Vec3r(0,0, std::min(ot_trans, _ot_distal_straight_length)));
    Geometry::CoordinateFrame ot_end_frame = ot_curve_end_frame * T_z_trans_ot;

    // frames for the inner tube
    // unrotate about z-axis for outer tube rotation, then rotate about z-axis with inner tube rotation to get to the beginning of the exposed part of the inner tube
    Geometry::TransformationMatrix T_rot_z_outer_to_inner(GeometryUtils::Rz(it_rot - ot_rot), Vec3r::Zero());
    const Real it_length = std::max(Real(0.0), it_trans - ot_trans);      // exposed length of the inner tube
    // relative transformation matrix along z-axis
    Geometry::TransformationMatrix T_z_trans_it(Mat3r::Identity(), Vec3r(0, 0, it_length));

    Geometry::CoordinateFrame it_end_frame = ot_end_frame * T_rot_z_outer_to_inner * T_z_trans_it;

    return it_end_frame.transform();
}

void VirtuosoArm::_jacobianDifferentialInverseKinematics(const Vec3r& dx)
{
    // _recomputeCoordinateFrames();
    _recomputeCoordinateFramesStaticsModel();
    const Vec3r target_position = tipPosition() + dx;

    for (int i = 0; i < 10; i++)
    {
        const Vec3r pos_err = target_position - tipPosition();
        if (pos_err.norm() < 1e-10)
            break;

        const Mat3r J_a = _3DOFAnalyticalHybridJacobian();
        const Vec3r dq = J_a.colPivHouseholderQr().solve(pos_err);
        _ot_rotation += dq[0];
        _ot_translation += dq[1];
        _it_translation += dq[2];

        // enforce joint limits
        _ot_translation = std::clamp(_ot_translation, Real(0.0), Real(50.0e-3));
        _it_translation = std::clamp(_it_translation, Real(0.0), Real(50.0e-3));

        // _recomputeCoordinateFrames();
        _recomputeCoordinateFramesStaticsModel();
    }

    const Vec3r final_err = target_position - tipPosition();
    std::cout << "Tip pos: " << tipPosition()[0] << ", " << tipPosition()[1] << ", " << tipPosition()[2] << std::endl;
    std::cout << "Tip err: " << final_err[0] << ", " << final_err[1] << ", " << final_err[2] << std::endl;
    std::cout << "q: " << _ot_rotation << ", " << _ot_translation << ", " << _it_translation << std::endl;

    _stale_frames = true;
}

void VirtuosoArm::_hybridDifferentialInverseKinematics(const Vec3r& dx)
{
    // _recomputeCoordinateFrames();
    _recomputeCoordinateFramesStaticsModel();
    const Vec3r target_position = tipPosition() + dx;

    // solve for the outer tube rotation analytically
    Geometry::TransformationMatrix global_to_arm_base = _arm_base_frame.transform().inverse();
    const Vec3r arm_base_pt = global_to_arm_base.rotMat() * target_position + global_to_arm_base.translation();
    const Real target_angle = std::atan2(arm_base_pt[0], -arm_base_pt[1]);

    _ot_rotation = target_angle;

    for (int i = 0; i < 1; i++)
    {
        const Vec3r pos_err = target_position - tipPosition();
        if (pos_err.norm() < 1e-10)
            break;

        const Mat3r J_a = _3DOFAnalyticalHybridJacobian();
        const Vec3r dq = J_a.colPivHouseholderQr().solve(pos_err);
        _ot_translation += dq[1];
        _it_translation += dq[2];

        // enforce constraints
        _ot_translation = std::clamp(_ot_translation, _ot_distal_straight_length+Real(1e-4), Real(20e-3));  // TODO: create var for joint limits
        _it_translation = std::clamp(_it_translation, _ot_distal_straight_length+Real(1e-4), Real(40e-3));
        if (_it_translation < _ot_translation)
        {
            const Real d = _ot_translation - _it_translation;
            _it_translation += d/2;
            _ot_translation -= d/2;
            // _it_translation += d;
        }

        // _recomputeCoordinateFrames();
        _recomputeCoordinateFramesStaticsModel();
    }

    
}

Eigen::Matrix<Real,6,3> VirtuosoArm::_3DOFSpatialJacobian()
{
    // make sure coordinate frames are up to date
    if (_stale_frames)      
        // _recomputeCoordinateFrames();
        _recomputeCoordinateFramesStaticsModel();

    // because there are max() and min() expressions used, we have a couple different cases for the derivatives
    // when outer tube translation >= length of the outer tube straight section, we have some curve in the outer tube
    Real dalpha_d1, dmin_d1;
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
    Real dmax_d1, dmax_d2;
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
    const Real alpha = std::max(_ot_translation - _ot_distal_straight_length, Real(0.0)) / _ot_r_curvature;    // angle swept by the outer tube curve
    const Real beta = _it_rotation - _ot_rotation;    // difference in angle between outer tube and inner tube
    const Real straight_length = std::min(_ot_distal_straight_length, _ot_translation) + std::max(Real(0.0), _it_translation - _ot_translation); // length of the straight section of the combined outer + inner tube

    // precompute some sin and cos
    const Real ct1 = std::cos(_ot_rotation);
    const Real st1 = std::sin(_ot_rotation);
    const Real ca = std::cos(alpha);
    const Real sa = std::sin(alpha);
    const Real cb = std::cos(beta);
    const Real sb = std::sin(beta);

    // std::cout << "alpha: " << alpha << ", beta: " << beta << std::endl;

    Mat4r dT_d_ot_rot, dT_d_ot_trans, dT_d_it_trans;
    dT_d_ot_rot << sb*ct1 + cb*-st1 + cb*st1*ca - sb*ct1*ca, cb*ct1 + sb*st1 - sb*st1*ca - cb*ct1*ca, ct1*sa, ct1*sa*straight_length - ct1*(_ot_r_curvature*ca - _ot_r_curvature),
                   sb*st1 + cb*ct1 - cb*ct1*ca - sb*st1*ca, cb*st1 - sb*ct1 + sb*ct1*ca + -cb*st1*ca, st1*sa, st1*sa*straight_length - st1*(_ot_r_curvature*ca - _ot_r_curvature),
                    -cb*sa, sb*sa, 0, 0,
                    0, 0, 0, 0;

    // std::cout << "dT_d_ot_rot:\n" << dT_d_ot_rot << std::endl;


    dT_d_ot_trans << sb*st1*sa*dalpha_d1, cb*st1*sa*dalpha_d1, st1*ca*dalpha_d1, st1*ca*dalpha_d1*straight_length + st1*sa*(dmin_d1 + dmax_d1) + st1*_ot_r_curvature*sa*dalpha_d1,
                     -sb*ct1*sa*dalpha_d1, -cb*ct1*sa*dalpha_d1, -ct1*ca*dalpha_d1, -ct1*ca*dalpha_d1*straight_length - ct1*sa*(dmin_d1 + dmax_d1) - ct1*_ot_r_curvature*sa*dalpha_d1,
                     sb*ca*dalpha_d1, cb*ca*dalpha_d1, -sa*dalpha_d1, -sa*dalpha_d1*straight_length + ca*(dmin_d1 + dmax_d1) + _ot_r_curvature*ca*dalpha_d1,
                     0, 0, 0, 0;

    // std::cout << "dT_d_ot_trans:\n" << dT_d_ot_trans << std::endl;

    dT_d_it_trans << 0, 0, 0, st1*sa*dmax_d2,
                     0, 0, 0, -ct1*sa*dmax_d2,
                     0, 0, 0, ca*dmax_d2,
                     0, 0, 0, 0;

    // and assemble them into the spatial Jacobian
    const Geometry::TransformationMatrix T_inv = innerTubeEndFrame().transform().inverse();
    
    Eigen::Matrix<Real,6,3> J_s;
    J_s.col(0) = GeometryUtils::Vee_SE3(_arm_base_frame.transform().asMatrix() * dT_d_ot_rot * T_inv.asMatrix());
    J_s.col(1) = GeometryUtils::Vee_SE3(_arm_base_frame.transform().asMatrix() * dT_d_ot_trans * T_inv.asMatrix());
    J_s.col(2) = GeometryUtils::Vee_SE3(_arm_base_frame.transform().asMatrix() * dT_d_it_trans * T_inv.asMatrix());

    return J_s;
}

Eigen::Matrix<Real,6,3> VirtuosoArm::_3DOFNumericalSpatialJacobian()
{
    if (_stale_frames)
        // _recomputeCoordinateFrames();
        _recomputeCoordinateFramesStaticsModel();

    Eigen::Matrix<Real,6,3> J_s;

    const Geometry::TransformationMatrix orig_transform = innerTubeEndFrame().transform();

    // std::cout << "Tip transform (_recomputeCoordinateFrames):\n" << orig_transform.asMatrix() << std::endl;
    // std::cout << "Tip transform (_computeTipTransform):\n" << _computeTipTransform(_ot_rotation, _ot_translation, _it_rotation, _it_translation).asMatrix() << std::endl;
    const Geometry::TransformationMatrix orig_transform_inv = orig_transform.inverse();

    // amount to vary each joint variable
    const Real delta = 1e-6;

    {
        _ot_rotation += delta;
        _recomputeCoordinateFramesStaticsModel();
        Geometry::TransformationMatrix new_transform = innerTubeEndFrame().transform();
        J_s.col(0) = GeometryUtils::Vee_SE3( (new_transform.asMatrix() - orig_transform.asMatrix())/delta * orig_transform_inv.asMatrix());
        _ot_rotation -= delta;
    }
    {
        _ot_translation += delta;
        _recomputeCoordinateFramesStaticsModel();
        Geometry::TransformationMatrix new_transform = innerTubeEndFrame().transform();
        J_s.col(1) = GeometryUtils::Vee_SE3( (new_transform.asMatrix() - orig_transform.asMatrix())/delta * orig_transform_inv.asMatrix());
        _ot_translation -= delta;
    }
    {
        _it_translation += delta;
        _recomputeCoordinateFramesStaticsModel();
        Geometry::TransformationMatrix new_transform = innerTubeEndFrame().transform();
        J_s.col(2) = GeometryUtils::Vee_SE3( (new_transform.asMatrix() - orig_transform.asMatrix())/delta * orig_transform_inv.asMatrix());
        _it_translation -= delta;
    }

    _recomputeCoordinateFramesStaticsModel();

    return J_s;
    
}

Mat3r VirtuosoArm::_3DOFAnalyticalHybridJacobian()
{
    // make sure coordinate frames are up to date
    if (_stale_frames)      
        // _recomputeCoordinateFrames();
        _recomputeCoordinateFramesStaticsModel();

    Eigen::Matrix<Real,3,6> C;
    C << Mat3r::Zero(), Mat3r::Identity();

    // std::cout << "C:\n" << C << std::endl;

    Eigen::Matrix<Real,6,6> R;
    R << innerTubeEndFrame().transform().rotMat(), Mat3r::Zero(), Mat3r::Zero(), innerTubeEndFrame().transform().rotMat();

    // std::cout << "R:\n" << R << std::endl;

    // std::cout << "J_s analytical:\n " << _3DOFSpatialJacobian() << std::endl;
    // std::cout << "J_s numerical:\n " << _3DOFNumericalSpatialJacobian() << std::endl;

    Eigen::Matrix<Real,6,3> J_b = innerTubeEndFrame().transform().inverse().adjoint() * _3DOFNumericalSpatialJacobian();
    // std::cout << "J_b:\n" << J_b << std::endl;
    Mat3r J_a = C * R * J_b;

    // std::cout << "J_a:\n" << J_a << std::endl;
    return J_a;
}

} // namespace Sim