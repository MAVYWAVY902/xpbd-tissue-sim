#include "simobject/VirtuosoArm.hpp"
#include "simobject/XPBDMeshObject.hpp"
#include "simulation/Simulation.hpp"

#include "config/simobject/VirtuosoArmConfig.hpp"

#include "utils/GeometryUtils.hpp"
#include "utils/MathUtils.hpp"

#include <math.h>
#include <algorithm>
#include <numeric>

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
    _tool_manipulated_object = (XPBDMeshObject_Base_<false>*)nullptr;

    _arm_base_position = config->baseInitialPosition();
    Vec3r initial_rot_xyz = config->baseInitialRotation() * M_PI / 180.0;
    _arm_base_rotation = GeometryUtils::quatToMat(GeometryUtils::eulXYZ2Quat(initial_rot_xyz[0], initial_rot_xyz[1], initial_rot_xyz[2]));

    _tip_force = Vec3r::Zero();
    _tip_moment = Vec3r::Zero();

    for (int i = 0; i < NUM_OT_FRAMES; i++)
        _ot_nodal_forces[i] = Vec3r::Zero();
    for (int i = 0; i < NUM_IT_FRAMES; i++)
        _it_nodal_forces[i] = Vec3r::Zero();

    _stale_frames = true;

}

std::string VirtuosoArm::toString(const int indent) const
{
    // TODO: better toString
    return Object::toString(indent);
}

Vec3r VirtuosoArm::actualTipPosition() const
{
    return innerTubeEndFrame().origin();
}

void VirtuosoArm::setCommandedTipPosition(const Vec3r& new_position)
{
    // _jacobianDifferentialInverseKinematics(new_position - tipPosition());
    const Geometry::TransformationMatrix T_tip = _computeTipTransform(_ot_rotation, _ot_translation, _it_rotation, _it_translation);
    _hybridDifferentialInverseKinematics(new_position - T_tip.translation());
    _commanded_tip_position = new_position;
    _stale_frames = true;
}

void VirtuosoArm::setTipForce(const Vec3r& new_tip_force)
{
    _tip_force = new_tip_force;
    // _recomputeCoordinateFramesStaticsModelWithNodalForces();
    _stale_frames = true;
}

void VirtuosoArm::setTipMoment(const Vec3r& new_tip_moment)
{
    _tip_moment = new_tip_moment;
    // _recomputeCoordinateFramesStaticsModelWithNodalForces();
    _stale_frames = true;
}

void VirtuosoArm::setTipForceAndMoment(const Vec3r& new_tip_force, const Vec3r& new_tip_moment)
{
    _tip_force = new_tip_force;
    _tip_moment = new_tip_moment;
    // _recomputeCoordinateFramesStaticsModelWithNodalForces();
    _stale_frames = true;
}

void VirtuosoArm::setOuterTubeNodalForce(int node_index, const Vec3r& force)
{
    assert(node_index < NUM_OT_FRAMES);
    _ot_nodal_forces[node_index] = force;
    _stale_frames = true;
}

void VirtuosoArm::setInnerTubeNodalForce(int node_index, const Vec3r& force)
{
    assert(node_index < NUM_IT_FRAMES);
    _it_nodal_forces[node_index] = force;
    _stale_frames = true;
}

void VirtuosoArm::setJointState(double ot_rotation, double ot_translation, double it_rotation, double it_translation, int tool)
{
    _ot_rotation = ot_rotation;
    _ot_translation = ot_translation;
    _it_rotation = it_rotation;
    _it_translation = it_translation;
    _tool_state = tool;

    // _recomputeCoordinateFrames();
    _stale_frames = true;
}

void VirtuosoArm::addCollisionConstraint(VirtuosoArm::CollisionConstraintInfo::ProjectorRefType&& proj_ref, int node_index, Real interp)
{
    _collision_constraints.emplace_back(std::move(proj_ref), node_index, interp);
}

void VirtuosoArm::clearCollisionConstraints()
{
    _collision_constraints.clear();
}

void VirtuosoArm::setup()
{
    _recomputeCoordinateFrames();

    _commanded_tip_position = actualTipPosition();
}

void VirtuosoArm::update()
{
    const Geometry::TransformationMatrix T_tip = _computeTipTransform(_ot_rotation, _ot_translation, _it_rotation, _it_translation);
    _hybridDifferentialInverseKinematics(_commanded_tip_position - T_tip.translation());
    if (_stale_frames)
    {
        // _recomputeCoordinateFrames();
        _recomputeCoordinateFramesStaticsModelWithNodalForces();
    }

    // _toolAction();
}

void VirtuosoArm::velocityUpdate()
{
    // perform the tool action AFTER the update step so that the XPBD update for deformable objects has already occurred so that
    // we can compute the constraint forces associated with projections of various constraints
    _toolAction();

    // apply forces from collision constraints
    std::vector<Vec3r> new_forces(NUM_OT_FRAMES + NUM_IT_FRAMES, Vec3r::Zero());
    for (const auto& collision : _collision_constraints)
    {
        std::vector<Vec3r> forces = collision.proj_ref.constraintForces();
        Vec3r net_force = std::reduce(forces.cbegin(), forces.cend());
        new_forces[collision.node_index] += net_force*(1-collision.interp);
        new_forces[collision.node_index+1] += net_force*collision.interp;
    }

    const Real frac = 0.01;
    for (int i = 0; i < NUM_OT_FRAMES; i++)
    {
        const Vec3r& cur_force = outerTubeNodalForce(i);
        Vec3r new_force = (1-frac)*cur_force + frac*new_forces[i];
        setOuterTubeNodalForce(i, new_force);
    }
    for (int i = 0; i < NUM_IT_FRAMES; i++)
    {
        const Vec3r& cur_force = innerTubeNodalForce(i);
        Vec3r new_force = (1-frac)*cur_force + frac*new_forces[NUM_OT_FRAMES + i];
        setInnerTubeNodalForce(i, new_force);
    }
    _stale_frames = true;
}

/** Returns the axis-aligned bounding-box (AABB) for this Object in global simulation coordinates. */
Geometry::AABB VirtuosoArm::boundingBox() const
{
    return Geometry::AABB(Vec3r::Zero(), Vec3r::Zero());
}

Real VirtuosoArm::_outerTubeClearanceAngle(Real ot_trans)
{
    Real tubed_length = std::max(_ot_distal_straight_length - ot_trans, Real(0.0));

    Real sqrt_arg = tubed_length*tubed_length + 2*OT_ENDOSCOPE_CLEARANCE*OT_RADIUS_OF_CURVATURE - OT_ENDOSCOPE_CLEARANCE*OT_ENDOSCOPE_CLEARANCE;
    Real atan_arg = (tubed_length - std::sqrt(sqrt_arg)) / (OT_ENDOSCOPE_CLEARANCE - 2*OT_RADIUS_OF_CURVATURE);
    return 2*std::atan(atan_arg);
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
                    int v = _tool_manipulated_object.mesh()->getClosestVertex(Vec3r(x, y, z));

                    // make sure v is inside grasping sphere
                    if ((_tool_position - _tool_manipulated_object.mesh()->vertex(v)).norm() <= GRASPING_RADIUS)
                        if (!_tool_manipulated_object.vertexFixed(v))
                        {
                            const Vec3r attachment_offset = (_tool_manipulated_object.mesh()->vertex(v) - _tool_position) * 0.9;
                            vertices_to_grasp[v] = attachment_offset;
                        }
                }
            }
        }

        for (const auto& [v, offset] : vertices_to_grasp)
        {
            Solver::ConstraintProjectorReferenceWrapper<Solver::AttachmentConstraint> proj_ref =
                _tool_manipulated_object.addAttachmentConstraint(v, &_tool_position, offset);
            _grasping_constraints.push_back(std::move(proj_ref));
        }
    }

    // if tool state has changed from 1 to 0, stop grasping
    else if (_tool_state == 0 && _last_tool_state == 1)
    {
        /** TODO: remove just the attachment constraints associated with grasping with this object */
        _tool_manipulated_object.clearAttachmentConstraints();
        _grasping_constraints.clear();
    }

    // apply tip forces
    Vec3r total_force = Vec3r::Zero();
    for (const auto& proj : _grasping_constraints)
    {
        std::vector<Vec3r> forces = proj.constraintForces();
        total_force += forces[0]; // attachment constraint only affects one vertex, so the vector only has 1 element
    }

    // smooth forces
    Vec3r new_tip_force = 0.99*tipForce() + -0.01* -total_force/1;
    setTipForce(new_tip_force);
    
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
    // consists of rotating about the current z-axis according to outer tube rotation
    // and rotating about the new x-axis according to the clearance angle
    Geometry::TransformationMatrix T_rot_z(GeometryUtils::Rz(_ot_rotation), Vec3r::Zero());
    Geometry::TransformationMatrix T_rot_x(GeometryUtils::Rx(_outerTubeClearanceAngle(_ot_translation)), Vec3r::Zero());
    Geometry::CoordinateFrame ot_base_frame = _arm_base_frame * T_rot_z * T_rot_x;

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
    // add contributions from force and moment at tube tip
    Vec3r base_moment = _tip_moment + tip_position.cross(_tip_force);
    Vec3r base_force = _tip_force;


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

void VirtuosoArm::_recomputeCoordinateFramesStaticsModelWithNodalForces()
{
    // compute arm base frame based on current position and orientation
    _arm_base_frame = Geometry::CoordinateFrame(Geometry::TransformationMatrix(_arm_base_rotation, _arm_base_position));

    // compute outer tube base frame
    // consists of rotating about the current z-axis according to outer tube rotation
    // and rotating about the new x-axis according to the clearance angle
    Geometry::TransformationMatrix T_rot_z(GeometryUtils::Rz(_ot_rotation), Vec3r::Zero());
    Geometry::TransformationMatrix T_rot_x(GeometryUtils::Rx(_outerTubeClearanceAngle(_ot_translation)), Vec3r::Zero());
    Geometry::CoordinateFrame ot_base_frame = _arm_base_frame * T_rot_z * T_rot_x;

    // calculate internal forces and moments carried at the base of the tube collection
    const Vec3r& tip_position = innerTubeEndFrame().origin();
    // add contributions from force and moment at tube tip
    Vec3r base_moment = _tip_moment + tip_position.cross(_tip_force);
    Vec3r base_force = _tip_force;
    // add contributions from forces applied along the tube
    for (int i = 0; i < NUM_OT_FRAMES; i++)
    {
        base_moment += _ot_frames[i].origin().cross(_ot_nodal_forces[i]);
        base_force += _ot_nodal_forces[i];
    }
    for (int i = 0; i < NUM_IT_FRAMES; i++)
    {
        base_moment += _it_frames[i].origin().cross(_it_nodal_forces[i]);
        base_force += _it_nodal_forces[i];
    }

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

        std::vector<TubeIntegrationState::VecType> ot_curve_states = 
            _integrateTubeWithForceBoundariesRK4(ot_base_state, ot_curve_s, _ot_nodal_forces.cbegin(), K_inv, precurvature);
        
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
        std::vector<TubeIntegrationState::VecType> ot_straight_states = 
            _integrateTubeWithForceBoundariesRK4(ot_curve_end_state, ot_straight_s, _ot_nodal_forces.cbegin() + NUM_OT_STRAIGHT_FRAMES, ot_K_inv, Vec3r(0,0,0));
        

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
        
        std::vector<TubeIntegrationState::VecType> it_states = _integrateTubeWithForceBoundariesRK4(it_start_state, it_s, _it_nodal_forces.cbegin(), it_K_inv, Vec3r(0,0,0));

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

    // a function that uses Cosserat rod ODEs to return the arc length derivative of the state at a given point along the tube
    auto ode_func = [](Real /*s*/, const TubeIntegrationState::VecType& state_vec, const TubeIntegrationParams& params) -> TubeIntegrationState::VecType {
        
        const TubeIntegrationState state = TubeIntegrationState::fromVec(state_vec);
        TubeIntegrationState state_dot; // note that these are ARC LENGTH derivatives, not time derivatives

        // p_dot = R * e_3
        state_dot.position = state.orientation.col(2);  

        // u = u_star + K^-1 * R^T * m
        const Vec3r u = params.precurvature + params.K_inv.asDiagonal() * state.orientation.transpose() * state.internal_moment;

        // R_dot = R * u^
        state_dot.orientation = state.orientation * MathUtils::Skew3(u);

        // internal force is constant ==> n_dot = 0
        state_dot.internal_force = Vec3r::Zero();

        // m_dot = -p x n
        state_dot.internal_moment = -state_dot.position.cross( state.internal_force );

        // theta_dot = u_3
        state_dot.torsional_displacement = u[2];

        return TubeIntegrationState::toVec(state_dot);
    };

    // integration parameters for the tube consist of the precurvature of the tube and the inverse bending stiffnesses
    TubeIntegrationParams params;
    params.precurvature = u_star;
    params.K_inv = K_inv;

    return MathUtils::RK4<TubeIntegrationState::VecType, TubeIntegrationParams>(TubeIntegrationState::toVec(tube_base_state), s, params, ode_func);
}

template <typename ForceIterator>
std::vector<VirtuosoArm::TubeIntegrationState::VecType> VirtuosoArm::_integrateTubeWithForceBoundariesRK4(
        const TubeIntegrationState& tube_base_state, const std::vector<Real>& s, ForceIterator force_iterator,
        const Vec3r& K_inv, const Vec3r& u_star) const
{
    // a function that uses Cosserat rod ODEs to return the arc length derivative of the state at a given point along the tube
    auto ode_func = [](Real /*s*/, const TubeIntegrationState::VecType& state_vec, const TubeIntegrationParams& params) -> TubeIntegrationState::VecType {
        
        const TubeIntegrationState state = TubeIntegrationState::fromVec(state_vec);
        TubeIntegrationState state_dot; // note that these are ARC LENGTH derivatives, not time derivatives

        // p_dot = R * e_3
        state_dot.position = state.orientation.col(2);  

        // u = u_star + K^-1 * R^T * m
        const Vec3r u = params.precurvature + params.K_inv.asDiagonal() * state.orientation.transpose() * state.internal_moment;

        // R_dot = R * u^
        state_dot.orientation = state.orientation * MathUtils::Skew3(u);

        // internal force is constant ==> n_dot = 0
        state_dot.internal_force = Vec3r::Zero();

        // m_dot = -p x n
        state_dot.internal_moment = -state_dot.position.cross( state.internal_force );

        // theta_dot = u_3
        state_dot.torsional_displacement = u[2];

        return TubeIntegrationState::toVec(state_dot);
    };

    // we need to break up the integration of the tube depending on the number of applied forces along its length
    std::vector<unsigned> tube_intervals;
    tube_intervals.reserve(s.size());
    tube_intervals.push_back(0);
    for (unsigned i = 1; i < s.size()-1; i++)
    {
        const Vec3r& F = *(force_iterator + i);
        // if force at this node is not zero, add its index to intervals
        if (!F.isZero(1e-8))
        {
            tube_intervals.push_back(i);
        }
    }
    tube_intervals.push_back(s.size()-1);

    // REMOVE: print out tube intervals
    // std::cout << "Tube intervals: ";
    // for (unsigned i = 0; i < tube_intervals.size()-1; i++)
    // {
    //     unsigned int_start = tube_intervals[i];
    //     unsigned int_end = tube_intervals[i+1];
    //     std::cout << "[" << int_start << ", " << int_end << "]";
    // }
    // std::cout << std::endl;

    // integration parameters for the tube consist of the precurvature of the tube and the inverse bending stiffnesses
    TubeIntegrationParams params;
    params.precurvature = u_star;
    params.K_inv = K_inv;

    std::vector<TubeIntegrationState::VecType> states(s.size());
    states[0] = TubeIntegrationState::toVec(tube_base_state);

    // integrate through each interval
    for (unsigned i = 0; i < tube_intervals.size()-1; i++)
    {
        unsigned int_start = tube_intervals[i];
        unsigned int_end = tube_intervals[i+1];

        const Vec3r& applied_force = *(force_iterator + int_start);
        const Vec3r current_internal_force = TubeIntegrationState::internalForceFromVec(states[int_start]);
        TubeIntegrationState::setInternalForceInVec(states[int_start], current_internal_force - applied_force);

        MathUtils::RK4<TubeIntegrationState::VecType, TubeIntegrationParams>(states[int_start], s.cbegin()+int_start, s.cbegin()+int_end+1, params, ode_func, states.begin()+int_start);
    }

    return states;
}

Geometry::TransformationMatrix VirtuosoArm::_computeTipTransform(Real ot_rot, Real ot_trans, Real it_rot, Real it_trans)
{
    Geometry::CoordinateFrame arm_base_frame = Geometry::CoordinateFrame(Geometry::TransformationMatrix(_arm_base_rotation, _arm_base_position));

    // compute outer tube base frame
    // consists of rotating about the current z-axis according to outer tube rotation
    // and rotating about the new x-axis according to the clearance angle
    Geometry::TransformationMatrix T_rot_z(GeometryUtils::Rz(ot_rot), Vec3r::Zero());
    Geometry::TransformationMatrix T_rot_x(GeometryUtils::Rx(_outerTubeClearanceAngle(ot_trans)), Vec3r::Zero());
    Geometry::CoordinateFrame ot_base_frame = arm_base_frame * T_rot_z * T_rot_x;

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
    // _recomputeCoordinateFramesStaticsModelWithNodalForces();
    const Geometry::TransformationMatrix T_tip = _computeTipTransform(_ot_rotation, _ot_translation, _it_rotation, _it_translation);
    const Vec3r target_position = T_tip.translation() + dx;

    for (int i = 0; i < 10; i++)
    {
        const Geometry::TransformationMatrix T_tip = _computeTipTransform(_ot_rotation, _ot_translation, _it_rotation, _it_translation);
        const Vec3r pos_err = target_position - T_tip.translation();
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
        _recomputeCoordinateFramesStaticsModelWithNodalForces();
    }

    _stale_frames = true;
}

void VirtuosoArm::_hybridDifferentialInverseKinematics(const Vec3r& dx)
{
    if (dx.norm() < 1e-6)
        return;

    // _recomputeCoordinateFrames();
    // _recomputeCoordinateFramesStaticsModelWithNodalForces();
    const Geometry::TransformationMatrix T_tip = _computeTipTransform(_ot_rotation, _ot_translation, _it_rotation, _it_translation);
    const Vec3r target_position = T_tip.translation() + dx;

    // solve for the outer tube rotation analytically
    Geometry::TransformationMatrix global_to_arm_base = _arm_base_frame.transform().inverse();
    const Vec3r arm_base_pt = global_to_arm_base.rotMat() * target_position + global_to_arm_base.translation();
    // the multiple of 2*pi to add to angles (avoids angle jumps from 180 to -180 and vice versa)
    int n_rev = static_cast<int>((_ot_rotation+M_PI) / (2*M_PI));
    Real target_angle = n_rev*2*M_PI + std::atan2(arm_base_pt[0], -arm_base_pt[1]);

    // avoid angle jumps from 180 to -180 and from -180 to 180
    if (_ot_rotation > n_rev*2*M_PI + M_PI/2.0 && target_angle < n_rev*2*M_PI - M_PI/2.0)
        target_angle += 2*M_PI;
    else if (_ot_rotation < n_rev*2*M_PI - M_PI/2.0 && target_angle > n_rev*2*M_PI + M_PI/2.0)
        target_angle += -2*M_PI;

    // compute max allowable changes in joint variables, given by motor limitations
    Real max_ot_rot_change = MAX_OT_ROTATION_SPEED * _sim->dt(); // rad/s
    Real max_ot_trans_change = MAX_OT_TRANSLATION_SPEED * _sim->dt(); // m/s
    Real max_it_trans_change = MAX_IT_TRANSLATION_SPEED * _sim->dt(); // m/s

    // clamp the change in outer tube rotation and update
    Real d_ot_rot = std::clamp(target_angle - _ot_rotation, -max_ot_rot_change, max_ot_rot_change);
    _ot_rotation += d_ot_rot;


    /** run 1 iteration of Jacobian-based differential inverse kinematics */
    // the solution may not be exact THIS frame, but will converge over a few successive frames
    // we can get away with this because we update at such a fast rate

    // compute position error
    const Geometry::TransformationMatrix T_tip_new = _computeTipTransform(_ot_rotation, _ot_translation, _it_rotation, _it_translation);
    const Vec3r pos_err = target_position - T_tip_new.translation();

    // get Jacobian and solve for joint updates (only outer tube translation and inner tube translation are used)
    const Mat3r J_a = _3DOFAnalyticalHybridJacobian();
    const Vec3r dq = J_a.colPivHouseholderQr().solve(pos_err);
    // update translational joint variables, clamping them to their acceptable ranges
    _ot_translation += std::clamp(dq[1], -max_ot_trans_change, max_ot_trans_change);
    _it_translation += std::clamp(dq[2], -max_it_trans_change, max_it_trans_change);

    // enforce constraints
    _ot_translation = std::clamp(_ot_translation, Real(0.0), Real(MAX_OT_TRANSLATION));
    _it_translation = std::clamp(_it_translation, Real(0.0), Real(MAX_IT_TRANSLATION));
    if (_it_translation < _ot_translation)
    {
        const Real d = _ot_translation - _it_translation;
        _it_translation += d/2;
        _ot_translation -= d/2;
    }
    

    // mark that the joint state has been updated so the coordinate frames are stale
    _stale_frames = true;
}

Eigen::Matrix<Real,6,3> VirtuosoArm::_3DOFSpatialJacobian()
{

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

    // derviative of max(length of OT straight section - OT translation, 0)
    Real dtubed_length_d1;
    if (_ot_translation > _ot_distal_straight_length)
    {
        dtubed_length_d1 = 0;
    }
    else
    {
        dtubed_length_d1 = -1;
    }

    // derivative of clearance angle w.r.t. d1
    Real tubed_length = std::max(_ot_distal_straight_length - _ot_translation, Real(0.0));
    Real sqrt_arg = tubed_length*tubed_length + 2*OT_ENDOSCOPE_CLEARANCE*OT_RADIUS_OF_CURVATURE - OT_ENDOSCOPE_CLEARANCE*OT_ENDOSCOPE_CLEARANCE;
    Real atan_arg = (tubed_length - std::sqrt(sqrt_arg)) / (OT_ENDOSCOPE_CLEARANCE - 2*OT_RADIUS_OF_CURVATURE);
    Real dca_d1 = 2 / (1+atan_arg*atan_arg) / (OT_ENDOSCOPE_CLEARANCE - 2*OT_RADIUS_OF_CURVATURE) * (1 - tubed_length/std::sqrt(sqrt_arg)) * dtubed_length_d1;
    
    Real clearance_angle = _outerTubeClearanceAngle(_ot_translation);
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
    const Real cg = std::cos(clearance_angle);
    const Real sg = std::sin(clearance_angle);

    // the chain of transforms that make up forward kinematics
    Mat4r T1, T2, T3, T4;
    T1 <<   ct1, -st1, 0, 0,
            st1, ct1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    
    T2 <<   1, 0, 0, 0,
            0, cg, -sg, 0,
            0, sg, cg, 0,
            0, 0, 0, 1;
    
    T3 <<   1, 0, 0, 0,
            0, ca, -sa, _ot_r_curvature*ca - _ot_r_curvature,
            0, sa, ca, _ot_r_curvature*sa,
            0, 0, 0, 1;
    
    T4 <<   cb, -sb, 0, 0,
            sb, cb, 0, 0,
            0, 0, 1, straight_length,
            0, 0, 0, 1;
            
    // derivatives w.r.t outer tube rotation (theta1)
    Mat4r dT1_dt1, dT4_dt1;
    dT1_dt1 <<  -st1, -ct1, 0, 0,
                ct1, -st1, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0;
    
    dT4_dt1 <<  sb, cb, 0, 0,
                -cb, sb, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0;

    // derivatives w.r.t. outer tube translation (d1)
    Mat4r dT2_dd1, dT3_dd1, dT4_dd1;
    dT2_dd1 <<  0, 0, 0, 0,
                0, -sg*dca_d1, -cg*dca_d1, 0,
                0, cg*dca_d1, -sg*dca_d1, 0, 
                0, 0, 0, 0;
    
    dT3_dd1 <<  0, 0, 0, 0,
                0, -sa*dalpha_d1, -ca*dalpha_d1, -_ot_r_curvature*sa*dalpha_d1,
                0, ca*dalpha_d1, -sa*dalpha_d1, _ot_r_curvature*ca*dalpha_d1,
                0, 0, 0, 0;
    
    dT4_dd1 <<  0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, dmin_d1 + dmax_d1,
                0, 0, 0, 0;

    // derivatives w.r.t. inner tube translation (d2)
    Mat4r dT4_dd2;
    dT4_dd2 <<  0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, dmax_d2,
                0, 0, 0, 0;

    
    // using product rule, evaluate dT/dq
    Mat4r dT_d_ot_rot = dT1_dt1*T2*T3*T4 + T1*T2*T3*dT4_dt1;
    Mat4r dT_d_ot_trans = T1*dT2_dd1*T3*T4 + T1*T2*dT3_dd1*T4 + T1*T2*T3*dT4_dd1;
    Mat4r dT_d_it_trans = T1*T2*T3*dT4_dd2; 

    // and assemble them into the spatial Jacobian
    const Geometry::TransformationMatrix T_tip = _computeTipTransform(_ot_rotation, _ot_translation, _it_rotation, _it_translation);
    const Geometry::TransformationMatrix T_inv = T_tip.inverse();
    
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
        _recomputeCoordinateFramesStaticsModelWithNodalForces();

    Eigen::Matrix<Real,6,3> J_s;

    const Geometry::TransformationMatrix orig_transform = innerTubeEndFrame().transform();

    // std::cout << "Tip transform (_recomputeCoordinateFrames):\n" << orig_transform.asMatrix() << std::endl;
    // std::cout << "Tip transform (_computeTipTransform):\n" << _computeTipTransform(_ot_rotation, _ot_translation, _it_rotation, _it_translation).asMatrix() << std::endl;
    const Geometry::TransformationMatrix orig_transform_inv = orig_transform.inverse();

    // amount to vary each joint variable
    const Real delta = 1e-6;

    {
        _ot_rotation += delta;
        _recomputeCoordinateFramesStaticsModelWithNodalForces();
        Geometry::TransformationMatrix new_transform = innerTubeEndFrame().transform();
        J_s.col(0) = GeometryUtils::Vee_SE3( (new_transform.asMatrix() - orig_transform.asMatrix())/delta * orig_transform_inv.asMatrix());
        _ot_rotation -= delta;
    }
    {
        _ot_translation += delta;
        _recomputeCoordinateFramesStaticsModelWithNodalForces();
        Geometry::TransformationMatrix new_transform = innerTubeEndFrame().transform();
        J_s.col(1) = GeometryUtils::Vee_SE3( (new_transform.asMatrix() - orig_transform.asMatrix())/delta * orig_transform_inv.asMatrix());
        _ot_translation -= delta;
    }
    {
        _it_translation += delta;
        _recomputeCoordinateFramesStaticsModelWithNodalForces();
        Geometry::TransformationMatrix new_transform = innerTubeEndFrame().transform();
        J_s.col(2) = GeometryUtils::Vee_SE3( (new_transform.asMatrix() - orig_transform.asMatrix())/delta * orig_transform_inv.asMatrix());
        _it_translation -= delta;
    }

    _recomputeCoordinateFramesStaticsModelWithNodalForces();

    return J_s;
    
}

Mat3r VirtuosoArm::_3DOFAnalyticalHybridJacobian()
{
    // make sure coordinate frames are up to date
    // if (_stale_frames)      
        // _recomputeCoordinateFrames();
        // _recomputeCoordinateFramesStaticsModelWithNodalForces();

    const Geometry::TransformationMatrix T_tip = _computeTipTransform(_ot_rotation, _ot_translation, _it_rotation, _it_translation);

    Eigen::Matrix<Real,3,6> C;
    C << Mat3r::Zero(), Mat3r::Identity();

    // std::cout << "C:\n" << C << std::endl;

    Eigen::Matrix<Real,6,6> R;
    R << T_tip.rotMat(), Mat3r::Zero(), Mat3r::Zero(), T_tip.rotMat();

    // std::cout << "R:\n" << R << std::endl;

    // std::cout << "J_s analytical:\n " << _3DOFSpatialJacobian() << std::endl;
    // std::cout << "J_s numerical:\n " << _3DOFNumericalSpatialJacobian() << std::endl;

    Eigen::Matrix<Real,6,3> J_b = T_tip.inverse().adjoint() * _3DOFSpatialJacobian();
    // std::cout << "J_b:\n" << J_b << std::endl;
    Mat3r J_a = C * R * J_b;

    // std::cout << "J_a:\n" << J_a << std::endl;
    return J_a;
}

} // namespace Sim