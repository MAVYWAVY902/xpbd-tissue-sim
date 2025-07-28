#ifndef __VIRTUOSO_ARM_HPP
#define __VIRTUOSO_ARM_HPP

#include "simobject/Object.hpp"
#include "simobject/XPBDMeshObjectBaseWrapper.hpp"

#include "geometry/CoordinateFrame.hpp"
#include "geometry/VirtuosoArmSDF.hpp"

#include <array>
#include <variant>

namespace Config
{
    class VirtuosoArmConfig;
}

namespace Sim
{

class XPBDMeshObject_BasePtrWrapper;

class VirtuosoArm : public Object
{

    private:
    constexpr static int NUM_OT_CURVE_FRAMES = 10;      // number of coordinate frames defined along the curved section of the outer tube
    constexpr static int NUM_OT_STRAIGHT_FRAMES = 5;    // number of coordinate frames defined along the straight distal section of the outer tube
    constexpr static int NUM_OT_FRAMES = NUM_OT_CURVE_FRAMES + NUM_OT_STRAIGHT_FRAMES; // total number of coordinate frames defined along the outer tube
    constexpr static int NUM_IT_FRAMES = 10;            // number of coordinate frames defined along the inner tube

    constexpr static int MAX_OT_TRANSLATION = 20e-3;    // maximum outer tube translation (joint limit on Virtuoso system)
    constexpr static int MAX_IT_TRANSLATION = 40e-3;    // maximum inner tube translation (joint limit on Virtuoso system)

    constexpr static double GRASPING_RADIUS = 0.002;    // grasping radius for the grasper tool

    public:
    using ConfigType = Config::VirtuosoArmConfig;
    using SDFType = Geometry::VirtuosoArmSDF;
    
    using OuterTubeFramesArray = std::array<Geometry::CoordinateFrame, NUM_OT_CURVE_FRAMES + NUM_OT_STRAIGHT_FRAMES>;
    using InnerTubeFramesArray = std::array<Geometry::CoordinateFrame, NUM_IT_FRAMES>;

    /** The type of tool attached to the tip of the arm */
    enum ToolType
    {
        SPATULA,
        GRASPER,
        CAUTERY
    };

    /** State of a tube at a given point along the tube.
     * Used in the statics model for the Virtuoso arm, where we integrate from the base to the tip, keeping track of
     *  - position and orientation
     *  - internal force and moment
     *  - total angle swept about z-axis (i.e. torsional angle displacement)
     * 
     * Two helper methods are provided to convert between the struct form and a 1D vector of states.
     */
    struct TubeIntegrationState
    {
        using VecType = Eigen::Vector<Real, 19>; 

        Vec3r position;                 // position at a point along the tube
        Mat3r orientation;              // orientation at a point along the tube
        Vec3r internal_force;           // (global) internal force at a point along the tube
        Vec3r internal_moment;          // (global) internal moment at a point along the tube
        Real torsional_displacement;    // total torsional displacement at this point along the tube

        /** Converts a TubeIntegrationState struct to a 1D vector (for use in integration) */
        static VecType toVec(const TubeIntegrationState& state)
        {
            VecType vec;
            vec << state.position, state.orientation.reshaped(), state.internal_force, state.internal_moment, state.torsional_displacement;
            return vec;
        }

        /** Converts from a 1D vector back to a TubeIntegrationState */
        static TubeIntegrationState fromVec(const VecType& vec)
        {
            TubeIntegrationState state;
            state.position = vec( Eigen::seqN(0,3) );
            state.orientation = vec( Eigen::seqN(3, 9) ).reshaped(3,3);
            state.internal_force = vec( Eigen::seqN(12, 3) );
            state.internal_moment = vec( Eigen::seqN(15, 3) );
            state.torsional_displacement = vec(18);
            return state;
        }

        // helpers for getting/setting parts of the state when it is represented as a vector
        // this way, we can edit the state directly as a vector without having to convert the whole thing back and forth
        static Vec3r positionFromVec(const VecType& vec) { return vec.head<3>(); }
        static Mat3r orientationFromVec(const VecType& vec) { return vec( Eigen::seqN(3,9) ).reshaped(3,3); }
        static Vec3r internalForceFromVec(const VecType& vec) { return vec( Eigen::seqN(12,3) ); }
        static Vec3r internalMomentFromVec(const VecType& vec) { return vec( Eigen::seqN(15,3) ); }
        static Real torsionalDisplacementFromVec(const VecType& vec) { return vec(18); }

        static void setPositionInVec(VecType& vec, const Vec3r& pos) { vec( Eigen::seqN(0,3) ) = pos; }
        static void setOrientationInVec(VecType& vec, const Mat3r& ori) { vec( Eigen::seqN(3,9) ) = ori.reshaped(); }
        static void setInternalForceInVec(VecType& vec, const Vec3r& int_f) { vec( Eigen::seqN(12,3) ) = int_f; }
        static void setInternalMomentInVec(VecType& vec, const Vec3r& int_m) { vec( Eigen::seqN(15,3) ) = int_m; }
        static void setTorsionalDisplacementInVec(VecType& vec, Real tor_disp) { vec(18) = tor_disp; }

    };

    /** Parameters of a tube needed for integration.
     * These are static parameters of the tube, such as the inverse bending/torsion stiffness and the tube's precurvature.
     */
    struct TubeIntegrationParams
    {
        using VecType = Eigen::Vector<Real, 6>;

        Vec3r precurvature;     // precurvature of the tube
        Vec3r K_inv;            // inverse stiffnesses of the tube (in body frame, hence K is diagonal and can be represented by a 3-vector)
    };

    /** Stores information for applying collision forces onto the tube at the appropriate location.
     * This includes the constraint projector for the collision constraint on the deformable object (calculates the force)
     * as well as the node index and interpolation (between 0 and 1) of the force.
     * The force is assumed to be applied between the node at node index and the next node.
     */
    struct CollisionConstraintInfo
    {
        using ProjectorRefType = Solver::ConstraintProjectorReferenceWrapper<Solver::StaticDeformableCollisionConstraint>;

        ProjectorRefType proj_ref;  // the constraint projector reference that can calculate the constraint force
        int node_index; // "global" node index, i.e. from 0 to NUM_OT_FRAMES + NUM_IT_FRAMES
        Real interp;    // between 0 and 1

        CollisionConstraintInfo(ProjectorRefType&& proj_ref_, int node_index_, Real interp_)
            : proj_ref(proj_ref_), node_index(node_index_), interp(interp_)
        {}

        CollisionConstraintInfo(const ProjectorRefType& proj_ref_, int node_index_, Real interp_)
            : proj_ref(proj_ref_), node_index(node_index_), interp(interp_)
        {}
        };

    public:
    VirtuosoArm(const Simulation* sim, const ConfigType* config);

    /** Returns a string with all relevant information about this object. 
     * @param indent : the level of indentation to use for formatting new lines of the string
    */
    virtual std::string toString(const int indent) const override;
    
    /** Returns a string with the type of the object. */
    virtual std::string type() const override { return "VirtuosoArm"; }

    /** Performs any necessary setup for this object.
     * Called after instantiation (i.e. outside the constructor) and before update() is called for the first time.
     */
    virtual void setup() override;

    /** Evolves this object one time step forward in time. 
     * Completely up to the derived classes to decide how they should step forward in time.
    */
    virtual void update() override;

    virtual void velocityUpdate() override;

    /** Returns the axis-aligned bounding-box (AABB) for this Object in global simulation coordinates. */
    virtual Geometry::AABB boundingBox() const override;

    virtual void createSDF() override 
    { 
        if(!_sdf.has_value()) 
            _sdf = SDFType(this); 
    }

    virtual const SDFType* SDF() const override { return _sdf.has_value() ? &_sdf.value() : nullptr; }

    Real innerTubeOuterDiameter() const { return _it_outer_dia; }
    Real innerTubeInnerDiameter() const { return _it_inner_dia; }
    Real innerTubeTranslation() const { return _it_translation; }
    Real innerTubeRotation() const { return _it_rotation; }

    Real outerTubeOuterDiameter() const { return _ot_outer_dia; }
    Real outerTubeInnerDiameter() const { return _ot_inner_dia; }
    Real outerTubeRadiusOfCurvature() const { return _ot_r_curvature; }
    Real outerTubeTranslation() const { return _ot_translation; }
    Real outerTubeRotation() const { return _ot_rotation; }
    Real outerTubeDistalStraightLength() const { return _ot_distal_straight_length; }
    int toolState() const { return _tool_state; }

    void setInnerTubeTranslation(double t) { _it_translation = (t >= 0) ? t : 0; _stale_frames = true; }
    void setInnerTubeRotation(double r) { _it_rotation = r; _stale_frames = true; }
    void setOuterTubeTranslation(double t) { _ot_translation = (t >= 0) ? t : 0; _stale_frames = true; }
    void setOuterTubeRotation(double r) { _ot_rotation = r; _stale_frames = true; }
    void setToolState(int tool) { _tool_state = tool; }
    void setBasePosition(const Vec3r& pos) { _arm_base_position = pos; _stale_frames = true; }
    void setBaseRotation(const Mat3r& rot_mat) { _arm_base_rotation = rot_mat; _stale_frames = true;}

    const Geometry::CoordinateFrame& armBaseFrame() const { return _arm_base_frame; }
    const Geometry::CoordinateFrame& outerTubeStartFrame() const { return _ot_frames[0]; }
    const Geometry::CoordinateFrame& outerTubeCurveEndFrame() const { return _ot_frames[NUM_OT_CURVE_FRAMES - 1]; }
    const Geometry::CoordinateFrame& outerTubeEndFrame() const { return _ot_frames.back(); }
    const Geometry::CoordinateFrame& innerTubeStartFrame() const { return _it_frames[0]; }
    const Geometry::CoordinateFrame& innerTubeEndFrame() const { return _it_frames.back(); }

    const OuterTubeFramesArray& outerTubeFrames() const { return _ot_frames; }
    const InnerTubeFramesArray& innerTubeFrames() const { return _it_frames; }

    Vec3r actualTipPosition() const;
    Vec3r commandedTipPosition() const{ return _commanded_tip_position; }
    void setCommandedTipPosition(const Vec3r& new_position);

    Vec3r tipForce() const { return _tip_force; }
    Vec3r tipMoment() const { return _tip_moment; }
    void setTipForce(const Vec3r& new_tip_force);
    void setTipMoment(const Vec3r& new_tip_moment);
    void setTipForceAndMoment(const Vec3r& new_tip_force, const Vec3r& new_tip_moment);

    void addCollisionConstraint(CollisionConstraintInfo::ProjectorRefType&& proj_ref, int node_index, Real interp);
    void clearCollisionConstraints();

    const Vec3r& outerTubeNodalForce(int node_index) const { return _ot_nodal_forces[node_index]; }
    const Vec3r& innerTubeNodalForce(int node_index) const { return _it_nodal_forces[node_index]; }
    void setOuterTubeNodalForce(int node_index, const Vec3r& force);
    void setInnerTubeNodalForce(int node_index, const Vec3r& force);

    const XPBDMeshObject_BasePtrWrapper& toolManipulatedObject() const { return _tool_manipulated_object; }
    void setToolManipulatedObject(const XPBDMeshObject_BasePtrWrapper& obj) { _tool_manipulated_object = obj; }

    void setJointState(double ot_rotation, double ot_translation, double it_rotation, double it_translation, int tool);

    private:
    /** Recomputes coordinate frames along the Virtuoso arm using purely geometry and not including any tip forces.
     * This is not used.
     */
    void _recomputeCoordinateFrames();

    /** Recomputes coordinate frames along the Virtuoso arm using a small-deflection assumption statics model.
     * This is able to incorporate tip forces and moments into the model.
     */
    void _recomputeCoordinateFramesStaticsModel();

    /** Recomputes coordinate frames along the Virtuoso arm using a small-deflection assumption statics model,
     * with forces at each "node" (i.e. coordinate frame along the backbone) included.
     */
    void _recomputeCoordinateFramesStaticsModelWithNodalForces();

    std::vector<TubeIntegrationState::VecType> _integrateTubeRK4(
        const TubeIntegrationState& tube_base_state, const std::vector<Real>& s, const Vec3r& K_inv, const Vec3r& u_star) const;

    template <typename ForceIterator>
    std::vector<TubeIntegrationState::VecType> _integrateTubeWithForceBoundariesRK4(
        const TubeIntegrationState& tube_base_state, const std::vector<Real>& s, ForceIterator force_iterator,
        const Vec3r& K_inv, const Vec3r& u_star) const;

    /** Performs any tool actions, if applicable.
     * 
     * For example, if the grasper tool is used and the tool state changes from 0 to 1, we will grasp all mesh vertices in the
     *   _tool_manipulated_object that are within the grasping radius. When the tool state changes from 1 to 0, we will release these vertices.
     */
    void _toolAction();

    void _spatulaToolAction();

    void _grasperToolAction();

    void _cauteryToolAction();

    Geometry::TransformationMatrix _computeTipTransform(Real ot_rot, Real ot_trans, Real it_rot, Real it_trans);

    /** Computes the new joint positions given a change in tip position, using only the analytical Jacobian.
     * The Jacobian used is the analytical derivative of the tip transformation matrix (i.e. not a numerical Jacobian)
     *   that does not incorporate the effect of tip forces.
     * 
     * Note: this method easily breaks when the Jacobian becomes singular or the commanded position is outside the reachable workspace of the robot.
     */
    void _jacobianDifferentialInverseKinematics(const Vec3r& dx);

    /** Computes the new joint positions given a change in tip position, using a hybrid approach combining analytical geometry and the analytical Jacobian.
     * The outer tube rotation is given by the angle of the commanded position in cylindrical coordinates (i.e. easily solved for analytically).
     *   - note: we limit the change in outer tube rotation (which may be quite large around a singularity) if the change is larger than what the motors can physically do.
     * 
     * The updates in outer tube and inner tube translation are given by the Jacobian.
     * 
     * Note: this method is more robust to the singularity while maintaining the favorable properties of Jacobian-based inverse kinematics.
     * Another note: the Jacobian used also does not incorporate applied forces. 
     */
    void _hybridDifferentialInverseKinematics(const Vec3r& dx);

    /** Computes the spatial Jacobian for the tip position w.r.t the outer tube rotation, outer tube translation, and inner tube translation.
     * Used in the 3DOF positional differential inverse kinematics.
     * 
     * (The only joint variables that affect tip position are outer tube rotation, outer tube translation, inner tube translation)
     */
    Eigen::Matrix<Real,6,3> _3DOFSpatialJacobian();

    /** Computes the spatial Jacobian for the tip position w.r.t the outer tube rotation, outer tube translation, and inner tube translation.
     * Uses a NUMERICAL approach - varies each joint variable and computes the change in the tip transform.
     */
    Eigen::Matrix<Real,6,3> _3DOFNumericalSpatialJacobian();

    /** Computes the hybrid Jacobian for the tip position w.r.t the outer tube rotation, outer tube translation, and inner tube translation.
     * This is a 3x3 matrix (J_a) that relates the velocity of the tip position in the world frame (p_dot) to the actuator velocities (q_dot):
     *      p_dot = J_a * q_dot
     */
    Mat3r _3DOFAnalyticalHybridJacobian();

    private:
    Real _it_outer_dia; // inner tube outer diameter, in m
    Real _ot_outer_dia; // outer tube outer diameter, in m
    Real _ot_inner_dia; // outer tube inner diameter, in m
    Real _it_inner_dia; // inner tube inner diameter, in m
    Real _ot_r_curvature; // outer tube radius of curvature, in m

    Real _it_translation; // translation of the inner tube. Right now, assuming that when translation=0, inner tube is fully retracted
    Real _it_rotation;    // rotation of inner tube. Right now, assuming angle is measured CCW from positive x-axis 
    Real _ot_translation; // translation of the outer tube. Right now, assuming that when translation=0, outer tube is fully retracted
    Real _ot_rotation;    // rotation of the outer tube. Right now, assuming rotation=0 corresponds to a curve to the left in the XY plane
    Real _ot_distal_straight_length; // the length of the straight section on the distal part of the outer tube

    int _tool_state; // state of the tool (i.e. 1=ON, 0=OFF)
    int _last_tool_state; // the previous state of the tool (needed so that we know when tool state has changed)
    ToolType _tool_type; // type of tool used on this arm
    XPBDMeshObject_BasePtrWrapper _tool_manipulated_object; // the deformable object that this tool is manipulating
    Vec3r _tool_position; // position of the tool in global coordinates (note that this may be different than the inner tube tip position)
    Vec3r _commanded_tip_position; // tip position of the arm in the absence of tip forces (i.e. where we tell the arm tip to be at)
    std::vector<int> _grasped_vertices; // vertices that are actively being grasped
    std::vector<Solver::ConstraintProjectorReferenceWrapper<Solver::AttachmentConstraint>> _grasping_constraints; // attachment constraints associated with the grasping
    std::vector<CollisionConstraintInfo> _collision_constraints;

    Vec3r _arm_base_position;
    Mat3r _arm_base_rotation;

    Vec3r _tip_force;
    Vec3r _tip_moment;


    Geometry::CoordinateFrame _arm_base_frame;        // coordinate frame at the tool channel (where it leaves the endoscope)
    
    OuterTubeFramesArray _ot_frames;  // coordinate frames along the backbone of the outer tube
    InnerTubeFramesArray _it_frames;  // coordinate frames along the backbone of the inner tube

    std::array<Vec3r, NUM_OT_FRAMES> _ot_nodal_forces;
    std::array<Vec3r, NUM_IT_FRAMES> _it_nodal_forces;

    bool _stale_frames;     // true if the joint variables have been updated and the coordinate frames need to be recomputed

    /** Signed Distance Field for the Virtuoso arm. Must be created explicitly with createSDF(). */
    std::optional<SDFType> _sdf;


};

} // namespace Sim

#endif // __VRITUOSO_ARM_HPP