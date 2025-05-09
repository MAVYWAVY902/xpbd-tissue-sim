#ifndef __VIRTUOSO_ARM_HPP
#define __VIRTUOSO_ARM_HPP

#include "simobject/Object.hpp"

#include "geometry/CoordinateFrame.hpp"

#include <array>

class VirtuosoArmConfig;

namespace Sim
{

class XPBDMeshObject_Base;

class VirtuosoArm : public Object
{

    private:
    constexpr static int NUM_OT_CURVE_FRAMES = 15;      // number of coordinate frames defined along the curved section of the outer tube
    constexpr static int NUM_OT_STRAIGHT_FRAMES = 5;    // number of coordinate frames defined along the straight distal section of the outer tube
    constexpr static int NUM_IT_FRAMES = 20;            // number of coordinate frames defined along the inner tube

    constexpr static int MAX_OT_TRANSLATION = 20e-3;    // maximum outer tube translation (joint limit on Virtuoso system)
    constexpr static int MAX_IT_TRANSLATION = 40e-3;    // maximum inner tube translation (joint limit on Virtuoso system)

    constexpr static double GRASPING_RADIUS = 0.002;    // grasping radius for the grasper tool

    public:
    using OuterTubeFramesArray = std::array<Geometry::CoordinateFrame, NUM_OT_CURVE_FRAMES + NUM_OT_STRAIGHT_FRAMES>;
    using InnerTubeFramesArray = std::array<Geometry::CoordinateFrame, NUM_IT_FRAMES>;

    enum ToolType
    {
        SPATULA,
        GRASPER,
        CAUTERY
    };

    public:
    VirtuosoArm(const Simulation* sim, const VirtuosoArmConfig* config);

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

    double innerTubeDiameter() const { return _it_dia; }
    double innerTubeTranslation() const { return _it_translation; }
    double innerTubeRotation() const { return _it_rotation; }

    double outerTubeDiameter() const { return _ot_dia; }
    double outerTubeRadiusOfCurvature() const { return _ot_r_curvature; }
    double outerTubeTranslation() const { return _ot_translation; }
    double outerTubeRotation() const { return _ot_rotation; }
    double outerTubeDistalStraightLength() const { return _ot_distal_straight_length; }
    int toolState() const { return _tool_state; }

    void setInnerTubeTranslation(double t) { _it_translation = (t >= 0) ? t : 0; _recomputeCoordinateFrames(); }
    void setInnerTubeRotation(double r) { _it_rotation = r; _recomputeCoordinateFrames(); }
    void setOuterTubeTranslation(double t) { _ot_translation = (t >= 0) ? t : 0; _recomputeCoordinateFrames(); }
    void setOuterTubeRotation(double r) { _ot_rotation = r; _recomputeCoordinateFrames(); }
    void setToolState(int tool) { _tool_state = tool; }
    void setBasePosition(const Eigen::Vector3d& pos) { _arm_base_position = pos; _recomputeCoordinateFrames(); }
    void setBaseRotation(const Eigen::Matrix3d& rot_mat) { _arm_base_rotation = rot_mat; _recomputeCoordinateFrames();}

    const Geometry::CoordinateFrame& armBaseFrame() const { return _arm_base_frame; }
    const Geometry::CoordinateFrame& outerTubeStartFrame() const { return _ot_frames[0]; }
    const Geometry::CoordinateFrame& outerTubeCurveEndFrame() const { return _ot_frames[NUM_OT_CURVE_FRAMES - 1]; }
    const Geometry::CoordinateFrame& outerTubeEndFrame() const { return _ot_frames.back(); }
    const Geometry::CoordinateFrame& innerTubeStartFrame() const { return _it_frames[0]; }
    const Geometry::CoordinateFrame& innerTubeEndFrame() const { return _it_frames.back(); }

    const OuterTubeFramesArray& outerTubeFrames() const { return _ot_frames; }
    const InnerTubeFramesArray& innerTubeFrames() const { return _it_frames; }

    Eigen::Vector3d tipPosition() const;
    void setTipPosition(const Eigen::Vector3d& new_position);

    const XPBDMeshObject_Base* toolManipulatedObject() const { return _tool_manipulated_object; }
    void setToolManipulatedObject(XPBDMeshObject_Base* obj) { _tool_manipulated_object = obj; }

    void setJointState(double ot_rotation, double ot_translation, double it_rotation, double it_translation, int tool);

    private:
    void _recomputeCoordinateFrames();

    /** Performs any tool actions, if applicable.
     * 
     * For example, if the grasper tool is used and the tool state changes from 0 to 1, we will grasp all mesh vertices in the
     *   _tool_manipulated_object that are within the grasping radius. When the tool state changes from 1 to 0, we will release these vertices.
     */
    void _toolAction();

    void _spatulaToolAction();

    void _grasperToolAction();

    void _cauteryToolAction();

    Geometry::TransformationMatrix _computeTipTransform(double ot_rot, double ot_trans, double it_rot, double it_trans);

    /** Computes the new joint positions given a change in tip position, using only the analytical Jacobian.
     * The Jacobian used is the analytical derivative of the tip transformation matrix (i.e. not a numerical Jacobian)
     *   that does not incorporate the effect of tip forces.
     * 
     * Note: this method easily breaks when the Jacobian becomes singular or the commanded position is outside the reachable workspace of the robot.
     */
    void _jacobianDifferentialInverseKinematics(const Eigen::Vector3d& dx);

    /** Computes the new joint positions given a change in tip position, using a hybrid approach combining analytical geometry and the analytical Jacobian.
     * The outer tube rotation is given by the angle of the commanded position in cylindrical coordinates (i.e. easily solved for analytically).
     *   - note: we limit the change in outer tube rotation (which may be quite large around a singularity) if the change is larger than what the motors can physically do.
     * 
     * The updates in outer tube and inner tube translation are given by the Jacobian.
     * 
     * Note: this method is more robust to the singularity while maintaining the favorable properties of Jacobian-based inverse kinematics.
     * Another note: the Jacobian used also does not incorporate applied forces. 
     */
    void _hybridDifferentialInverseKinematics(const Eigen::Vector3d& dx);

    /** Computes the spatial Jacobian for the tip position w.r.t the outer tube rotation, outer tube translation, and inner tube translation.
     * Used in the 3DOF positional differential inverse kinematics.
     * 
     * (The only joint variables that affect tip position are outer tube rotation, outer tube translation, inner tube translation)
     */
    Eigen::Matrix<double,6,3> _3DOFSpatialJacobian();

    /** Computes the spatial Jacobian for the tip position w.r.t the outer tube rotation, outer tube translation, and inner tube translation.
     * Uses a NUMERICAL approach - varies each joint variable and computes the change in the tip transform.
     */
    Eigen::Matrix<double,6,3> _3DOFNumericalSpatialJacobian();

    /** Computes the hybrid Jacobian for the tip position w.r.t the outer tube rotation, outer tube translation, and inner tube translation.
     * This is a 3x3 matrix (J_a) that relates the velocity of the tip position in the world frame (p_dot) to the actuator velocities (q_dot):
     *      p_dot = J_a * q_dot
     */
    Eigen::Matrix3d _3DOFAnalyticalHybridJacobian();

    private:
    double _it_dia; // inner tube diameter, in m
    double _ot_dia; // outer tube diameter, in m
    double _ot_r_curvature; // outer tube radius of curvature, in m

    double _it_translation; // translation of the inner tube. Right now, assuming that when translation=0, inner tube is fully retracted
    double _it_rotation;    // rotation of inner tube. Right now, assuming angle is measured CCW from positive x-axis 
    double _ot_translation; // translation of the outer tube. Right now, assuming that when translation=0, outer tube is fully retracted
    double _ot_rotation;    // rotation of the outer tube. Right now, assuming rotation=0 corresponds to a curve to the left in the XY plane
    double _ot_distal_straight_length; // the length of the straight section on the distal part of the outer tube

    int _tool_state; // state of the tool (i.e. 1=ON, 0=OFF)
    int _last_tool_state; // the previous state of the tool (needed so that we know when tool state has changed)
    ToolType _tool_type; // type of tool used on this arm
    XPBDMeshObject_Base* _tool_manipulated_object; // the deformable object that this tool is manipulating
    Eigen::Vector3d _tool_position; // position of the tool in global coordinates (note that this may be different than the inner tube tip position)
    std::vector<int> _grasped_vertices; // vertices that are actively being grasped

    Eigen::Vector3d _arm_base_position;
    Eigen::Matrix3d _arm_base_rotation;


    Geometry::CoordinateFrame _arm_base_frame;        // coordinate frame at the tool channel (where it leaves the endoscope)
    
    OuterTubeFramesArray _ot_frames;  // coordinate frames along the backbone of the outer tube
    InnerTubeFramesArray _it_frames;  // coordinate frames along the backbone of the inner tube

    bool _stale_frames;     // true if the joint variables have been updated and the coordinate frames need to be recomputed


};

} // namespace Sim

#endif // __VRITUOSO_ARM_HPP