#ifndef __VIRTUOSO_ARM_HPP
#define __VIRTUOSO_ARM_HPP

#include "simobject/Object.hpp"
#include "config/VirtuosoArmConfig.hpp"

#include "geometry/CoordinateFrame.hpp"

#include <array>

namespace Sim
{

class VirtuosoArm : public Object
{

    private:
    constexpr static int NUM_OT_CURVE_FRAMES = 15;      // number of coordinate frames defined along the curved section of the outer tube
    constexpr static int NUM_OT_STRAIGHT_FRAMES = 5;    // number of coordinate frames defined along the straight distal section of the outer tube
    constexpr static int NUM_IT_FRAMES = 20;            // number of coordinate frames defined along the inner tube

    constexpr static int MAX_OT_TRANSLATION = 20e-3;    // maximum outer tube translation (joint limit on Virtuoso system)
    constexpr static int MAX_IT_TRANSLATION = 40e-3;    // maximum inner tube translation (joint limit on Virtuoso system)

    public:
    using OuterTubeFramesArray = std::array<Geometry::CoordinateFrame, NUM_OT_CURVE_FRAMES + NUM_OT_STRAIGHT_FRAMES>;
    using InnerTubeFramesArray = std::array<Geometry::CoordinateFrame, NUM_IT_FRAMES>;

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

    Real innerTubeDiameter() const { return _it_dia; }
    Real innerTubeTranslation() const { return _it_translation; }
    Real innerTubeRotation() const { return _it_rotation; }

    Real outerTubeDiameter() const { return _ot_dia; }
    Real outerTubeRadiusOfCurvature() const { return _ot_r_curvature; }
    Real outerTubeTranslation() const { return _ot_translation; }
    Real outerTubeRotation() const { return _ot_rotation; }
    Real outerTubeDistalStraightLength() const { return _ot_distal_straight_length; }

    void setInnerTubeTranslation(Real t) { _it_translation = (t >= 0) ? t : 0; _recomputeCoordinateFrames(); }
    void setInnerTubeRotation(Real r) { _it_rotation = r; _recomputeCoordinateFrames(); }
    void setOuterTubeTranslation(Real t) { _ot_translation = (t >= 0) ? t : 0; _recomputeCoordinateFrames(); }
    void setOuterTubeRotation(Real r) { _ot_rotation = r; _recomputeCoordinateFrames(); }
    void setBasePosition(const Vec3r& pos) { _arm_base_position = pos; _recomputeCoordinateFrames(); }
    void setBaseRotation(const Mat3r& rot_mat) { _arm_base_rotation = rot_mat; _recomputeCoordinateFrames();}

    const Geometry::CoordinateFrame& armBaseFrame() const { return _arm_base_frame; }
    const Geometry::CoordinateFrame& outerTubeStartFrame() const { return _ot_frames[0]; }
    const Geometry::CoordinateFrame& outerTubeCurveEndFrame() const { return _ot_frames[NUM_OT_CURVE_FRAMES - 1]; }
    const Geometry::CoordinateFrame& outerTubeEndFrame() const { return _ot_frames.back(); }
    const Geometry::CoordinateFrame& innerTubeStartFrame() const { return _it_frames[0]; }
    const Geometry::CoordinateFrame& innerTubeEndFrame() const { return _it_frames.back(); }

    const OuterTubeFramesArray& outerTubeFrames() const { return _ot_frames; }
    const InnerTubeFramesArray& innerTubeFrames() const { return _it_frames; }

    Vec3r tipPosition() const;
    void setTipPosition(const Vec3r& new_position);

    void setActuatorValues(Real ot_rotation, Real ot_translation, Real it_rotation, Real it_translation);

    #ifdef HAVE_CUDA
    virtual void createGPUResource() override { assert(0); /* not implemented */ }
    #endif

    private:
    void _recomputeCoordinateFrames();

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
    Real _it_dia; // inner tube diameter, in m
    Real _ot_dia; // outer tube diameter, in m
    // TODO: change to radius of curvature
    Real _ot_r_curvature; // outer tube radius of curvature, in m

    Real _it_translation; // translation of the inner tube. Right now, assuming that when translation=0, inner tube is fully retracted
    Real _it_rotation;    // rotation of inner tube. Right now, assuming angle is measured CCW from positive x-axis 
    Real _ot_translation; // translation of the outer tube. Right now, assuming that when translation=0, outer tube is fully retracted
    Real _ot_rotation;    // rotation of the outer tube. Right now, assuming rotation=0 corresponds to a curve to the left in the XY plane
    Real _ot_distal_straight_length; // the length of the straight section on the distal part of the outer tube

    Vec3r _arm_base_position;
    Mat3r _arm_base_rotation;


    Geometry::CoordinateFrame _arm_base_frame;        // coordinate frame at the tool channel (where it leaves the endoscope)
    
    OuterTubeFramesArray _ot_frames;  // coordinate frames along the backbone of the outer tube
    InnerTubeFramesArray _it_frames;  // coordinate frames along the backbone of the inner tube

    bool _stale_frames;     // true if the joint variables have been updated and the coordinate frames need to be recomputed


};

} // namespace Sim

#endif // __VRITUOSO_ARM_HPP