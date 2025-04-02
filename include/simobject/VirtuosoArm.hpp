#ifndef __VIRTUOSO_ARM_HPP
#define __VIRTUOSO_ARM_HPP

#include "simobject/Object.hpp"
#include "config/VirtuosoArmConfig.hpp"

#include "geometry/CoordinateFrame.hpp"

namespace Sim
{

class VirtuosoArm : public Object
{

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
    Eigen::Vector3d outerTubePosition() const { return _ot_position; }

    void setInnerTubeTranslation(double t) { _it_translation = (t >= 0) ? t : 0; }
    void setInnerTubeRotation(double r) { _it_rotation = r; }
    void setOuterTubeTranslation(double t) { _ot_translation = (t >= 0) ? t : 0; }
    void setOuterTubeRotation(double r) { _ot_rotation = r; }
    void setOuterTubePosition(const Eigen::Vector3d& p) { _ot_position = p; }

    const Geometry::CoordinateFrame& endoscopeFrame() const { return _endoscope_frame; }
    const Geometry::CoordinateFrame& outerTubeBaseFrame() const { return _ot_base_frame; }
    const Geometry::CoordinateFrame& outerTubeCurveEndFrame() const { return _ot_curve_end_frame; }
    const Geometry::CoordinateFrame& outerTubeEndFrame() const { return _ot_end_frame; }
    const Geometry::CoordinateFrame& innerTubeEndFrame() const { return _it_end_frame; }

    private:
    void _recomputeCoordinateFrames();

    private:
    double _it_dia; // inner tube diameter, in m
    double _ot_dia; // outer tube diameter, in m
    // TODO: change to radius of curvature
    double _ot_r_curvature; // outer tube radius of curvature, in m

    double _it_translation; // translation of the inner tube. Right now, assuming that when translation=0, inner tube is fully retracted
    double _it_rotation;    // rotation of inner tube. Right now, assuming angle is measured CCW from positive x-axis 
    double _ot_translation; // translation of the outer tube. Right now, assuming that when translation=0, outer tube is fully retracted
    double _ot_rotation;    // rotation of the outer tube. Right now, assuming rotation=0 corresponds to a curve to the left in the XY plane
    double _ot_distal_straight_length; // the length of the straight section on the distal part of the outer tube

    Eigen::Vector3d _ot_position; // the position of the base of the outer tube


    Geometry::CoordinateFrame _endoscope_frame;       // coordinate frame of the end of the endoscope
    Geometry::CoordinateFrame _ot_base_frame;         // coordinate frame of the base of the robot (i.e. this can be rotated relative tot the endoscope frame)
    Geometry::CoordinateFrame _ot_curve_end_frame;    // coordinate frame at the end of the curved section of the outer tube
    Geometry::CoordinateFrame _ot_end_frame;          // coordinate frame at the end of the outer tube
    Geometry::CoordinateFrame _it_end_frame;          // coordinate frame at the end of the inner tube


};

} // namespace Sim

#endif // __VRITUOSO_ARM_HPP