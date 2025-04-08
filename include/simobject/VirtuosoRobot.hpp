#ifndef __VIRTUOSO_ROBOT_HPP
#define __VIRTUOSO_ROBOT_HPP

#include "config/VirtuosoRobotConfig.hpp"

#include "simobject/Object.hpp"
#include "simobject/VirtuosoArm.hpp"

namespace Sim
{

class VirtuosoRobot : public Object
{
    public:
    explicit VirtuosoRobot(const Simulation* sim, const VirtuosoRobotConfig* config);

    /** Returns a string with all relevant information about this object. 
     * @param indent : the level of indentation to use for formatting new lines of the string
    */
    virtual std::string toString(const int indent) const override;
    
    /** Returns a string with the type of the object. */
    virtual std::string type() const override { return "VirtuosoRobot"; }

    bool hasArm1() const { return (_arm1 != nullptr); }
    bool hasArm2() const { return (_arm2 != nullptr); }
    int numArms() const { return (_arm1 != nullptr) + (_arm2 != nullptr); }
    const VirtuosoArm* arm1() const { return _arm1.get(); }
    const VirtuosoArm* arm2() const { return _arm2.get(); }

    double endoscopeLength() const { return _endoscope_length; }
    double endoscopeDiameter() const { return _endoscope_dia; }
    double armSeparationDistance() const { return _arm_separation_dist; }

    const Geometry::CoordinateFrame& endoscopeFrame() const { return _endoscope_frame; }
    const Geometry::CoordinateFrame& VBFrame() const { return _VB_frame; }
    const Geometry::CoordinateFrame& VRFrame() const { return _VR_frame; }
    const Geometry::CoordinateFrame& camFrame() const { return _cam_frame; }

    virtual Geometry::AABB boundingBox() const override
    {
        // TODO: valid AABB
        return Geometry::AABB(0,0,0,1,1,1);
    }

    /** Performs any necessary setup for this object.
     * Called after instantiation (i.e. outside the constructor) and before update() is called for the first time.
     */
    virtual void setup() override;

    /** Evolves this object one time step forward in time. 
     * Completely up to the derived classes to decide how they should step forward in time.
    */
    virtual void update() override;

    virtual void velocityUpdate() override;

    private:
    double _endoscope_dia;              // diameter (in m) of the endoscope
    double _endoscope_length;           // length (in m) of the endoscope
    double _arm_separation_dist;        // horizontal distance (in m) between the centers of the two arms
    double _optic_vertical_dist;        // vertical distance (in m) between the centers of the arms and the optic
    double _optic_tilt;            // rotation (in rad) of the optic around the positive X axis

    // frame at the center of the end of the endoscope
    // Z points forward away from the endoscope (aligned with initial arm Z axes)
    // Y up, X left
    Geometry::CoordinateFrame _endoscope_frame;

    Geometry::CoordinateFrame _VB_frame;    // frame of the base of the left Virtuoso arm
    Geometry::CoordinateFrame _VR_frame;    // frame of the base of the right Virtuoso arm
    Geometry::CoordinateFrame _cam_frame;   // frame of the camera

    std::unique_ptr<VirtuosoArm> _arm1;
    std::unique_ptr<VirtuosoArm> _arm2;
};

} // namespace Sim

#endif // __VIRTUOSO_ROBOT_HPP