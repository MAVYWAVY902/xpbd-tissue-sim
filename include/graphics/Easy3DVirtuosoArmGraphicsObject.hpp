#ifndef __EASY_3D_VIRTUOSO_ARM_GRAPHICS_OBJECT_HPP
#define __EASY_3D_VIRTUOSO_ARM_GRAPHICS_OBJECT_HPP

#include "graphics/VirtuosoArmGraphicsObject.hpp"

#include <easy3d/core/surface_mesh.h>
#include <easy3d/core/model.h>

namespace Graphics
{

class Easy3DVirtuosoArmGraphicsObject : public VirtuosoArmGraphicsObject, public easy3d::Model
{
    public:
    explicit Easy3DVirtuosoArmGraphicsObject(const std::string& name, const Sim::VirtuosoArm* virtuoso_arm);

    virtual void update() override;

    /** Returns the easy3d::vec3 vertex cache.
     * Does NOT check if vertices are stale.
     * 
     * A required override for the easy3d::Model class.
     */
    std::vector<easy3d::vec3>& points() override { return _e3d_mesh.points(); };

    /** Returns the easy3d::vec3 vertex cache.
     * Does NOT check if vertices are stale.
     * 
     * A required override for the easy3d::Model class.
     */
    const std::vector<easy3d::vec3>& points() const override { return _e3d_mesh.points(); };

    private:
    /** Creates the initial easy3d::SurfaceMesh for the Virtuoso arm that will be updated throughout the simulation */
    void _generateInitialMesh();

    /** Creates a partial torus whose center curve is in the XY plane mesh given input parameters.
     * @param radius - the radius of the center curve of the torus
     * @param thickness - the thickness of the "tube" of the torus
     * @param max_angle - the maximum angle the torus should go through
     * @param radial_res - the number of subdivisions (i.e. # of discrete sample points) along the center curve
     * @param tubular_res - the number of subdivisions in the circles around the center curve
     */
    easy3d::SurfaceMesh _generateTorusMesh(double radius, double thickness, double max_angle, int radial_res, int tubular_res) const;

    /** Updates the entire mesh given the current state of the Virtuoso arm. */
    void _updateMesh();
    /** Updates just the part of the mesh associated with the outer tube of the Virtuoso arm. */
    void _updateOuterTubeMesh();
    /** Updates just the part of the mesh associated with the inner tube of the Virtuoso arm. */
    void _updateInnerTubeMesh();

    private:
    // TODO: make these settable parameters
    constexpr static int _OT_RADIAL_RES = 20;
    constexpr static int _OT_TUBULAR_RES = 10;
    constexpr static int _IT_TUBULAR_RES = 15;
    
    easy3d::SurfaceMesh _e3d_mesh;

};

} // namespace Graphics

#endif // __EASY_3D_VIRTUOSO_ARM_GRAPHICS_OBJECT_HPP