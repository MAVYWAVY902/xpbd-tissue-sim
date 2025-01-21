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
    void _generateMesh();
    void _generateTorusMesh(double radius, double thickness, double max_angle, int radial_res, int tubular_res);

    protected:
    easy3d::SurfaceMesh _e3d_mesh;

};

} // namespace Graphics

#endif // __EASY_3D_VIRTUOSO_ARM_GRAPHICS_OBJECT_HPP