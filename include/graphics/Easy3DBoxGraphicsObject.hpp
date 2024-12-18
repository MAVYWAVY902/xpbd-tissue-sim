#ifndef __EASY3D_BOX_GRAPHICS_OBJECT_HPP
#define __EASY3D_BOX_GRAPHICS_OBJECT_HPP

#include "graphics/BoxGraphicsObject.hpp"

#include <easy3d/core/surface_mesh.h>
#include <easy3d/core/model.h>

namespace Graphics
{

class Easy3DBoxGraphicsObject : public BoxGraphicsObject, public easy3d::Model
{

    public:
    explicit Easy3DBoxGraphicsObject(const std::string& name, const Sim::RigidBox* box);

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

    protected:
    easy3d::SurfaceMesh _e3d_mesh;
    std::vector<easy3d::vec3> _initial_points;

};

} // namespace Graphics

#endif // __EASY3D_BOX_GRAPHICS_OBJECT_HPP