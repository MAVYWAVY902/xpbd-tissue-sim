#ifndef __MESH_SDF_HPP
#define __MESH_SDF_HPP

// define MESH2SDF_DOUBLE_PRECISION so the Mesh2SDF library compiles with double precision
#ifndef MESH2SDF_DOUBLE_PRECISION
#define MESH2SDF_DOUBLE_PRECISION
#endif

#include <Mesh2SDF/MeshSDF.hpp>

#include "geometry/SDF.hpp"
#include "simobject/RigidMeshObject.hpp"
#include "config/RigidMeshObjectConfig.hpp"

namespace Geometry
{
class MeshSDF : public SDF
{
    public:
    MeshSDF(const Sim::RigidMeshObject* mesh_obj, const RigidMeshObjectConfig* config)
        : SDF(), _mesh_obj(mesh_obj)
    {
        if (config->sdfFilename().has_value())
        {
            _from_file = true;
            // load SDF from file
            _sdf = mesh2sdf::MeshSDF(config->sdfFilename().value());
        }
        else
        {
            // calculate the SDF at the mesh's un-rotated state
            // if the corresponding RigidMeshObject to this SDF has an initial rotation, the mesh is already rotated, which will throw off the SDF
            Geometry::Mesh mesh_copy = *(mesh_obj->mesh());
            // unrotate the copy of the mesh
            const Eigen::Matrix3d rot_mat = GeometryUtils::quatToMat(GeometryUtils::inverseQuat(mesh_obj->orientation()));
            mesh_copy.rotateAbout(_mesh_obj->position(), rot_mat);
            // compute the SDF
            _sdf = mesh2sdf::MeshSDF(mesh_copy.vertices(), mesh_copy.faces(), 128, 5, true);
        }
    }

    virtual double evaluate(const Eigen::Vector3d& x) const
    {
        // transform x into body coordinates
        const Eigen::Vector3d x_body = GeometryUtils::rotateVectorByQuat(x - _mesh_obj->position(), GeometryUtils::inverseQuat(_mesh_obj->orientation()));
        // SDF may not be centered about the origin
        if (_from_file)
        {
            // TODO: fix alignment between SDF coordinates and body coordinates
            const mesh2sdf::BoundingBox sdf_mesh_bbox = _sdf.meshBoundingBox();
            const Eigen::Vector3d sdf_mesh_size = sdf_mesh_bbox.second - sdf_mesh_bbox.first;
            const Eigen::Vector3d obj_mesh_size = sdf_mesh_size; //_mesh_obj->mesh()->unrotatedSize();
            const Eigen::Vector3d sdf_mesh_center = (sdf_mesh_bbox.first + sdf_mesh_bbox.second)/2;
            const Eigen::Vector3d x_body = sdf_mesh_center.array() + x_body.array() * (sdf_mesh_size.array() / obj_mesh_size.array());
        }
        return _sdf.evaluate(x_body);
    }

    virtual Eigen::Vector3d gradient(const Eigen::Vector3d& x) const
    {
        // transform x into body coordinates
        const Eigen::Vector3d x_body = GeometryUtils::rotateVectorByQuat(x - _mesh_obj->position(), GeometryUtils::inverseQuat(_mesh_obj->orientation()));
        // SDF may not be centered about the origin
        if (_from_file)
        {
            // TODO: fix alignment between SDF coordinates and body coordinates
            const mesh2sdf::BoundingBox sdf_mesh_bbox = _sdf.meshBoundingBox();
            const Eigen::Vector3d sdf_mesh_size = sdf_mesh_bbox.second - sdf_mesh_bbox.first;
            const Eigen::Vector3d obj_mesh_size = sdf_mesh_size; //_mesh_obj->mesh()->unrotatedSize();
            const Eigen::Vector3d sdf_mesh_center = (sdf_mesh_bbox.first + sdf_mesh_bbox.second)/2;
            const Eigen::Vector3d x_body = sdf_mesh_center.array() + x_body.array() * (sdf_mesh_size.array() / obj_mesh_size.array());
        }
        const Eigen::Vector3d grad = _sdf.gradient(x_body);
        return GeometryUtils::rotateVectorByQuat(grad, _mesh_obj->orientation());
    }

    protected:
    const Sim::RigidMeshObject* _mesh_obj;
    mesh2sdf::MeshSDF _sdf;
    bool _from_file;

};

} // namespace Geometry

#endif // __MESH_SDF_HPP