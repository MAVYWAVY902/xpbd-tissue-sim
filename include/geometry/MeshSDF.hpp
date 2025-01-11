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
            // calculate the SDF at the mesh's un-transformed state
            // if the corresponding RigidMeshObject to this SDF has an initial rotation, the mesh is already rotated, which will throw off the SDF
            Geometry::Mesh mesh_copy = *(mesh_obj->mesh());
            // untranslate the copy of the mesh
            mesh_copy.moveTogether(-mesh_obj->position());
            // unrotate the copy of the mesh
            const Eigen::Matrix3d rot_mat = GeometryUtils::quatToMat(GeometryUtils::inverseQuat(mesh_obj->orientation()));
            mesh_copy.rotateAbout(Eigen::Vector3d::Zero(), rot_mat);
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
            const Eigen::Vector3d obj_mesh_size = _mesh_obj->mesh()->unrotatedSize();
            const Eigen::Vector3d sdf_mesh_cm = _sdf.meshMassCenter();
            const Eigen::Vector3d scaling_factors_xyz = sdf_mesh_size.array() / obj_mesh_size.array();
            const Eigen::Vector3d x_sdf = sdf_mesh_cm.array() + x_body.array() * scaling_factors_xyz.array();
            // std::cout << "x_body: " << x_body[0] << ", " << x_body[1] << ", " << x_body[2] << std::endl;
            // std::cout << "x_sdf: " << x_sdf[0] << ", " << x_sdf[1] << ", " << x_sdf[2] << std::endl;
            const double dist = _sdf.evaluate(x_sdf);
            // std::cout << "dist: " << dist << std::endl;
            const Eigen::Vector3d grad = _sdf.gradient(x_sdf);
            const Eigen::Vector3d scaled_dist_vec =  grad.array() * dist / scaling_factors_xyz.array();
            return scaled_dist_vec.norm() * ( (dist < 0) ? -1 : 1);
        }
        return _sdf.evaluate(x_body);
    }

    virtual Eigen::Vector3d gradient(const Eigen::Vector3d& x) const
    {
        // transform x into body coordinates
        const Eigen::Vector3d x_body = GeometryUtils::rotateVectorByQuat(x - _mesh_obj->position(), GeometryUtils::inverseQuat(_mesh_obj->orientation()));
        Eigen::Vector3d grad;
        // SDF may not be centered about the origin
        if (_from_file)
        {
            // TODO: fix alignment between SDF coordinates and body coordinates
            const mesh2sdf::BoundingBox sdf_mesh_bbox = _sdf.meshBoundingBox();
            const Eigen::Vector3d sdf_mesh_size = sdf_mesh_bbox.second - sdf_mesh_bbox.first;
            const Eigen::Vector3d obj_mesh_size = _mesh_obj->mesh()->unrotatedSize();
            const Eigen::Vector3d sdf_mesh_cm = _sdf.meshMassCenter();
            const Eigen::Vector3d scaling_factors_xyz = sdf_mesh_size.array() / obj_mesh_size.array();
            const Eigen::Vector3d x_sdf = sdf_mesh_cm.array() + x_body.array() * scaling_factors_xyz.array();
            grad = _sdf.gradient(x_sdf);
        }
        else
        {
            grad = _sdf.gradient(x_body);
        }
        return GeometryUtils::rotateVectorByQuat(grad, _mesh_obj->orientation());
    }

    protected:
    const Sim::RigidMeshObject* _mesh_obj;
    mesh2sdf::MeshSDF _sdf;
    bool _from_file;

};

} // namespace Geometry

#endif // __MESH_SDF_HPP