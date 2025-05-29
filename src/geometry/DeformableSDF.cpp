#include "geometry/DeformableSDF.hpp"

namespace Geometry
{

DeformableSDF::DeformableSDF(const Sim::TetMeshObject* obj, const EmbreeScene* embree_scene)
    : _obj(obj), _embree_scene(embree_scene)
{
    if (config && config->sdfFilename().has_value())
    {
        _from_file = true;
        // load SDF from file
        _sdf = mesh2sdf::MeshSDF(config->sdfFilename().value());
    }
    else
    {
        // calculate the SDF at the mesh's un-transformed state
        // if the corresponding TetMeshObject to this SDF has an initial rotation, the mesh is already rotated, which will throw off the SDF
        Geometry::Mesh mesh_copy(*(mesh_obj->mesh()));
        // untranslate the copy of the mesh
        mesh_copy.moveTogether(-mesh_obj->position());
        // unrotate the copy of the mesh
        const Mat3r rot_mat = GeometryUtils::quatToMat(GeometryUtils::inverseQuat(mesh_obj->orientation()));
        mesh_copy.rotateAbout(Vec3r::Zero(), rot_mat);
        // compute the SDF
        _sdf = mesh2sdf::MeshSDF(mesh_copy.vertices(), mesh_copy.faces(), 128, 5, true);
    }
}

Real DeformableSDF::evaluate(const Vec3r& x) const
{
    // find enclosing tetrahedron for query point
    std::set<EmbreeHit> query_result = _embree_scene->pointInTetrahedraQuery(x, _obj);

    if (query_result.empty())
    {
        EmbreeHit hit = _embree_scene->closestPointTetMesh(x, _obj);
        return (hit.hit_point - x).norm();
    }
    
    // map query point back to undeformed configuration (Xm = F^-1 (x - v4) + v4)
    int elem_index = query_result.begin()->prim_index;
    const Mat3r F = _obj->tetMesh()->elementDeformationGradient(elem_index);
    const Eigen::Vector4i& elem = _obj->tetMesh()->element(elem_index);
    const Vec3r& v4 = _obj->tetMesh()->vertex(elem[3]);

    const Vec3r X_m = F.inverse() * (x - v4) + v4;

    // query MeshSDF with Xm
    

    // find closest point on surface

    // map closest point on surface back to deformed configuration
    

    return 0;
}

Vec3r DeformableSDF::gradient(const Vec3r& x) const
{
    // TODO
    return Vec3r(0,0,0);
}

Real DeformableSDF::_evaluateStaticSDF(const Vec3r& X_m) const
{
    // SDF may not be centered about the origin
    if (_from_file)
    {
        // TODO: fix alignment between SDF coordinates and body coordinates
        const mesh2sdf::BoundingBox sdf_mesh_bbox = _sdf.meshBoundingBox();
        const Vec3r sdf_mesh_size = sdf_mesh_bbox.second - sdf_mesh_bbox.first;
        const Vec3r obj_mesh_size = _mesh_obj->mesh()->unrotatedSize();
        const Vec3r sdf_mesh_cm = _sdf.meshMassCenter();
        const Vec3r scaling_factors_xyz = sdf_mesh_size.array() / obj_mesh_size.array();
        const Vec3r x_sdf = sdf_mesh_cm.array() + X_m.array() * scaling_factors_xyz.array();
        // std::cout << "x_body: " << x_body[0] << ", " << x_body[1] << ", " << x_body[2] << std::endl;
        // std::cout << "x_sdf: " << x_sdf[0] << ", " << x_sdf[1] << ", " << x_sdf[2] << std::endl;
        const Real dist = _sdf.evaluate(x_sdf);
        // std::cout << "dist: " << dist << std::endl;
        const Vec3r grad = _sdf.gradient(x_sdf);
        const Vec3r scaled_dist_vec =  grad.array() * dist / scaling_factors_xyz.array();
        return scaled_dist_vec.norm() * ( (dist < 0) ? -1 : 1);
    }
    return _sdf.evaluate(X_m);
}

Vec3r DeformableSDF::_gradientStaticSDF(const Vec3r& X_m) const
{
    Vec3r grad;
    // SDF may not be centered about the origin
    if (_from_file)
    {
        // TODO: fix alignment between SDF coordinates and body coordinates
        const mesh2sdf::BoundingBox sdf_mesh_bbox = _sdf.meshBoundingBox();
        const Vec3r sdf_mesh_size = sdf_mesh_bbox.second - sdf_mesh_bbox.first;
        const Vec3r obj_mesh_size = _mesh_obj->mesh()->unrotatedSize();
        const Vec3r sdf_mesh_cm = _sdf.meshMassCenter();
        const Vec3r scaling_factors_xyz = sdf_mesh_size.array() / obj_mesh_size.array();
        const Vec3r x_sdf = sdf_mesh_cm.array() + x_body.array() * scaling_factors_xyz.array();
        grad = _sdf.gradient(x_sdf);
    }
    else
    {
        grad = _sdf.gradient(x_body);
    }

    return grad;
}

} // namespace Geometry