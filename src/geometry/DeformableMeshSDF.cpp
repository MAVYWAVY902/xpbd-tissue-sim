#include "geometry/DeformableMeshSDF.hpp"

#include "utils/GeometryUtils.hpp"

namespace Geometry
{
DeformableMeshSDF::DeformableMeshSDF(const Sim::TetMeshObject* mesh_obj, const EmbreeScene* embree_scene)
    : _mesh_obj(mesh_obj), _embree_scene(embree_scene)
{
    
}

Real DeformableMeshSDF::evaluate(const Vec3r& x) const
{
    // find enclosing tetrahedron for query point
    std::set<EmbreeHit> query_result = _embree_scene->pointInTetrahedraQuery(x, _mesh_obj);

    if (query_result.empty())
    {
        EmbreeHit hit = _embree_scene->closestPointTetMesh(x, _mesh_obj);
        return (hit.hit_point - x).norm();
    }
    
    // map query point back to undeformed configuration (Xm = F^-1 (x - v4) + v4)
    int elem_index = query_result.begin()->prim_index;
    const Mat3r F = _mesh_obj->tetMesh()->elementDeformationGradient(elem_index);
    const Eigen::Vector4i& elem = _mesh_obj->tetMesh()->element(elem_index);
    const Vec3r& v4 = _mesh_obj->tetMesh()->vertex(elem[3]);

    const Vec3r X_m = F.inverse() * (x - v4) + v4;

    // query MeshSDF with Xm
    

    // find closest point on surface

    // map closest point on surface back to deformed configuration
    

    return 0;
}

Vec3r DeformableMeshSDF::gradient(const Vec3r& x) const
{
    // TODO
    return Vec3r(0,0,0);
}

// Real DeformableMeshSDF::_evaluateStaticSDF(const Vec3r& X_m) const
// {
//     // SDF may not be centered about the origin
//     if (_from_file)
//     {
//         // TODO: fix alignment between SDF coordinates and body coordinates
//         const mesh2sdf::BoundingBox sdf_mesh_bbox = _sdf.meshBoundingBox();
//         const Vec3r sdf_mesh_size = sdf_mesh_bbox.second - sdf_mesh_bbox.first;
//         const Vec3r obj_mesh_size = _mesh_obj->mesh()->unrotatedSize();
//         const Vec3r sdf_mesh_cm = _sdf.meshMassCenter();
//         const Vec3r scaling_factors_xyz = sdf_mesh_size.array() / obj_mesh_size.array();
//         const Vec3r x_sdf = sdf_mesh_cm.array() + X_m.array() * scaling_factors_xyz.array();
//         // std::cout << "x_body: " << x_body[0] << ", " << x_body[1] << ", " << x_body[2] << std::endl;
//         // std::cout << "x_sdf: " << x_sdf[0] << ", " << x_sdf[1] << ", " << x_sdf[2] << std::endl;
//         const Real dist = _sdf.evaluate(x_sdf);
//         // std::cout << "dist: " << dist << std::endl;
//         const Vec3r grad = _sdf.gradient(x_sdf);
//         const Vec3r scaled_dist_vec =  grad.array() * dist / scaling_factors_xyz.array();
//         return scaled_dist_vec.norm() * ( (dist < 0) ? -1 : 1);
//     }
//     return _sdf.evaluate(X_m);
// }

// Vec3r DeformableMeshSDF::_gradientStaticSDF(const Vec3r& X_m) const
// {
//     Vec3r grad;
//     // SDF may not be centered about the origin
//     if (_from_file)
//     {
//         // TODO: fix alignment between SDF coordinates and body coordinates
//         const mesh2sdf::BoundingBox sdf_mesh_bbox = _sdf.meshBoundingBox();
//         const Vec3r sdf_mesh_size = sdf_mesh_bbox.second - sdf_mesh_bbox.first;
//         const Vec3r obj_mesh_size = _mesh_obj->mesh()->unrotatedSize();
//         const Vec3r sdf_mesh_cm = _sdf.meshMassCenter();
//         const Vec3r scaling_factors_xyz = sdf_mesh_size.array() / obj_mesh_size.array();
//         const Vec3r x_sdf = sdf_mesh_cm.array() + X_m.array() * scaling_factors_xyz.array();
//         grad = _sdf.gradient(x_sdf);
//     }
//     else
//     {
//         grad = _sdf.gradient(X_m);
//     }

//     return grad;
// }

} // namespace Geometry