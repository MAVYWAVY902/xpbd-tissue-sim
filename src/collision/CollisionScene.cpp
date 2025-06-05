#include "collision/CollisionScene.hpp"
#include "simulation/Simulation.hpp"
#include "simobject/RigidPrimitives.hpp"
#include "simobject/MeshObject.hpp"
#include "simobject/RigidMeshObject.hpp"
#include "simobject/XPBDMeshObject.hpp"
#include "simobject/VirtuosoArm.hpp"
#include "simobject/VirtuosoRobot.hpp"
#include "geometry/SphereSDF.hpp"
#include "geometry/BoxSDF.hpp"
#include "geometry/CylinderSDF.hpp"
#include "geometry/MeshSDF.hpp"
#include "geometry/Mesh.hpp"
#include "geometry/VirtuosoArmSDF.hpp"
#include "utils/GeometryUtils.hpp"

#ifdef HAVE_CUDA
#include "gpu/resource/GPUResource.hpp"
#include "gpu/resource/MeshGPUResource.hpp"
#include "gpu/Collision.cuh"
#endif

// namespace Collision
// {

CollisionScene::CollisionScene(const Sim::Simulation* sim, const Geometry::EmbreeScene* embree_scene)
    : _sim(sim), _embree_scene(embree_scene)
{

}

void CollisionScene::collideObjects()
{
    // collide object pairs
    _objects.for_each_element([this](auto obj1){
        _objects.for_each_element([this, obj1](auto obj2){
            _collideObjectPair(obj1, obj2);
        });
    });
}

void CollisionScene::_collideObjectPair(Sim::Object* obj1, Sim::Object* obj2)
{
    // do nothing in the general case
}

void CollisionScene::_collideObjectPair(Sim::VirtuosoArm* virtuoso_arm, Sim::XPBDMeshObject_Base* xpbd_mesh_obj)
{
    const typename Sim::XPBDMeshObject_Base::SDFType* sdf = xpbd_mesh_obj->SDF();
    const Vec3r& tip_pos = virtuoso_arm->tipPosition();
    const Real dist = sdf->evaluate(tip_pos);
    
    std::cout << "DeformableSDF distance: " << dist << std::endl;
}

void CollisionScene::_collideObjectPair(Sim::XPBDMeshObject_Base* xpbd_mesh_obj1, Sim::XPBDMeshObject_Base* xpbd_mesh_obj2)
{
    // deformable-deformable collision not supported for now
}

void CollisionScene::_collideObjectPair(Sim::XPBDMeshObject_Base* xpbd_mesh_obj, Sim::VirtuosoArm* virtuoso_arm)
{
    // iterate through faces of mesh
    const typename Sim::VirtuosoArm::SDFType* sdf = virtuoso_arm->SDF();
    const Geometry::Mesh* mesh = xpbd_mesh_obj->mesh();
    const Geometry::Mesh::FacesMat& faces = mesh->faces();
    for (int i = 0; i < faces.cols(); i++)
    {
        const Eigen::Vector3i& f = faces.col(i);
        const Vec3r& p1 = mesh->vertex(f[0]);
        const Vec3r& p2 = mesh->vertex(f[1]);
        const Vec3r& p3 = mesh->vertex(f[2]);

        const Real p1p2 = (p2-p1).norm();
        const Real p1p3 = (p3-p1).norm();
        const Real p2p3 = (p3-p2).norm();

        const Real max_edge = std::max({p1p2, p1p3, p2p3});
        const int num_samples = (int)(5*max_edge / 1e-3);

        // const int num_samples = 4;
        for (int si = 0; si <= num_samples; si++)
        {
            for (int sj = 0; sj <= num_samples - si; sj++)
            {
                const Real u = (Real)(si+1) / (num_samples+2);
                const Real v = (Real)(sj+1) / (num_samples+2);
                const Real w = 1 - u - v;
                const Vec3r x = u*p1 + v*p2 + w*p3;
                const Real distance = sdf->evaluate(x);
                if (distance <= 1e-3)
                {// collision occurred, find barycentric coordinates (u,v,w) of x on triangle face
                    // from https://ceng2.ktu.edu.tr/~cakir/files/grafikler/Texture_Mapping.pdf
                    const auto [u, v, w] = GeometryUtils::barycentricCoords(x, p1, p2, p3);
                    const Vec3r grad = sdf->gradient(x);
                    const Vec3r surface_x = x - grad*distance;
                    // std::cout << "COLLISION!" << std::endl;
                    // std::cout << "u: " << u << ", v: " << v << ", w: " << w << std::endl;
                    // std::cout << "position: " << x[0] << ", " << x[1] << ", " << x[2] << "\tnormal: " << grad[0] << ", " << grad[1] << ", " << grad[2] << std::endl;
                    // std::cout << "surface position: " << surface_x[0] << ", " << surface_x[1] << ", " << surface_x[2] << std::endl;
                
                    xpbd_mesh_obj->addStaticCollisionConstraint(sdf, surface_x, grad, i, u, v, w);
                    
                }
            }
        }
    }
}

void CollisionScene::_collideObjectPair(Sim::XPBDMeshObject_Base* xpbd_mesh_obj, Sim::RigidObject* rigid_obj)
{
    // iterate through faces of mesh
    const Geometry::SDF* sdf = rigid_obj->SDF();
    const Geometry::Mesh* mesh = xpbd_mesh_obj->mesh();
    const Geometry::Mesh::FacesMat& faces = mesh->faces();
    for (int i = 0; i < faces.cols(); i++)
    {
        const Eigen::Vector3i& f = faces.col(i);
        const Vec3r& p1 = mesh->vertex(f[0]);
        const Vec3r& p2 = mesh->vertex(f[1]);
        const Vec3r& p3 = mesh->vertex(f[2]);
        const Vec3r x = _frankWolfe(sdf, p1, p2, p3);
        const double distance = sdf->evaluate(x);
        if (distance <= 1e-4)
        {// collision occurred, find barycentric coordinates (u,v,w) of x on triangle face
            // from https://ceng2.ktu.edu.tr/~cakir/files/grafikler/Texture_Mapping.pdf
            const auto [u, v, w] = GeometryUtils::barycentricCoords(x, p1, p2, p3);
            const Vec3r grad = sdf->gradient(x);
            const Vec3r surface_x = x - grad*distance;
            
            if (rigid_obj->isFixed())
            {
                xpbd_mesh_obj->addStaticCollisionConstraint(sdf, surface_x, grad, i, u, v, w);
            }
            else
            {
                xpbd_mesh_obj->addRigidDeformableCollisionConstraint(sdf, rigid_obj, surface_x, grad, i, u, v, w);
            }
            
        }
    }
}

void CollisionScene::_collideObjectPair(Sim::XPBDMeshObject_Base* xpbd_mesh_obj, Sim::Object* obj2)
{
    // iterate through faces of mesh
    const Geometry::SDF* sdf = obj2->SDF();
    const Geometry::Mesh* mesh = xpbd_mesh_obj->mesh();
    const Geometry::Mesh::FacesMat& faces = mesh->faces();
    for (int i = 0; i < faces.cols(); i++)
    {
        const Eigen::Vector3i& f = faces.col(i);
        const Vec3r& p1 = mesh->vertex(f[0]);
        const Vec3r& p2 = mesh->vertex(f[1]);
        const Vec3r& p3 = mesh->vertex(f[2]);
        const Vec3r x = _frankWolfe(sdf, p1, p2, p3);
        const double distance = sdf->evaluate(x);
        if (distance <= 1e-4)
        {// collision occurred, find barycentric coordinates (u,v,w) of x on triangle face
            // from https://ceng2.ktu.edu.tr/~cakir/files/grafikler/Texture_Mapping.pdf
            const auto [u, v, w] = GeometryUtils::barycentricCoords(x, p1, p2, p3);
            const Vec3r grad = sdf->gradient(x);
            const Vec3r surface_x = x - grad*distance;
            xpbd_mesh_obj->addStaticCollisionConstraint(sdf, surface_x, grad, i, u, v, w);
            
        }
    }
}

Vec3r CollisionScene::_frankWolfe(const Geometry::SDF* sdf, const Vec3r& p1, const Vec3r& p2, const Vec3r& p3) const
{
    // find starting iterate - the triangle vertex with the smallest value of SDF
    const Real d_p1 = sdf->evaluate(p1);
    const Real d_p2 = sdf->evaluate(p2);
    const Real d_p3 = sdf->evaluate(p3);

    Vec3r x;
    if (d_p1 <= d_p2 && d_p1 <= d_p3)       x = p1;
    else if (d_p2 <= d_p1 && d_p2 <= d_p3)  x = p2;
    else                                    x = p3;

    Vec3r s;
    for (int i = 0; i < 32; i++)
    {
        const Real alpha = 2.0/(i+3);
        const Vec3r& gradient = sdf->gradient(x);
        const Real sg1 = p1.dot(gradient);
        const Real sg2 = p2.dot(gradient);
        const Real sg3 = p3.dot(gradient);

        if (sg1 < sg2 && sg1 < sg3)       s = p1;
        else if (sg2 < sg1 && sg2 < sg3)  s = p2;
        else                                s = p3;

        x = x + alpha * (s - x);
        
    }

    return x;
}

// } // namespace Collision