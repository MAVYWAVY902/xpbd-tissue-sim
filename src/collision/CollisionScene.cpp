#include "collision/CollisionScene.hpp"
#include "simulation/Simulation.hpp"
#include "simobject/RigidPrimitives.hpp"
#include "simobject/MeshObject.hpp"
#include "simobject/XPBDMeshObject.hpp"
#include "geometry/SphereSDF.hpp"
#include "geometry/Mesh.hpp"
#include "utils/GeometryUtils.hpp"

// namespace Collision
// {

CollisionScene::CollisionScene(const Sim::Simulation* sim)
    : _sim(sim)
{

}

void CollisionScene::addObject(Sim::Object* new_obj)
{
    // create a SDF for the new object, if applicable
    std::unique_ptr<Geometry::SDF> sdf = nullptr;
    if (Sim::RigidSphere* sphere = dynamic_cast<Sim::RigidSphere*>(new_obj))
    {
        sdf = std::make_unique<Geometry::SphereSDF>(sphere);
    }

    // create a new collision object
    CollisionObject c_obj;
    c_obj.obj = new_obj;
    c_obj.sdf = std::move(sdf);

    _collision_objects.push_back(std::move(c_obj));
}

void CollisionScene::collideObjects()
{
    // collide object pairs
    for (size_t i = 0; i < _collision_objects.size(); i++)
    {
        for (size_t j = i+1; j < _collision_objects.size(); j++)
        {
            _collideObjectPair(_collision_objects[i], _collision_objects[j]);
        }
    }
}

void CollisionScene::_collideObjectPair(CollisionObject& c_obj1, CollisionObject& c_obj2)
{
    const Geometry::SDF* sdf;
    const Geometry::Mesh* mesh;
    Sim::XPBDMeshObject* xpbd_obj;
    if (c_obj1.sdf && c_obj2.sdf)
    {
        std::cout << "Can't collide 2 SDFs (yet)!" << std::endl;
        return;
    }

    if (!c_obj1.sdf && !c_obj2.sdf)
    {
        std::cout << "At least one object must have an SDF!" << std::endl;
        return;
    }

    if (c_obj1.sdf)
    {
        sdf = c_obj1.sdf.get();

        Sim::MeshObject* mesh_obj = dynamic_cast<Sim::MeshObject*>(c_obj2.obj);
        assert(mesh_obj);
        mesh = mesh_obj->mesh();
        xpbd_obj = dynamic_cast<Sim::XPBDMeshObject*>(mesh_obj);
        assert(xpbd_obj);
    } 
    else
    {
        sdf = c_obj2.sdf.get();

        Sim::MeshObject* mesh_obj = dynamic_cast<Sim::MeshObject*>(c_obj1.obj);
        assert(mesh_obj);
        mesh = mesh_obj->mesh();
        xpbd_obj = dynamic_cast<Sim::XPBDMeshObject*>(mesh_obj);
        assert(xpbd_obj);
    }

    // iterate through faces of mesh
    const Geometry::Mesh::FacesMat faces = mesh->faces();
    for (const auto& f : faces.colwise())
    {
        const Eigen::Vector3d& p1 = mesh->vertex(f[0]);
        const Eigen::Vector3d& p2 = mesh->vertex(f[1]);
        const Eigen::Vector3d& p3 = mesh->vertex(f[2]);
        const Eigen::Vector3d x = _frankWolfe(sdf, p1, p2, p3);
        const double distance = sdf->evaluate(x);
        if (distance < 1e-4)
        {// collision occurred, find barycentric coordinates (u,v,w) of x on triangle face
            // from https://ceng2.ktu.edu.tr/~cakir/files/grafikler/Texture_Mapping.pdf
            const auto [u, v, w] = GeometryUtils::barycentricCoords(x, p1, p2, p3);
            const Eigen::Vector3d grad = sdf->gradient(x);
            const Eigen::Vector3d surface_x = x - grad*distance;
            std::cout << "COLLISION!" << std::endl;
            std::cout << "u: " << u << ", v: " << v << ", w: " << w << std::endl;
            std::cout << "position: " << x[0] << ", " << x[1] << ", " << x[2] << "\tnormal: " << grad[0] << ", " << grad[1] << ", " << grad[2] << std::endl;
            std::cout << "surface position: " << surface_x[0] << ", " << surface_x[1] << ", " << surface_x[2] << std::endl;
        
            xpbd_obj->addStaticCollisionConstraint(sdf, surface_x, grad, xpbd_obj, f[0], f[1], f[2], u, v, w);
        }
    }


}

Eigen::Vector3d CollisionScene::_frankWolfe(const Geometry::SDF* sdf, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3) const
{
    // find starting iterate - the triangle vertex with the smallest value of SDF
    const double d_p1 = sdf->evaluate(p1);
    const double d_p2 = sdf->evaluate(p2);
    const double d_p3 = sdf->evaluate(p3);
    Eigen::Vector3d x;
    if (d_p1 <= d_p2 && d_p1 <= d_p3)       x = p1;
    else if (d_p2 <= d_p1 && d_p2 <= d_p3)  x = p2;
    else                                    x = p3;

    Eigen::Vector3d s;
    for (int i = 0; i < 32; i++)
    {
        const double alpha = 2.0/(i+3);
        const Eigen::Vector3d& gradient = sdf->gradient(x);
        const double sg1 = p1.dot(gradient);
        const double sg2 = p2.dot(gradient);
        const double sg3 = p3.dot(gradient);

        if (sg1 < sg2 && sg1 < sg3)       s = p1;
        else if (sg2 < sg1 && sg2 < sg3)  s = p2;
        else                                s = p3;

        x = x + alpha * (s - x);
        // std::cout << "x: " << x[0] << ", " << x[1] << ", " << x[2] << std::endl;
    }

    return x;
}

// } // namespace Collision