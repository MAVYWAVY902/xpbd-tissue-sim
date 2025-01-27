#include "collision/CollisionScene.hpp"
#include "simulation/Simulation.hpp"
#include "simobject/RigidPrimitives.hpp"
#include "simobject/MeshObject.hpp"
#include "simobject/XPBDMeshObject.hpp"
#include "simobject/VirtuosoArm.hpp"
#include "geometry/SphereSDF.hpp"
#include "geometry/BoxSDF.hpp"
#include "geometry/CylinderSDF.hpp"
#include "geometry/MeshSDF.hpp"
#include "geometry/Mesh.hpp"
#include "geometry/VirtuosoArmSDF.hpp"
#include "utils/GeometryUtils.hpp"

// namespace Collision
// {

CollisionScene::CollisionScene(const Sim::Simulation* sim)
    : _sim(sim)
{

}

void CollisionScene::addObject(Sim::Object* new_obj, const ObjectConfig* config)
{
    // create a SDF for the new object, if applicable
    std::unique_ptr<Geometry::SDF> sdf = nullptr;
    if (Sim::RigidSphere* sphere = dynamic_cast<Sim::RigidSphere*>(new_obj))
    {
        sdf = std::make_unique<Geometry::SphereSDF>(sphere);
    }
    else if (Sim::RigidBox* box = dynamic_cast<Sim::RigidBox*>(new_obj))
    {
        sdf = std::make_unique<Geometry::BoxSDF>(box);
    }
    else if (Sim::RigidCylinder* cyl = dynamic_cast<Sim::RigidCylinder*>(new_obj))
    {
        sdf = std::make_unique<Geometry::CylinderSDF>(cyl);
    }
    else if (Sim::RigidMeshObject* mesh_obj = dynamic_cast<Sim::RigidMeshObject*>(new_obj))
    {
        const RigidMeshObjectConfig* obj_config = dynamic_cast<const RigidMeshObjectConfig*>(config);
        assert(obj_config);
        sdf = std::make_unique<Geometry::MeshSDF>(mesh_obj, obj_config);
    }
    else if (Sim::VirtuosoArm* arm = dynamic_cast<Sim::VirtuosoArm*>(new_obj))
    {
        sdf = std::make_unique<Geometry::VirtuosoArmSDF>(arm);
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
    // std::cout << "Colliding " << c_obj1.obj->name() << " and " << c_obj2.obj->name() << std::endl;
    const Geometry::SDF* sdf;
    const Geometry::Mesh* mesh;
    Sim::XPBDMeshObject* xpbd_obj;
    Sim::RigidObject* rigid_obj;
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
        rigid_obj = dynamic_cast<Sim::RigidObject*>(c_obj1.obj);
        assert(xpbd_obj);
    } 
    else
    {
        sdf = c_obj2.sdf.get();

        Sim::MeshObject* mesh_obj = dynamic_cast<Sim::MeshObject*>(c_obj1.obj);
        assert(mesh_obj);
        mesh = mesh_obj->mesh();
        xpbd_obj = dynamic_cast<Sim::XPBDMeshObject*>(mesh_obj);
        rigid_obj = dynamic_cast<Sim::RigidObject*>(c_obj2.obj);
        assert(xpbd_obj);
    }

    // iterate through faces of mesh
    const Geometry::Mesh::FacesMat& faces = mesh->faces();
    for (const auto& f : faces.colwise())
    {
        const Eigen::Vector3d& p1 = mesh->vertex(f[0]);
        const Eigen::Vector3d& p2 = mesh->vertex(f[1]);
        const Eigen::Vector3d& p3 = mesh->vertex(f[2]);
        // std::cout << "v1: " << p1[0] << ", " << p1[1] << ", " << p1[2] << "\tdist: " << sdf->evaluate(p1) << std::endl;
        // std::cout << "v2: " << p2[0] << ", " << p2[1] << ", " << p2[2] << "\tdist: " << sdf->evaluate(p2) << std::endl;
        // std::cout << "v3: " << p3[0] << ", " << p3[1] << ", " << p3[2] << "\tdist: " << sdf->evaluate(p3) << std::endl;
        const Eigen::Vector3d x = _frankWolfe(sdf, p1, p2, p3);
        const double distance = sdf->evaluate(x);
        if (distance <= 1e-4)
        {// collision occurred, find barycentric coordinates (u,v,w) of x on triangle face
            // from https://ceng2.ktu.edu.tr/~cakir/files/grafikler/Texture_Mapping.pdf
            const auto [u, v, w] = GeometryUtils::barycentricCoords(x, p1, p2, p3);
            const Eigen::Vector3d grad = sdf->gradient(x);
            const Eigen::Vector3d surface_x = x - grad*distance;
            std::cout << "COLLISION!" << " distance: " << distance << std::endl;
            std::cout << "u: " << u << ", v: " << v << ", w: " << w << std::endl;
            std::cout << "position: " << x[0] << ", " << x[1] << ", " << x[2] << "\tnormal: " << grad[0] << ", " << grad[1] << ", " << grad[2] << std::endl;
            std::cout << "surface position: " << surface_x[0] << ", " << surface_x[1] << ", " << surface_x[2] << std::endl;
        
            if (!rigid_obj)
            {
                xpbd_obj->addStaticCollisionConstraint(sdf, surface_x, grad, xpbd_obj, f[0], f[1], f[2], u, v, w);
            }
            else if (rigid_obj->isFixed())
            {
                xpbd_obj->addStaticCollisionConstraint(sdf, surface_x, grad, xpbd_obj, f[0], f[1], f[2], u, v, w);
            }
            else
            {
                xpbd_obj->addRigidDeformableCollisionConstraint(sdf, rigid_obj, surface_x, grad, xpbd_obj, f[0], f[1], f[2], u, v, w);
            }
            
        }

        // TODO: check each vertex in the mesh separately instead of inside the faces loop
        const double distance_p1 = sdf->evaluate(p1);
        if (distance_p1 <= 1e-4)
        {
            const auto [u, v, w] = GeometryUtils::barycentricCoords(p1, p1, p2, p3);
            const Eigen::Vector3d grad = sdf->gradient(p1);
            const Eigen::Vector3d surface_x = p1 - grad*distance;

            if (!rigid_obj)
            {
                xpbd_obj->addStaticCollisionConstraint(sdf, surface_x, grad, xpbd_obj, f[0], f[1], f[2], u, v, w);
            }
            else if (rigid_obj->isFixed())
            {
                xpbd_obj->addStaticCollisionConstraint(sdf, surface_x, grad, xpbd_obj, f[0], f[1], f[2], u, v, w);
            }
            else
            {
                xpbd_obj->addRigidDeformableCollisionConstraint(sdf, rigid_obj, surface_x, grad, xpbd_obj, f[0], f[1], f[2], u, v, w);
            }
        }

        const double distance_p2 = sdf->evaluate(p2);
        if (distance_p2 <= 1e-4)
        {
            const auto [u, v, w] = GeometryUtils::barycentricCoords(p2, p1, p2, p3);
            const Eigen::Vector3d grad = sdf->gradient(p2);
            const Eigen::Vector3d surface_x = p2 - grad*distance;

            if (!rigid_obj)
            {
                xpbd_obj->addStaticCollisionConstraint(sdf, surface_x, grad, xpbd_obj, f[0], f[1], f[2], u, v, w);
            }
            else if (rigid_obj->isFixed())
            {
                xpbd_obj->addStaticCollisionConstraint(sdf, surface_x, grad, xpbd_obj, f[0], f[1], f[2], u, v, w);
            }
            else
            {
                xpbd_obj->addRigidDeformableCollisionConstraint(sdf, rigid_obj, surface_x, grad, xpbd_obj, f[0], f[1], f[2], u, v, w);
            }
        }

        const double distance_p3 = sdf->evaluate(p3);
        if (distance_p3 <= 1e-4)
        {
            const auto [u, v, w] = GeometryUtils::barycentricCoords(p3, p1, p2, p3);
            const Eigen::Vector3d grad = sdf->gradient(p3);
            const Eigen::Vector3d surface_x = p3 - grad*distance;

            if (!rigid_obj)
            {
                xpbd_obj->addStaticCollisionConstraint(sdf, surface_x, grad, xpbd_obj, f[0], f[1], f[2], u, v, w);
            }
            else if (rigid_obj->isFixed())
            {
                xpbd_obj->addStaticCollisionConstraint(sdf, surface_x, grad, xpbd_obj, f[0], f[1], f[2], u, v, w);
            }
            else
            {
                xpbd_obj->addRigidDeformableCollisionConstraint(sdf, rigid_obj, surface_x, grad, xpbd_obj, f[0], f[1], f[2], u, v, w);
            }
        }
    }


}

Eigen::Vector3d CollisionScene::_frankWolfe(const Geometry::SDF* sdf, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3) const
{
    // find starting iterate - the triangle vertex with the smallest value of SDF
    const double d_p1 = sdf->evaluate(p1);
    const double d_p2 = sdf->evaluate(p2);
    const double d_p3 = sdf->evaluate(p3);

    // std::cout << "p1: " << p1[0] << ", " << p1[1] << ", " << p1[2] << std::endl;
    // std::cout << "p2: " << p2[0] << ", " << p2[1] << ", " << p2[2] << std::endl;
    // std::cout << "p3: " << p3[0] << ", " << p3[1] << ", " << p3[2] << std::endl;

    // special case: face-face collision, edge-face collision - not sure if this is needed
    // bool e1 = std::abs(d_p1 - d_p2) < 1e-8;
    // bool e2 = std::abs(d_p2 - d_p3) < 1e-8;
    // bool e3 = std::abs(d_p3 - d_p1) < 1e-8;
    // if (e1 && e2)
    // {
    //     return 0.3333333*p1 + 0.3333333*p2 + 0.3333333*p3;
    // }
    // else if (e1)
    // {
    //     return 0.5*p1 + 0.5*p2;
    // }
    // else if (e2)
    // {
    //     return 0.5*p2 + 0.5*p3;
    // }
    // else if (e3)
    // {
    //     return 0.5*p1 + 0.5*p3;
    // }

    Eigen::Vector3d x;
    if (d_p1 <= d_p2 && d_p1 <= d_p3)       x = p1;
    else if (d_p2 <= d_p1 && d_p2 <= d_p3)  x = p2;
    else                                    x = p3;

    Eigen::Vector3d s;
    for (int i = 0; i < 32; i++)
    {
        const double alpha = 2.0/(i+3);
        const Eigen::Vector3d& gradient = sdf->gradient(x);
        // std::cout << "x: " << x[0] << ", " << x[1] << ", " << x[2] << std::endl;
        // std::cout << "gradient: " << gradient[0] << ", " << gradient[1] << ", " << gradient[2] << std::endl;
        const double sg1 = p1.dot(gradient);
        const double sg2 = p2.dot(gradient);
        const double sg3 = p3.dot(gradient);

        if (sg1 < sg2 && sg1 < sg3)       s = p1;
        else if (sg2 < sg1 && sg2 < sg3)  s = p2;
        else                                s = p3;

        x = x + alpha * (s - x);
        
    }

    return x;
}

// } // namespace Collision