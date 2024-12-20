#ifndef __COLLISION_SCENE_HPP
#define __COLLISION_SCENE_HPP

// #include "simobject/MeshObject.hpp"
#include "simobject/Object.hpp"
#include "geometry/SDF.hpp"

namespace Sim
{
    class Simulation;
}

/** Represents a collision between a vertex and a face */
struct Collision
{
    Sim::Object* obj1;
    int vertex_ind;
    Sim::Object* obj2;
    int face_ind;
};

struct CollisionObject
{
    Sim::Object* obj;
    std::unique_ptr<Geometry::SDF> sdf;
};

class CollisionScene
{
    public:
    explicit CollisionScene(const Sim::Simulation* sim);

    void addObject(Sim::Object* new_obj);

    void collideObjects();

    protected:
    void _collideObjectPair(CollisionObject& c_obj1, CollisionObject& c_obj2);
    Eigen::Vector3d _frankWolfe(const Geometry::SDF* sdf, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3) const;

    const Sim::Simulation* _sim;

    /** Stores the mesh objects in the scene. */
    std::vector<CollisionObject> _collision_objects;


};

#endif // __COLLISION_SCENE_HPP