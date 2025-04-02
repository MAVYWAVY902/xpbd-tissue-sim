#ifndef __COLLISION_SCENE_HPP
#define __COLLISION_SCENE_HPP

// #include "simobject/MeshObject.hpp"
#include "simobject/Object.hpp"
#include "geometry/SDF.hpp"

namespace Sim
{
    class Simulation;
}

/** Includes a reference to the object itself and its collision geometry. */
struct CollisionObject
{
    Sim::Object* obj;
    std::unique_ptr<Geometry::SDF> sdf;
};

/** Responsible for determining collisions between objects in the simulation.
 * When a collision is identified between two objects, a collision constraint is created to resolve the collision over the subsequent time steps.
 * When the Simulation adds an object to the CollisionScene, a collision geometry (i.e. a SDF) is created for that object.
 */
class CollisionScene
{
    public:
    /** Constructor - needs a reference back to the simulation to access the time step, current sim time, etc. */
    explicit CollisionScene(const Sim::Simulation* sim);

    /** Adds a new object to the CollisionScene.
     * Creates a new collision geometry for this object depending on the type. (i.e. for a RigidBox, a BoxSDF is created, for a RigidSphere, a SphereSDF, etc.)
     * 
     * @param new_obj - the pointer to the new simulation object to be added to the scene
    */
    void addObject(Sim::Object* new_obj, const ObjectConfig* config = nullptr);

    /** Detects collisions between objects in the CollisionScene.
     * When collisions are detected, collision constraints are created and added to the appropriate objects to resolve collisions.
     */
    void collideObjects();

    protected:
    /** Helper function that checks for collision between a pair of objects.
     * Implements the per face optimization described in Macklin et. al 2020 to find contact points between a SDF and a mesh.
     */
    void _collideObjectPair(CollisionObject& c_obj1, CollisionObject& c_obj2);

    /** Implements the Frank-Wolfe optimization algorithm applied to finding a contact point between a SDF and a 3D triangle face. 
     * @param sdf - the signed distance function (SDF) to collide against
     * @param p1 - 1st triangle vertex
     * @param p2 - 2nd triangle vertex
     * @param p3 - 3rd tringle vertex
     * @returns the closest point on the triangle to the boundary of the SDF - if at this closest point the SDF evaluates to negative, we have a collision!
    */
    Eigen::Vector3d _frankWolfe(const Geometry::SDF* sdf, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3) const;

    protected:
    /** Non-owning pointer to the Simulation object that this CollisionScene belongs to */
    const Sim::Simulation* _sim;

    /** Stores the mesh objects in the scene. */
    std::vector<CollisionObject> _collision_objects;


};

#endif // __COLLISION_SCENE_HPP