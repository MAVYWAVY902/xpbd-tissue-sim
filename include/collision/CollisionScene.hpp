#ifndef __COLLISION_SCENE_HPP
#define __COLLISION_SCENE_HPP

// #include "simobject/MeshObject.hpp"
#include "simobject/Object.hpp"
#include "simobject/FirstOrderXPBDMeshObject.hpp"
#include "simobject/XPBDMeshObject.hpp"
#include "simobject/RigidMeshObject.hpp"
#include "simobject/RigidObject.hpp"
#include "simobject/RigidPrimitives.hpp"
#include "simobject/VirtuosoArm.hpp"
#include "simobject/VirtuosoRobot.hpp"

#include "geometry/embree/EmbreeScene.hpp"

#include "common/VariadicVectorContainer.hpp"
#include "common/SimulationTypeDefs.hpp"

#ifdef HAVE_CUDA
#include "gpu/resource/WritableArrayGPUResource.hpp"
#include "gpu/GPUStructs.hpp"
#endif

namespace Sim
{
    class Simulation;
}

/** Responsible for determining collisions between objects in the simulation.
 * When a collision is identified between two objects, a collision constraint is created to resolve the collision over the subsequent time steps.
 * When the Simulation adds an object to the CollisionScene, a collision geometry (i.e. a SDF) is created for that object.
 */
class CollisionScene
{
    public:
    using ObjectVectorType = VariadicVectorContainerFromTypeList<SimulationObjectTypes>::ptr_type;

    public:
    /** Constructor - needs a reference back to the simulation to access the time step, current sim time, etc. */
    explicit CollisionScene(const Sim::Simulation* sim, const Geometry::EmbreeScene* embree_scene);

    /** Adds a new object to the CollisionScene.
     * Creates a SDF for the object and adds the object's pointer to the vector of objects in the CollisionScene.
     * 
     * @param obj - the pointer to the new simulation object to be added to the scene
    */
    template <typename ObjectType>
    void addObject(ObjectType* obj)
    {
        obj->createSDF();
        
#ifdef HAVE_CUDA
        const typename ObjectType::SDFType* sdf = obj->SDF();
        if (sdf)
        {
            sdf->createGPUResource();
            sdf->gpuResource()->fullCopyToDevice();
        }

        if (Sim::XPBDMeshObject_Base* mesh_obj = dynamic_cast<Sim::XPBDMeshObject_Base*>(obj))
        {
            // create a managed resource for the mesh
            Geometry::Mesh* mesh_ptr = mesh_obj->mesh();
            mesh_ptr->createGPUResource();
            mesh_ptr->gpuResource()->fullCopyToDevice();
        
            // create a block of data of GPUCollision structs that will be populated during collision detection
            // at most, we will have one collision per face in the mesh, so to be safe this is the amount of memory we allocate
            std::vector<Sim::GPUCollision> collisions_vec(mesh_obj->mesh()->numFaces());

            // initialize the time for each collision slot to some negative number so that we can distinguish when there is an active collision
            for (auto& gc : collisions_vec)
            {
                gc.penetration_dist = 100;
            }

            // CHECK_CUDA_ERROR(cudaHostRegister(collisions_vec.data(), collisions_vec.size()*sizeof(Sim::GPUCollision), cudaHostRegisterDefault));

            // create the GPUResource for the array of collision structs
            std::unique_ptr<Sim::WritableArrayGPUResource<Sim::GPUCollision>> arr_resource = 
                std::make_unique<Sim::WritableArrayGPUResource<Sim::GPUCollision>>(collisions_vec.data(), mesh_obj->mesh()->numFaces());
            arr_resource->allocate();

            assert(_gpu_collisions.count(new_obj) == 0);

            // move the vector into the member map
            _gpu_collisions[new_obj] = std::move(collisions_vec);
            _gpu_collision_resources[new_obj] = std::move(arr_resource);
        }
#endif
        _objects.template push_back<ObjectType*>(obj);

    }

    /** Specialization for VirtuosoRobot - for now, we just add its arms separately to the CollisionScene.
     * (we don't care about collisions between the endoscope and the tissue)
     */
    void addObject(Sim::VirtuosoRobot* virtuoso_robot)
    {
        if (virtuoso_robot->hasArm1())
            addObject(virtuoso_robot->arm1());
        if (virtuoso_robot->hasArm2())
            addObject(virtuoso_robot->arm2());
    }

    /** Detects collisions between objects in the CollisionScene.
     * When collisions are detected, collision constraints are created and added to the appropriate objects to resolve collisions.
     */
    void collideObjects();

    protected:
    /** Helper function that checks for collision between a pair of objects.
     * In general, collisions between two arbitrary objects are not supported.
     * Overrides for specific pairs of objects are implemented.
     */
    void _collideObjectPair(Sim::Object* obj1, Sim::Object* obj2);  // most general, does nothing
    void _collideObjectPair(Sim::VirtuosoArm* virtuoso_arm, Sim::XPBDMeshObject_Base* xpbd_mesh_obj);
    void _collideObjectPair(Sim::XPBDMeshObject_Base* xpbd_mesh_obj1, Sim::XPBDMeshObject_Base* xpbd_mesh_obj2);
    void _collideObjectPair(Sim::XPBDMeshObject_Base* xpbd_mesh_obj, Sim::VirtuosoArm* virtuoso_arm);
    void _collideObjectPair(Sim::XPBDMeshObject_Base* xpbd_mesh_obj, Sim::RigidObject* rigid_obj);
    void _collideObjectPair(Sim::XPBDMeshObject_Base* xpbd_mesh_obj, Sim::Object* obj);

    /** Implements the Frank-Wolfe optimization algorithm applied to finding a contact point between a SDF and a 3D triangle face. 
     * @param sdf - the signed distance function (SDF) to collide against
     * @param p1 - 1st triangle vertex
     * @param p2 - 2nd triangle vertex
     * @param p3 - 3rd tringle vertex
     * @returns the closest point on the triangle to the boundary of the SDF - if at this closest point the SDF evaluates to negative, we have a collision!
    */
    Vec3r _frankWolfe(const Geometry::SDF* sdf, const Vec3r& p1, const Vec3r& p2, const Vec3r& p3) const;

    protected:
    /** Non-owning pointer to the Simulation object that this CollisionScene belongs to */
    const Sim::Simulation* _sim;

    /** Stores the objects that have been added to the collision scene. */
    ObjectVectorType _objects;

    /** Pointer to the simulation's Embree scene.
     * Note: The CollisionScene IS NOT RESPONSIBLE for updating the Embree scene. This will be done by the Simulation class.
     * We assume that before performing any queries to the Embree scene that it is up to date.
     */
    const Geometry::EmbreeScene* _embree_scene;

    #ifdef HAVE_CUDA
    std::map<Sim::Object*, std::vector<Sim::GPUCollision> > _gpu_collisions;
    std::map<Sim::Object*, std::unique_ptr<Sim::WritableArrayGPUResource<Sim::GPUCollision> > > _gpu_collision_resources;
    #endif


};

#endif // __COLLISION_SCENE_HPP