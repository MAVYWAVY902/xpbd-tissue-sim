#ifndef __SIMULATION_HPP
#define __SIMULATION_HPP

#include <assimp/Importer.hpp>

#include "simobject/Object.hpp"
#include "simobject/XPBDMeshObject.hpp"
#include "simobject/RigidMeshObject.hpp"
#include "simobject/RigidObject.hpp"
#include "simobject/RigidPrimitives.hpp"
#include "simobject/VirtuosoArm.hpp"
#include "simobject/VirtuosoRobot.hpp"

#include "config/simulation/SimulationConfig.hpp"
#include "collision/CollisionScene.hpp"
#include "graphics/GraphicsScene.hpp"
#include "geometry/embree/EmbreeScene.hpp"

#include "common/VariadicVectorContainer.hpp"
#include "common/SimulationTypeDefs.hpp"
#include "common/SimulationInput.hpp"

#include <yaml-cpp/yaml.h>

#include <thread>
#include <optional>
#include <functional>

namespace Sim
{
/** A class for managing the simulation being performed.
 * Owns the Objects, keeps track fo the sim time, etc.
 * 
 */
class Simulation
{
    public:
    using ObjectVectorType = VariadicVectorContainerFromTypeList<SimulationObjectTypes>::unique_ptr_type;

    public:
    struct CallbackInfo
    {
        std::function<void()> callback;
        double interval;
        double next_exec_time;

        CallbackInfo(std::function<void()> cb, double intvl, double start_time)
            : callback(std::move(cb)), interval(intvl), next_exec_time(start_time + interval)
        {}
    };

    public:
        explicit Simulation(const Config::SimulationConfig* config);

    protected:
        /** Protected default constructor - only callable from derived objects
         * Assumes that the _config object is set and exists
         */
        // explicit Simulation();
    
    public:
        virtual std::string toString(const int indent) const;
        virtual std::string type() const { return "Simulation"; }

        /** Adds a MeshObject to the simulation. Will add its Drawables to the Viewer as well.
         * @param mesh_obj : the MeshObject being added  
        */        
        // void addObject(std::shared_ptr<MeshObject> mesh_obj);

        Real time() const { return _time; }

        Real dt() const { return _time_step; }
        
        Real gAccel() const { return _g_accel; }

        const Graphics::GraphicsScene* graphicsScene() const { return _graphics_scene.get(); }
        const Geometry::EmbreeScene* embreeScene() const { return _embree_scene.get(); }
        void updateEmbreeScene() { _embree_scene->update(); }
        const CollisionScene* collisionScene() const { return _collision_scene.get(); }

        const ObjectVectorType& objects() const { return _objects; }
        const ObjectVectorType& graphicsObjects() const { return _graphics_only_objects; }
        /** Performs setup for the Simulation.
         * Creates initial MeshObjects, sets up Viewer, etc.
         */
        virtual void setup();

        /** Runs the simulation.
         * Spawns a separate thread to do updates.
         */
        int run();

        /** Updates the simulation at a fixed time step. */
        virtual void update();

        /** Notifies the simulation that a key has been pressed in the viewer.
         * @param key : the key that was pressed
         * @param action : the action performed on the keyboard
         * @param modifiers : the modifiers (i.e. Shift, Ctrl, Alt)
         */
        virtual void notifyKeyPressed(SimulationInput::Key key, SimulationInput::KeyAction action, int modifiers);

        virtual void notifyMouseButtonPressed(SimulationInput::MouseButton button, SimulationInput::MouseAction action, int modifiers);

        virtual void notifyMouseMoved(double x, double y);

        virtual void notifyMouseScrolled(double dx, double dy);

        template<typename CallbackT>
        void addCallback(double interval, CallbackT&& lambda)
        {
            std::function<void()> wrapper = [lambda = std::forward<CallbackT>(lambda)]() {
                lambda();
            };

            _callbacks.emplace_back(std::move(wrapper), interval, _time);
        }
    
    protected:
        /** Helper to add an object to the simulation given an ObjectConfig.
         * Will create an object (RigidMeshObject, RigidSphere, XPBDMeshObject, etc.) depending on the type of ObjectConfig given.
         * Adds the object to the appropriate part of the simulation (i.e. to the CollisionScene if collisions are enabled, GraphicsScene if graphics are enabled, etc.)
        */
        template<typename ConfigType>        
        typename ConfigType::ObjectType* _addObjectFromConfig(const ConfigType* obj_config)
        {
            using ObjPtrType = std::unique_ptr<typename ConfigType::ObjectType>;

            ObjPtrType new_obj = obj_config->createObject(this);
            new_obj->setup();

            // handle XPBDMeshObjects slightly differently so that we can tell the CollisionScene if self-collisions are enabled
            if constexpr (std::is_convertible_v<ConfigType*, Config::XPBDMeshObjectConfig*>)
            {
                std::cout << "Self collisions enabled: " << obj_config->selfCollisions() << std::endl;
                if (obj_config->collisions() && !obj_config->graphicsOnly())
                    _collision_scene->addObject(new_obj.get(), obj_config->selfCollisions());
            }
            else
            {
                // add the new object to the collision scene if collisions are enabled
                if (obj_config->collisions() && !obj_config->graphicsOnly())
                    _collision_scene->addObject(new_obj.get());
            }
            
            // add the new object to the graphics scene to be visualized
            if (_graphics_scene)
            {
                _graphics_scene->addObject(new_obj.get(), obj_config->renderConfig());
            }

            // if we get to here, we have successfully created a new MeshObject of some kind
            // so add the new object to the simulation
            typename ConfigType::ObjectType* tmp_ptr = new_obj.get();
            if (obj_config->graphicsOnly())
            {
                _graphics_only_objects.template push_back<ObjPtrType>(std::move(new_obj));
            }
            else
            {
                _objects.template push_back<ObjPtrType>(std::move(new_obj));
            }
            return tmp_ptr;
        }

        /** Time step the simulation */
        virtual void _timeStep();

        /** Update graphics in the sim */
        virtual void _updateGraphics();

    protected:
        /** Whether or not the simulation has been setup already with a call to setup()  */
        bool _setup;
        
        /** YAML config dictionary for setting up the simulation */
        const Config::SimulationConfig* _config;

        /** Name of the simulation */
        std::string _name;

        /** Description of the simulation */
        std::string _description;

        /** How the simulation should be run */
        Config::SimulationMode _sim_mode;

        /** Current sim time */
        Real _time;
        /** The time step to take */
        Real _time_step;
        /** End time of the simulation */
        Real _end_time;
        /** Number of time steps taken */
        size_t _steps_taken;
        /** Acceleration due to gravity */
        Real _g_accel;
        /** Time to wait inbetween viewer updates (in ms). This is 1/fps */
        int _viewer_refresh_time;
        /** Time to wait inbetween collision checks (in seconds). This is 1/collision_rate */
        Real _time_between_collision_checks;

        Real _last_collision_detection_time;

        /** scheduled callbacks */
        std::vector<CallbackInfo> _callbacks;

        /** storage of all Objects in the simulation.
         * These objects will evolve in time through the update() method that they all provide
         */
        ObjectVectorType _objects;

        /** storage of objects in the simulation that are purely visual (i.e. no physics involved, just graphics)
         * update() will NOT get called on these objects.
         */
        ObjectVectorType _graphics_only_objects;

        std::unique_ptr<CollisionScene> _collision_scene;

        std::unique_ptr<Graphics::GraphicsScene> _graphics_scene;

        std::unique_ptr<Geometry::EmbreeScene> _embree_scene;
};

} // namespace Sim

#endif

