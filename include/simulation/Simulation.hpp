#ifndef __SIMULATION_HPP
#define __SIMULATION_HPP

#include <assimp/Importer.hpp>

#include "simobject/Object.hpp"

#include "config/SimulationConfig.hpp"
#include "collision/CollisionScene.hpp"
#include "graphics/GraphicsScene.hpp"

#include <yaml-cpp/yaml.h>

#include <thread>
#include <optional>

namespace Sim
{
/** A class for managing the simulation being performed.
 * Owns the Objects, keeps track fo the sim time, etc.
 * 
 */
class Simulation
{
    public:
        /** Creates a Simulation object with specified name and default parameters.
         * @param name : the name of the Simulation
         */
        explicit Simulation(const std::string& config_filename);

        explicit Simulation(SimulationConfig&& config);

    protected:
        /** Protected default constructor - only callable from derived objects
         * Assumes that the _config object is set and exists
         */
        explicit Simulation();
    
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
        virtual void notifyKeyPressed(int key, int action, int modifiers);

        virtual void notifyMouseButtonPressed(int button, int action, int modifiers);

        virtual void notifyMouseMoved(Real x, Real y);

        virtual void notifyMouseScrolled(Real dx, Real dy);
    
    protected:
        /** Helper to add an object to the simulation given an ObjectConfig.
         * Will create an object (RigidMeshObject, RigidSphere, XPBDMeshObject, etc.) depending on the type of ObjectConfig given.
         * Adds the object to the appropriate part of the simulation (i.e. to the CollisionScene if collisions are enabled, GraphicsScene if graphics are enabled, etc.)
        */        
       Object* _addObjectFromConfig(const ObjectConfig* obj_config);

        /** Time step the simulation */
        virtual void _timeStep();

        /** Update graphics in the sim */
        virtual void _updateGraphics();

        virtual void _init();

    protected:
        /** YAML config dictionary for setting up the simulation */
        std::unique_ptr<SimulationConfig> _config;

        /** Name of the simulation */
        std::string _name;

        /** Description of the simulation */
        std::string _description;

        /** How the simulation should be run */
        SimulationMode _sim_mode;

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

        /** storage of all Objects in the simulation.
         * These objects will evolve in time through the update() method that they all provide
         */
        std::vector<std::unique_ptr<Object>> _objects;

        /** storage of objects in the simulation that are purely visual (i.e. no physics involved, just graphics)
         * update() will NOT get called on these objects.
         */
        std::vector<std::unique_ptr<Object>> _graphics_only_objects;

        std::unique_ptr<CollisionScene> _collision_scene;

        std::unique_ptr<Graphics::GraphicsScene> _graphics_scene;
};

} // namespace Sim

#endif

