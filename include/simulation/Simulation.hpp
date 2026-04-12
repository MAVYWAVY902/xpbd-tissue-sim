#ifndef __SIMULATION_HPP
#define __SIMULATION_HPP

#include <assimp/Importer.hpp>

#include "simulation/SimulationLogger.hpp"
#include "simulation/SimulationStateRecorder.hpp"

#include "simobject/Object.hpp"
#include "simobject/XPBDMeshObject.hpp"
#include "simobject/RigidMeshObject.hpp"
#include "simobject/RigidObject.hpp"
#include "simobject/RigidPrimitives.hpp"
#include "simobject/VirtuosoArm.hpp"
#include "simobject/VirtuosoRobot.hpp"

#include "config/simulation/SimulationConfig.hpp"
#include "collision/CollisionScene.hpp"
// Forward declaration only — full header included in Simulation.cpp
namespace Graphics { class GraphicsScene; }
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
class Simulation : public PhysicsContext
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
        virtual ~Simulation();  // defined in .cpp (needs complete GraphicsScene type)


    protected:
        // Step 1: 只做一个最简单的两点神经
        Vec3r _nerveP0;
        Vec3r _nerveP1;
        Real  _nerveInvMass0;
        Real  _nerveInvMass1;
        Real  _nerveRestLen;


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
        
        const Vec3r& gAccel() const { return _g_accel; }  // Returns 3D gravity vector
        
        const Config::SimulationConfig* config() const { return _config; }

        const Graphics::GraphicsScene* graphicsScene() const {
            return static_cast<const Graphics::GraphicsScene*>(_graphics_scene_raw);
        }
        const Geometry::EmbreeScene* embreeScene() const { return _embree_scene.get(); }
        void updateEmbreeScene() { _embree_scene->update(); }
        const CollisionScene* collisionScene() const { return _collision_scene.get(); }

        const ObjectVectorType& objects() const { return _objects; }
        const ObjectVectorType& graphicsObjects() const { return _graphics_only_objects; }

        /** Adds a new material to the sim. Useful for setting up sims without a config file. */
        void addMaterial(const ElasticMaterial& mat) { _materials.push_back(mat); }

        /** Returns the material with the specified name, if it exists. If it doesn't exist,
         * the program will throw an error and exit.
         */
        const ElasticMaterial& getMaterial(const std::string& name) const 
        { 
            for (const auto& mat : _materials)
            {
                if (mat.name() == name)
                    return mat;
            }

            std::cerr << "Material with name " << name << " does not exist in the simulation!" << std::endl;
            assert(0);

            return _materials[0];
        }

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

            std::cout << "\n[LOADING] ======================================\n";
            std::cout << "[LOADING] Creating object: '" << obj_config->name() << "'\n";
            std::cout << "[LOADING] Type: " << typeid(typename ConfigType::ObjectType).name() << "\n";

            ObjPtrType new_obj = obj_config->createObject(this);
            new_obj->setup();

            // Print size/bounding box information if it's a MeshObject
            if (const Sim::MeshObject* mo = dynamic_cast<const Sim::MeshObject*>(new_obj.get()))
            {
                const auto* mesh = mo->mesh();
                if (mesh && mesh->numVertices() > 0)
                {
                    Vec3r min_coord = mesh->vertex(0);
                    Vec3r max_coord = mesh->vertex(0);
                    
                    for (int i = 1; i < mesh->numVertices(); ++i)
                    {
                        const Vec3r& v = mesh->vertex(i);
                        min_coord = min_coord.cwiseMin(v);
                        max_coord = max_coord.cwiseMax(v);
                    }
                    
                    Vec3r size = max_coord - min_coord;
                    Vec3r center = (max_coord + min_coord) / 2.0;
                    Real max_dim = size.maxCoeff();
                    
                    std::cout << "[LOADING] Mesh stats:\n";
                    std::cout << "[LOADING]   Vertices: " << mesh->numVertices() << "\n";
                    std::cout << "[LOADING]   Faces: " << mesh->numFaces() << "\n";
                    std::cout << "[LOADING]   Bounding box min: (" << min_coord.transpose() << ") m\n";
                    std::cout << "[LOADING]   Bounding box max: (" << max_coord.transpose() << ") m\n";
                    std::cout << "[LOADING]   Size (LxWxH): (" << size.transpose() << ") m\n";
                    std::cout << "[LOADING]   Max dimension: " << max_dim << " m = " << (max_dim * 1000) << " mm\n";
                    std::cout << "[LOADING]   Center: (" << center.transpose() << ") m\n";
                }
            }

            std::cout << "[LOADING] ======================================\n\n";

            // handle XPBDMeshObjects slightly differently so that we can tell the CollisionScene if self-collisions are enabled
            if constexpr (std::is_convertible_v<ConfigType*, Config::XPBDMeshObjectConfig*>)
            {
                if (obj_config->collisions() && !obj_config->graphicsOnly())
                {
                    std::cout << "[sim] DEBUG: About to add object '" << obj_config->name() << "' to COLLISION scene (self-collision=" << obj_config->selfCollisions() << ")...\n" << std::flush;
                    _collision_scene->addObject(new_obj.get(), obj_config->selfCollisions());
                    std::cout << "[sim] DEBUG: Successfully added '" << obj_config->name() << "' to collision scene!\n" << std::flush;
                }
            }
            else
            {
                // add the new object to the collision scene if collisions are enabled
                if (obj_config->collisions() && !obj_config->graphicsOnly())
                {
                    std::cout << "[sim] DEBUG: About to add object '" << obj_config->name() << "' to COLLISION scene...\n" << std::flush;
                    _collision_scene->addObject(new_obj.get());
                    std::cout << "[sim] DEBUG: Successfully added '" << obj_config->name() << "' to collision scene!\n" << std::flush;
                }
            }
            
            // add the new object to the graphics scene to be visualized
#ifndef NO_GRAPHICS
            if (_graphics_scene_raw)
            {
                static_cast<Graphics::GraphicsScene*>(_graphics_scene_raw)->addObject(new_obj.get(), obj_config->renderConfig());
            }
#endif

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

        /** Hook called after collision detection, before XPBD solve.
         *  Subclasses can override to add extra collision constraints. */
        virtual void _onPostCollisionDetection() {}

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
        /** Wall clock sim start time */
        std::chrono::time_point<std::chrono::steady_clock> _wall_time_start;
        /** The time step to take */
        Real _time_step;
        /** End time of the simulation */
        Real _end_time;
        /** Number of time steps taken */
        size_t _steps_taken;
        /** Acceleration due to gravity (3D vector) */
        Vec3r _g_accel;
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

        /** storage of the elastic materials used by deformable objects in the simulation */
        std::vector<ElasticMaterial> _materials;

        /** Manages collision detection and creating constraints for collision response.
         * Only objects with collisions enabled will be added to the CollisionScene.
         */
        std::unique_ptr<CollisionScene> _collision_scene;

        /** Manages graphics objects and displaying things to the screen. */
        // When NO_GRAPHICS: stored as void* (same 8 bytes, no complete type needed)
        // When full build: Simulation.cpp casts this back to GraphicsScene*
        void* _graphics_scene_raw = nullptr;

        /** Embree is used to make some ray-tracing and collision queries.
         * The EmbreeScene acts as an interface between the Simulation and the Embree library.
          */
        std::unique_ptr<Geometry::EmbreeScene> _embree_scene;

        /** Responsible for logging various simulation quantities. */
        std::unique_ptr<SimulationLogger> _logger;

        /** Responsible for recording state snapshots for offline analysis. */
        std::unique_ptr<SimulationStateRecorder> _state_recorder;
};

} // namespace Sim

#endif

