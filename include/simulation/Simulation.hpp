#ifndef __SIMULATION_HPP
#define __SIMULATION_HPP

#include <easy3d/viewer/viewer.h>
#include <easy3d/renderer/renderer.h>
#include <easy3d/renderer/camera.h>
#include <easy3d/renderer/drawable_lines.h>
#include <easy3d/renderer/drawable_points.h>
#include <easy3d/renderer/drawable_triangles.h>
#include <easy3d/renderer/text_renderer.h>
#include <easy3d/core/types.h>
#include <easy3d/util/resource.h>
#include <easy3d/util/initializer.h>

#include <assimp/Importer.hpp>

#include "TextRenderingViewer.hpp"
#include "RigidMeshObject.hpp"
#include "XPBDMeshObject.hpp"
#include "FastFEMMeshObject.hpp"

#include "config/SimulationConfig.hpp"

#include <yaml-cpp/yaml.h>

#include <thread>
#include <optional>

class TextRenderingViewer;

/** A class for managing the simulation being performed.
 * Owns the MeshObjects, keeps track fo the sim time, etc.
 * 
 */
class Simulation
{
    public:
        /** Creates a Simulation object with specified name and default parameters.
         * @param name : the name of the Simulation
         */
        explicit Simulation(const std::string& config_filename);

    protected:
        /** Protected default constructor - only callable from derived objects
         * Assumes that the _config object is set and exists
         */
        explicit Simulation();
    
    public:
        virtual std::string toString() const;
        virtual std::string type() const { return "Simulation"; }

        /** Adds a MeshObject to the simulation. Will add its Drawables to the Viewer as well.
         * @param mesh_obj : the MeshObject being added  
        */        
        void addObject(MeshObject* mesh_obj);

        double time() const { return _time; }

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

        virtual void notifyMouseMoved(double x, double y);
    
    protected:
        /** Time step the simulation */
        virtual void _timeStep();

        /** Update graphics in the sim */
        void _updateGraphics();

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
        double _time;
        /** The time step to take */
        double _time_step;
        /** End time of the simulation */
        double _end_time;
        /** Number of time steps taken */
        size_t _steps_taken;
        /** Acceleration due to gravity */
        double _g_accel;
        /** Time to wait inbetween viewer updates (in ms). This is 1/fps */
        int _viewer_refresh_time;

        /** the Viewer which renders graphics
         * unique_ptr used here for lazy initialization, since easy3d::Viewer must be instantiated 
         * AFTER the easy3d::initialize() call.
         */
        std::unique_ptr<TextRenderingViewer> _viewer;

        /** storage of all MeshObjects in the simulation */
        std::vector<MeshObject*> _mesh_objects;
};

#endif

