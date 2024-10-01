#include "Simulation.hpp"
#include "config/RigidMeshObjectConfig.hpp"
#include "config/XPBDMeshObjectConfig.hpp"

#include "MeshUtils.hpp"

#include <gmsh.h>

void Simulation::_init()
{
    // initialize gmsh
    gmsh::initialize();

    // initialize easy3d
    easy3d::initialize();

    // set simulation properties based on YAML file
    _name = _config->name().value();
    _description = _config->description().value();
    _time_step = _config->timeStep().value();
    _end_time = _config->endTime().value();
    _time = 0;
    _g_accel = _config->gAccel().value();
    _viewer_refresh_time = 1/_config->fps().value()*1000;

    // set the Simulation mode from the YAML config
    if (_config->simMode().has_value())
    {
        _sim_mode = _config->simMode().value();
    }

    // now we can create the Viewer
    _viewer = std::make_unique<TextRenderingViewer>(_name);
    _viewer->set_usage("");
    _viewer->registerSimulation(this);
    _viewer->resize(1000,650);
}

Simulation::Simulation(const std::string& config_filename)
{
    _config = std::make_unique<SimulationConfig>(YAML::LoadFile(config_filename));
    _init();
}

Simulation::Simulation()
{

}

std::string Simulation::toString() const
{
    return  type() + " '" + _name + "':\n\tTime step: " + std::to_string(_time_step) + " s\n\tEnd time: " + std::to_string(_end_time) +
        " s\n\tGravity: " + std::to_string(_g_accel) + " m/s^2";
}

void Simulation::addObject(MeshObject* mesh_object)
{
    mesh_object->setSimulation(this);
    
    // add new object to MeshObjects container
    _mesh_objects.push_back(mesh_object);
    

    // add the Drawables for the new MeshObject to the Viewer
    for (const auto& pt_drawable : mesh_object->renderer()->points_drawables())
    {
        _viewer->add_drawable(pt_drawable);
    }
    for (const auto& tri_drawable : mesh_object->renderer()->triangles_drawables())
    {
        _viewer->add_drawable(tri_drawable);
    }
    
}

void Simulation::setup()
{   
    for (const auto& obj_config : _config->meshObjectConfigs())
    {
        // try downcasting
        if (XPBDMeshObjectConfig* xpbd_config = dynamic_cast<XPBDMeshObjectConfig*>(obj_config.get()))
        {
            XPBDMeshObject* new_obj = new XPBDMeshObject(xpbd_config);
            addObject(new_obj);
        }
        else if (FastFEMMeshObjectConfig* fem_config = dynamic_cast<FastFEMMeshObjectConfig*>(obj_config.get()))
        {
            FastFEMMeshObject* new_obj = new FastFEMMeshObject(fem_config);
            addObject(new_obj);
        }
        else if (RigidMeshObjectConfig* rigid_config = dynamic_cast<RigidMeshObjectConfig*>(obj_config.get()))
        {
            RigidMeshObject* new_obj = new RigidMeshObject(rigid_config);
            addObject(new_obj);
        }
    }

    // add text that displays the current Sim Time   
    _viewer->addText("time", "Sim Time: 0.000 s", 10.0f, 10.0f, 15.0f, easy3d::TextRenderer::ALIGN_LEFT, TextRenderingViewer::Font::MAO, easy3d::vec3(0,0,0), 0.5f, false);
}

void Simulation::update()
{
    auto start = std::chrono::steady_clock::now();

    // the start time in wall clock time of the simulation
    auto wall_time_start = std::chrono::steady_clock::now();
    // the wall time of the last viewer redraw
    auto last_redraw = std::chrono::steady_clock::now();

    // loop until end time is reached
    while(_time < _end_time)
    {
        // the elapsed seconds in wall time since the simulation has started
        double wall_time_elapsed_s = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - wall_time_start).count() / 1000000000.0;
        // if the simulation is ahead of the current elapsed wall time, stall
        if (_sim_mode == SimulationMode::VISUALIZATION && _time > wall_time_elapsed_s)
        {
            continue;
        }

        _timeStep();

        // the time in ms since the viewer was last redrawn
        auto time_since_last_redraw_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - last_redraw).count();
        // we want ~30 fps, so update the viewer every 33 ms
        if (time_since_last_redraw_ms > _viewer_refresh_time)
        {
            // std::cout << _haptic_device_manager->getPosition() << std::endl;
            _updateGraphics();

            last_redraw = std::chrono::steady_clock::now();
        }
        
    }

    // one final redraw of final state
    _updateGraphics();

    auto end = std::chrono::steady_clock::now();
    std::cout << "Simulating " << _end_time << " seconds took " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;
}

void Simulation::_timeStep()
{
    // update each MeshObject
    for (auto& mo : _mesh_objects)
    {
        mo->update(_time_step, _g_accel);
    }

    // increment the time by the time step
    _time += _time_step;
}

void Simulation::_updateGraphics()
{
    // update the sim time text
    _viewer->editText("time", "Sim Time: " + std::to_string(_time) + " s");

    // make sure graphics are up to date for all the objects in the sim
    for (auto& mesh_object : _mesh_objects)
    {
        mesh_object->updateGraphics();
    }

    // update the viewer
    _viewer->update();
}

void Simulation::notifyKeyPressed(int /* key */, int action, int /* modifiers */)
{
    // action = 0 ==> key up event
    // action = 1 ==> key down event
    // action = 2 ==> key hold event
    
    // if key is pressed down or held, we want to time step
    if (_sim_mode == SimulationMode::FRAME_BY_FRAME && action > 0)
    {
        _timeStep();
        _updateGraphics();
    }
}

void Simulation::notifyMouseButtonPressed(int /* button */, int /* action */, int /* modifiers */)
{
    // button = 0 ==> left mouse button
    // button = 1 ==> right mouse button
    // action = 0 ==> mouse up
    // action = 1 ==> mouse down
    
    // do nothing
}

void Simulation::notifyMouseMoved(double /* x */, double /* y */)
{
    // do nothing
}

int Simulation::run()
{
    // first, setup
    setup();

    // spwan the update thread
    std::thread update_thread;
    if (_sim_mode != SimulationMode::FRAME_BY_FRAME)
    {
        update_thread = std::thread(&Simulation::update, this);
    }

    // run the Viewer
    _viewer->fit_screen();
    return _viewer->run();
}