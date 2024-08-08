#include "Simulation.hpp"

#include <gmsh.h>

Simulation::Simulation(const std::string& name, const std::string& config_filename)
    : _name(name)
{
    // initialize gmsh
    gmsh::initialize();

    // initialize easy3d
    easy3d::initialize();

    // now we can create the Viewer
    _viewer = std::make_unique<TextRenderingViewer>(_name);
    _viewer->set_usage("");

    // open the config file using yaml-cpp
    _config = YAML::LoadFile(config_filename);
    // set simulation properties based on YAML file
    _time_step = _config["time-step"].as<double>();
    _end_time = _config["end-time"].as<double>();
}

void Simulation::addObject(MeshObject* mesh_object)
{
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
    // create a MeshObject for each object specified in the YAML file
    for (const auto& obj : _config["objects"])
    {
        // extract name and type information
        const std::string& name = obj["name"].as<std::string>();
        const std::string& type = obj["type"].as<std::string>();

        // create the specified type of object
        if(type == "XPBDMeshObject")
        { 
            XPBDMeshObject* mesh_obj = new XPBDMeshObject(name, obj);
            addObject(mesh_obj);
        }
        if(type == "RigidMeshObject")
        {
            RigidMeshObject* mesh_obj = new RigidMeshObject(name, obj);
            addObject(mesh_obj);
        }
    }

    // create a ground plane
    RigidMeshObject* ground_plane = new RigidMeshObject("ground");
    ground_plane->createPlaneGeometry({0, 0, 0}, 20);
    addObject(ground_plane);

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
        if (_time > wall_time_elapsed_s)
        {
            continue;
        }

        // update each MeshObject
        for (auto& mo : _mesh_objects)
        {
            mo->update(_time_step);
        }

        // the time in ms since the viewer was last redrawn
        auto time_since_last_redraw_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - last_redraw).count();
        // we want ~30 fps, so update the viewer every 33 ms
        if (time_since_last_redraw_ms > 33)
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

            last_redraw = std::chrono::steady_clock::now();
        }

        // increment the time by the time step
        _time += _time_step;


        
    }

    // one final redraw of final state

    // update the sim time text
    _viewer->editText("time", "Sim Time: " + std::to_string(_time) + " s");

    for (auto& mesh_object : _mesh_objects)
    {
        mesh_object->updateGraphics();
    }

    // update the viewer
    _viewer->update();

    auto end = std::chrono::steady_clock::now();
    std::cout << "Simulating " << _end_time << " seconds took " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms" << std::endl;
}

int Simulation::run()
{
    // first, setup
    setup();

    // spwan the update thread
    std::thread update_thread(&Simulation::update, this);

    // run the Viewer
    _viewer->fit_screen();
    return _viewer->run();
}