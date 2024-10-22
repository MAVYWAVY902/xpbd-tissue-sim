#include "StaticLoadingSimulation.hpp"

#include "XPBDMeshObject.hpp"
#include "FirstOrderXPBDMeshObject.hpp"

#include <regex>

StaticLoadingSimulation::StaticLoadingSimulation(const std::string& config_filename)
    : OutputSimulation()
{
    // create a more specialized config object specifically for BeamStretchSimulations
    _config = std::make_unique<StaticLoadingSimulationConfig>(YAML::LoadFile(config_filename));

    // initialize quantities using config object
    _init();

    // extract the stretch velocity and time from the config object
    StaticLoadingSimulationConfig* static_loading_simulation_config = dynamic_cast<StaticLoadingSimulationConfig*>(_config.get());
    _center_force = static_loading_simulation_config->centerForce().value();

    // write some general info about the simulation to file
    _out_file << "Static Loading Simulation" << std::endl;
}

std::string StaticLoadingSimulation::toString() const
{
    return Simulation::toString() + "\n\tCenter Force: " + std::to_string(_center_force);
}

void StaticLoadingSimulation::setup()
{
    // call the parent setup
    Simulation::setup();

    _out_file << toString() << std::endl;


    // resize the vectors that store initial deflection state
    _deflection_vertices.resize(_mesh_objects.size());
    _deflection_starts.resize(_mesh_objects.size());
    for (unsigned i = 0; i < _mesh_objects.size(); i++)
    {
        MeshObject* mesh_object = _mesh_objects[i].get();
        // for now, assume a roughly rectangular shape and just fix the bottom corners
        if (ElasticMeshObject* mo = dynamic_cast<ElasticMeshObject*>(mesh_object))
        {
            const Eigen::Vector3d bbox_min_coords = mo->bboxMinCoords();
            const Eigen::Vector3d bbox_max_coords = mo->bboxMaxCoords();
            unsigned v1 = mo->getClosestSurfaceVertex(bbox_min_coords(0), bbox_min_coords(1), bbox_min_coords(2));
            unsigned v2 = mo->getClosestSurfaceVertex(bbox_min_coords(0), bbox_max_coords(1), bbox_min_coords(2));
            unsigned v3 = mo->getClosestSurfaceVertex(bbox_max_coords(0), bbox_max_coords(1), bbox_min_coords(2));
            unsigned v4 = mo->getClosestSurfaceVertex(bbox_max_coords(0), bbox_min_coords(1), bbox_min_coords(2));

            unsigned v5 = mo->getClosestSurfaceVertex(bbox_min_coords(0), bbox_min_coords(1), bbox_max_coords(2));
            unsigned v6 = mo->getClosestSurfaceVertex(bbox_min_coords(0), bbox_max_coords(1), bbox_max_coords(2));
            unsigned v7 = mo->getClosestSurfaceVertex(bbox_max_coords(0), bbox_max_coords(1), bbox_max_coords(2));
            unsigned v8 = mo->getClosestSurfaceVertex(bbox_max_coords(0), bbox_min_coords(1), bbox_max_coords(2));


            mo->fixVertex(v1);
            mo->fixVertex(v2);
            mo->fixVertex(v3);
            mo->fixVertex(v4);
            mo->fixVertex(v5);
            mo->fixVertex(v6);
            mo->fixVertex(v7);
            mo->fixVertex(v8);

            // write mesh object information to file
            _out_file << mo->toString() << "\n" << std::endl;

            const Eigen::Vector3d bbox_center = bbox_min_coords + (bbox_max_coords - bbox_min_coords) / 2;
            unsigned deflection_vertex = mesh_object->getClosestSurfaceVertex(bbox_center(0), bbox_center(1), bbox_min_coords(2));
            _deflection_vertices.at(i) = deflection_vertex;
            _deflection_starts.at(i) = mo->getVertex(deflection_vertex);
        }
    }

    // write appropriate CSV column headers
    _out_file << "\nTime(s)";
    for (auto& mesh_object : _mesh_objects)
    {
        if (ElasticMeshObject* elastic_mesh_object = dynamic_cast<ElasticMeshObject*>(mesh_object.get()))
        {
            std::regex r("\\s+");
            const std::string& name = std::regex_replace(elastic_mesh_object->name(), r, "");
            _out_file << " "+name+"DeflectionZ(m)" << " "+name+"DynamicsResidual" << " "+name+"PrimaryResidual" << " "+name+"ConstraintResidual" << " "+name+"VolumeRatio";
        }
    }
    _out_file << std::endl;
}

void StaticLoadingSimulation::printInfo() const
{
    _out_file << _time;
    for (unsigned i = 0; i < _mesh_objects.size(); i++)
    {
        const Eigen::Vector3d deflection = _deflection_starts[i] - _mesh_objects[i]->getVertex(_deflection_vertices[i]);
    

        double dynamics_residual = 0;
        double primary_residual = 0;
        double constraint_residual = 0;
        double volume_ratio = 1;
        if (XPBDMeshObject* elastic_mesh_object = dynamic_cast<XPBDMeshObject*>(_mesh_objects[i].get()))
        {
            primary_residual = elastic_mesh_object->primaryResidual();
            constraint_residual = elastic_mesh_object->constraintResidual();
            dynamics_residual = elastic_mesh_object->dynamicsResidual();
            volume_ratio = elastic_mesh_object->volumeRatio();
            // std::cout << "Time: " << _time << std::endl;
            // std::cout << "\tDynamics residual: " << elastic_mesh_object->dynamicsResidual() << std::endl;
            // std::cout << "\tPrimary residual: " << elastic_mesh_object->primaryResidual() << std::endl;
            // std::cout << "\tConstraint residual: " << elastic_mesh_object->constraintResidual() << std::endl;
            // std::cout << "\tVolume ratio: " << elastic_mesh_object->volumeRatio() << std::endl;
        }
        if (FirstOrderXPBDMeshObject* elastic_mesh_object = dynamic_cast<FirstOrderXPBDMeshObject*>(_mesh_objects[i].get()))
        {
            primary_residual = elastic_mesh_object->primaryResidual();
            constraint_residual = elastic_mesh_object->constraintResidual();
            dynamics_residual = elastic_mesh_object->dynamicsResidual();
            volume_ratio = elastic_mesh_object->volumeRatio();
        }

        _out_file << " " << deflection(2) << " " << dynamics_residual << " " << primary_residual << " " << constraint_residual << " " << volume_ratio;
    }
    _out_file << std::endl; 
}