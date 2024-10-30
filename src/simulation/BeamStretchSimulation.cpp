#include "BeamStretchSimulation.hpp"
#include "MeshUtils.hpp"
#include "BeamStretchSimulationConfig.hpp"

#include "XPBDMeshObject.hpp"
#include "FirstOrderXPBDMeshObject.hpp"

#include <regex>

BeamStretchSimulation::BeamStretchSimulation(const std::string& config_filename)
    : OutputSimulation()
{
    // create a more specialized config object specifically for BeamStretchSimulations
    _config = std::make_unique<BeamStretchSimulationConfig>(YAML::LoadFile(config_filename));

    // initialize quantities using config object
    _init();

    // extract the stretch velocity and time from the config object
    BeamStretchSimulationConfig* beam_stretch_simulation_config = dynamic_cast<BeamStretchSimulationConfig*>(_config.get());
    _stretch_velocity = beam_stretch_simulation_config->stretchVelocity().value();
    _stretch_time = beam_stretch_simulation_config->stretchTime().value();

    // write some general info about the simulation to file
    _out_file << "Beam Stretch Simulation" << std::endl;
}

std::string BeamStretchSimulation::toString() const
{
    return Simulation::toString() + "\n\tStretch velocity: " + std::to_string(_stretch_velocity) + "m/s\n\tStretch time: " + std::to_string(_stretch_time) + " s"; 
}

void BeamStretchSimulation::setup()
{
    // uncomment to create the beam object
    // MeshUtils::createBeamObj("../resource/tissue/tissue_block1x1x0.1_2subdiv.obj", 1, 1, 0.1, 2);

    // call the parent setup
    Simulation::setup();

    _out_file << toString() << std::endl;

    for (auto& mesh_object : _mesh_objects) {

        // fix the minY face of the beam, and attach drivers to stretch the maxY face of the beam
        if (ElasticMeshObject* elastic_mesh_object = dynamic_cast<ElasticMeshObject*>(mesh_object.get()))
        {
            // fix one minY side of the beam
            elastic_mesh_object->fixVerticesWithMinY();

            // drag all vertices on the other side of the beam
            const Eigen::Vector3d& min_coords = elastic_mesh_object->bboxMinCoords();
            const Eigen::Vector3d& max_coords = elastic_mesh_object->bboxMaxCoords();
            // get all the vertices on the maxY face of the beam
            std::vector<unsigned> free_end_verts = elastic_mesh_object->getVerticesWithY(max_coords(1));
            // create a VertexDriver object for each vertex on the maxY face of the beam
            for (unsigned i = 0; i < free_end_verts.size(); i++)
            {
                // get the original vertex coordinates
                Eigen::Vector3d vert = elastic_mesh_object->getVertex(free_end_verts[i]);

                if (vert(0) < min_coords(0) + 7*(max_coords(0)-min_coords(0))/16 ||
                    vert(0) > min_coords(0) + 9*(max_coords(0)-min_coords(0))/16)
                    continue;

                // define the driving function
                VertexDriver::DriverFunction func = [=] (const double t) {
                    // stretch in +Y direction by stretching velocity until stretch time is reached
                    if (t <= _stretch_time)
                    {
                        int int_time = static_cast<int>(t*3000);
                        int snapped_time = (int_time / 100) * 100;
                        double snapped_time_d = snapped_time / 3000.0;
                        // return Eigen::Vector3d({vert(0), vert(1)+_stretch_velocity*snapped_time_d, vert(2)+_stretch_velocity*snapped_time_d});
                        return Eigen::Vector3d({vert(0), vert(1)+_stretch_velocity*t, vert(2)+_stretch_velocity*t});
                    }
                    else
                    {
                        return Eigen::Vector3d({vert(0), vert(1)+_stretch_velocity*_stretch_time, vert(2)+_stretch_velocity*_stretch_time});
                    }
                    
                };

                // add the vertex river to the mesh object
                std::shared_ptr<VertexDriver> vd = std::make_shared<VertexDriver>("vertex"+std::to_string(i), free_end_verts[i], func);
                elastic_mesh_object->addVertexDriver(vd);
            }

            // write mesh object information to file
            _out_file << elastic_mesh_object->toString() << "\n" << std::endl;
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
            _out_file << " "+name+"DynamicsResidual" << " "+name+"PrimaryResidual" << " "+name+"ConstraintResidual" << " "+name+"VolumeRatio" << " "+name+"VelocityRMS";
        }
    }
    _out_file << std::endl;
}

void BeamStretchSimulation::printInfo() const
{
    if (_time >= _stretch_time+1)
    {
        for (auto& mesh_object : _mesh_objects)
    {
        if (ElasticMeshObject* elastic_mesh_object = dynamic_cast<ElasticMeshObject*>(mesh_object.get()))
        {
            elastic_mesh_object->clearVertexDrivers();
        }
    }
    }

    double primary_residual = 0;
    double constraint_residual = 0;
    double dynamics_residual = 0;
    double volume_ratio = 1;
    double velocity_rms = 0;
    _out_file << _time;
    for (auto& mesh_object : _mesh_objects) {
        if (XPBDMeshObject* elastic_mesh_object = dynamic_cast<XPBDMeshObject*>(mesh_object.get()))
        {
            primary_residual = elastic_mesh_object->primaryResidual();
            constraint_residual = elastic_mesh_object->constraintResidual();
            dynamics_residual = elastic_mesh_object->dynamicsResidual();
            volume_ratio = elastic_mesh_object->volumeRatio();
            MeshObject::VerticesMat velocities = mesh_object->velocities();
            const double frob_norm = velocities.norm();
            velocity_rms = std::sqrt(frob_norm/velocities.rows());
        }
        if (FirstOrderXPBDMeshObject* elastic_mesh_object = dynamic_cast<FirstOrderXPBDMeshObject*>(mesh_object.get()))
        {
            primary_residual = elastic_mesh_object->primaryResidual();
            constraint_residual = elastic_mesh_object->constraintResidual();
            dynamics_residual = elastic_mesh_object->dynamicsResidual();
            volume_ratio = elastic_mesh_object->volumeRatio();
            MeshObject::VerticesMat velocities = mesh_object->velocities();
            const double frob_norm = velocities.norm();
            velocity_rms = std::sqrt(frob_norm/velocities.rows());
        }
        _out_file << " " << dynamics_residual << " " << primary_residual << " " << constraint_residual << " " << volume_ratio << " " << velocity_rms;
    }
    _out_file << std::endl;
}