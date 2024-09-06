#include "BeamStretchSimulation.hpp"
#include "MeshUtils.hpp"
#include "BeamStretchSimulationConfig.hpp"

#include <regex>

BeamStretchSimulation::BeamStretchSimulation(const std::string& config_filename)
    : Simulation()
{
    // create a more specialized config object specifically for BeamStretchSimulations
    _config = std::make_unique<BeamStretchSimulationConfig>(YAML::LoadFile(config_filename));

    // initialize quantities using config object
    _init();

    // initialize the output file
    _out_file = std::ofstream("../output/beam_stretch/out_" + _name + ".txt");

    // extract the stretch velocity and time from the config object
    BeamStretchSimulationConfig* beam_stretch_simulation_config = dynamic_cast<BeamStretchSimulationConfig*>(_config.get());
    _stretch_velocity = beam_stretch_simulation_config->stretchVelocity().value();
    _stretch_time = beam_stretch_simulation_config->stretchTime().value();

    // write some general info about the simulation to file
    _out_file << "Beam Stretch Simulation" << std::endl;
    _out_file << _description << std::endl;

    _out_file << "Stretch velocity: " << _stretch_velocity << " m/s" << std::endl;
    _out_file << "Stretching time: " << _stretch_time << " s" << std::endl;
}

void BeamStretchSimulation::setup()
{
    // uncomment to create the beam object
    // MeshUtils::createBeamObj("../resource/beam/cube16.obj", 1, 1, 1, 16);

    // call the parent setup
    Simulation::setup();

    for (auto& mesh_object : _mesh_objects) {

        // fix the minY face of the beam, and attach drivers to stretch the maxY face of the beam
        if (ElasticMeshObject* elastic_mesh_object = dynamic_cast<ElasticMeshObject*>(mesh_object))
        {
            // fix one minY side of the beam
            elastic_mesh_object->fixVerticesWithMinY();

            // drag all vertices on the other side of the beam
            const Eigen::Vector3d& max_coords = elastic_mesh_object->bboxMaxCoords();
            // get all the vertices on the maxY face of the beam
            std::vector<unsigned> free_end_verts = elastic_mesh_object->getVerticesWithY(max_coords(1));
            // create a VertexDriver object for each vertex on the maxY face of the beam
            for (unsigned i = 0; i < free_end_verts.size(); i++)
            {
                // get the original vertex coordinates
                Eigen::Vector3d vert = elastic_mesh_object->getVertex(free_end_verts[i]);
                // define the driving function
                VertexDriver::DriverFunction func = [=] (const double t) {
                    // stretch in +Y direction by stretching velocity until stretch time is reached
                    if (t <= _stretch_time)
                    {
                        return Eigen::Vector3d({vert(0), vert(1)+_stretch_velocity*t, vert(2)});
                    }
                    else
                    {
                        return Eigen::Vector3d({vert(0), vert(1)+_stretch_velocity*_stretch_time, vert(2)});
                    }
                    
                };

                // add the vertex river to the mesh object
                VertexDriver vd("vertex"+std::to_string(i), free_end_verts[i], func);
                elastic_mesh_object->addVertexDriver(vd);
            }

            // write material information to file
            _out_file << "\n" << elastic_mesh_object->name() + " Material:\n\t" << "Density: " << elastic_mesh_object->material().density() << std::endl;
            _out_file << "\tE: " << elastic_mesh_object->material().E() << std::endl;
            _out_file << "\tnu: " << elastic_mesh_object->material().nu() << std::endl;
            _out_file << "\tmu: " << elastic_mesh_object->material().mu() << std::endl;
            _out_file << "\tlambda: " << elastic_mesh_object->material().lambda() << std::endl;
        }

        // write XPBD-configuration-specific information to file
        if (XPBDMeshObject* xpbd_mesh_object = dynamic_cast<XPBDMeshObject*>(mesh_object))
        {
            _out_file << "\nXPBD:" << std::endl;
            _out_file << "\tNum solver iters: " << xpbd_mesh_object->numSolverIters() << std::endl;
            _out_file << "\tSolve mode: " << xpbd_mesh_object->solveMode() << std::endl;
        }
    }

    // write appropriate CSV column headers
    _out_file << "\nTime(s)";
    for (auto& mesh_object : _mesh_objects)
    {
        if (ElasticMeshObject* elastic_mesh_object = dynamic_cast<ElasticMeshObject*>(mesh_object))
        {
            std::regex r("\\s+");
            const std::string& name = std::regex_replace(elastic_mesh_object->name(), r, "");
            _out_file << " "+name+"DynamicsResidual" << " "+name+"PrimaryResidual" << " "+name+"ConstraintResidual" << " "+name+"VolumeRatio";
        }
    }
    _out_file << std::endl;
}

void BeamStretchSimulation::update()
{
    Simulation::update();
}

void BeamStretchSimulation::_timeStep()
{
    Simulation::_timeStep();

    if (_time - _last_print_sim_time >= 1e-3)
    {
        _printInfo();
        _last_print_sim_time = _time;
    }
}

void BeamStretchSimulation::_printInfo()
{
    double primary_residual = 0;
    double constraint_residual = 0;
    double dynamics_residual = 0;
    double volume_ratio = 1;
    _out_file << _time;
    for (auto& mesh_object : _mesh_objects) {
        if (XPBDMeshObject* elastic_mesh_object = dynamic_cast<XPBDMeshObject*>(mesh_object))
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
        _out_file << " " << dynamics_residual << " " << primary_residual << " " << constraint_residual << " " << volume_ratio;
    }
    _out_file << std::endl;
}