#include "BeamSimulation.hpp"
#include "MeshUtils.hpp"

#include <regex>


BeamSimulation::BeamSimulation(const std::string& config_filename)
    : OutputSimulation(config_filename)
{
    _out_file << "Cantilever Beam Simulation\n";
}

void BeamSimulation::setup()
{
    // MeshUtils::createBeamObj("../resource/10x1x1_subdivided16_beam.obj", 10, 1, 1, 16);

    Simulation::setup();

    _out_file << toString() << std::endl;

    for (auto& mesh_object : _mesh_objects) {
        if (ElasticMeshObject* elastic_mesh_object = dynamic_cast<ElasticMeshObject*>(mesh_object))
        {
            elastic_mesh_object->fixVerticesWithMinY();
            _out_file << elastic_mesh_object->toString() << std::endl;


            const Eigen::Vector3d& bbox_min = mesh_object->bboxMinCoords();
            const Eigen::Vector3d& bbox_max = mesh_object->bboxMaxCoords();
            const Eigen::Vector3d& bbox_center = bbox_min + (bbox_max - bbox_min) / 2;

            unsigned tip_vertex = mesh_object->getClosestVertex(bbox_center(0), bbox_max(1), bbox_center(2));
            _beams_tip_vertex.push_back(tip_vertex);
            _beams_tip_start.push_back(mesh_object->getVertex(tip_vertex));
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
            _out_file << " "+name+"DeflectionX(m)" << " "+name+"DeflectionZ(m)" << " "+name+"DynamicsResidual" << " "+name+"PrimaryResidual" << " "+name+"ConstraintResidual" << " "+name+"VolumeRatio";
        }
    }
    _out_file << std::endl;
    
    
    _last_print_sim_time = _time;
}

void BeamSimulation::printInfo() const
{
    _out_file << _time;
    for (unsigned i = 0; i < _mesh_objects.size(); i++) {

        const Eigen::Vector3d& beam_deflection = _beams_tip_start[i] - _mesh_objects[i]->getVertex(_beams_tip_vertex[i]);

        double dynamics_residual = 0;
        double primary_residual = 0;
        double constraint_residual = 0;
        double volume_ratio = 1;
        if (XPBDMeshObject* elastic_mesh_object = dynamic_cast<XPBDMeshObject*>(_mesh_objects[i]))
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
        _out_file << " " << beam_deflection(0) << " " << beam_deflection(2) << " " << dynamics_residual << " " << primary_residual << " " << constraint_residual << " " << volume_ratio;
        
    }
    _out_file << std::endl;
}