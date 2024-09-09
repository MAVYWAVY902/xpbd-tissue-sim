#include "BeamSimulation.hpp"
#include "MeshUtils.hpp"


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
    if (ElasticMeshObject* elastic_mesh_object = dynamic_cast<ElasticMeshObject*>(_mesh_objects[0]))
    {
        elastic_mesh_object->fixVerticesWithMinY();
        _out_file << elastic_mesh_object->toString() << std::endl;
    }

    const Eigen::Vector3d& bbox_min = _mesh_objects[0]->bboxMinCoords();
    const Eigen::Vector3d& bbox_max = _mesh_objects[0]->bboxMaxCoords();
    const Eigen::Vector3d& bbox_center = bbox_min + (bbox_max - bbox_min) / 2;
    _out_file << "\nBeam size: " << bbox_max(0) - bbox_min(0) << "x" << bbox_max(1) - bbox_min(1) << "x" << bbox_max(2) - bbox_min(2) << " (m)" << std::endl;

    _out_file << "\n\nTime(s) DeflectionX(m) DeflectionZ(m) DynamicsResidual PrimaryResidual ConstraintResidual" << std::endl;
    
    _beam_tip_vertex = _mesh_objects[0]->getClosestVertex(bbox_center(0), bbox_max(1), bbox_center(2));
    std::cout << "Beam tip: " << _mesh_objects[0]->getVertex(_beam_tip_vertex) << std::endl;
    _beam_tip_start = _mesh_objects[0]->getVertex(_beam_tip_vertex);
    _last_print_sim_time = _time;
}

void BeamSimulation::printInfo() const
{
    const Eigen::Vector3d& beam_deflection = _beam_tip_start - _mesh_objects[0]->getVertex(_beam_tip_vertex);

    double dynamics_residual, primary_residual, constraint_residual;
    if (XPBDMeshObject* elastic_mesh_object = dynamic_cast<XPBDMeshObject*>(_mesh_objects[0]))
    {
        primary_residual = elastic_mesh_object->primaryResidual();
        constraint_residual = elastic_mesh_object->constraintResidual();
        dynamics_residual = elastic_mesh_object->dynamicsResidual();
        // std::cout << "Time: " << _time << std::endl;
        // std::cout << "\tBeam Deflection X: " << beam_deflection(0) << std::endl;
        // std::cout << "\tBeam Deflection Y: " << beam_deflection(1) << std::endl;
        // std::cout << "\tBeam Deflection Z: " << beam_deflection(2) << std::endl;
        // std::cout << "\tPrimary residual: " << elastic_mesh_object->primaryResidual() << std::endl;
        // std::cout << "\tConstraint residual: " << elastic_mesh_object->constraintResidual() << std::endl;
    }
    _out_file << _time << " " << beam_deflection(0) << " " << beam_deflection(2) << " " << dynamics_residual << " " << primary_residual << " " << constraint_residual << std::endl;
}