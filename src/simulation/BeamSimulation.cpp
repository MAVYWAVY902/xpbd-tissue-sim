#include "simulation/BeamSimulation.hpp"
#include "utils/MeshUtils.hpp"

#include "simobject/XPBDMeshObject.hpp"
#include "solver/xpbd_solver/XPBDSolver.hpp"

#include <regex>

namespace Sim
{

BeamSimulation::BeamSimulation(const Config::BeamSimulationConfig* config)
    : Simulation(config)
{
}

void BeamSimulation::setup()
{
    Simulation::setup();

    // get all the XPBDMeshObjects
    _objects.for_each_element<std::unique_ptr<XPBDMeshObject_Base>, std::unique_ptr<FirstOrderXPBDMeshObject_Base>>([&](auto& obj)
    {
        _xpbd_objs.emplace_back(obj.get());
    });

    for (auto& xpbd_obj : _xpbd_objs) 
    {
        // fix the "left" side of the beam
        // i.e. fix vertices that have the minimum Y-coordinate in the mesh
        Geometry::AABB aabb = xpbd_obj.boundingBox();
        std::vector<int> vertices_to_fix = xpbd_obj.mesh()->getVerticesWithY(aabb.min[1]);
        for (const auto& v : vertices_to_fix)
            xpbd_obj.fixVertex(v);

        // find the "tip" vertex - we take the tip to be in the center of the bottom-right edge, since this is where most vertical deflection happens
        const Vec3r bbox_center = aabb.center();
        unsigned tip_vertex = xpbd_obj.mesh()->getClosestVertex( {bbox_center[0], aabb.max[1], bbox_center[2]} );
        _beams_tip_vertex.push_back(tip_vertex);
        // track where the tip started for this object
        Vec3r tip_start = xpbd_obj.mesh()->vertex(tip_vertex);
        _beams_tip_start.push_back(tip_start);
        
        // add logging variables if logging enabled - z-deflection, constraint residual
        if (_logger)
        {
            std::string z_deflection_var_name  = xpbd_obj.name() + "_Z-deflection[m]";
            std::string constraint_res_var_name = xpbd_obj.name() + "_Constraint-Residual";
            _logger->addOutput(z_deflection_var_name, [xpbd_obj, tip_vertex, tip_start](){
                return tip_start[2] - xpbd_obj.mesh()->vertex(tip_vertex)[2];
            });
            _logger->addOutput(constraint_res_var_name, [xpbd_obj](){
                return xpbd_obj.lastConstraintResidual().norm();
            });
        }
    }
    
    if (_graphics_scene)
        _graphics_scene->setCameraOrthographic();

    
}

} // namespace Sim