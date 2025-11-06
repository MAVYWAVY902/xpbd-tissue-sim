#include "simulation/HangingSimulation.hpp"
#include <cstdio>

namespace Sim
{

HangingSimulation::HangingSimulation(const Config::HangingSimulationConfig* config)
    : Simulation(config)
{
    _fix_max_z = config->fixMaxZ();
}

void HangingSimulation::setup()
{
    Simulation::setup();

    if (_fix_max_z)
    {
        // Fix top vertices for both types of XPBD mesh objects
        std::vector<std::unique_ptr<Sim::XPBDMeshObject_Base>>& xpbd_mesh_objs = _objects.template get<std::unique_ptr<Sim::XPBDMeshObject_Base>>();
        std::vector<std::unique_ptr<Sim::FirstOrderXPBDMeshObject_Base>>& fo_xpbd_mesh_objs = _objects.template get<std::unique_ptr<Sim::FirstOrderXPBDMeshObject_Base>>();
        
        auto fix_top_vertices = [&](auto& mesh_objs) {
            for (auto& obj : mesh_objs)
            {
                // get max z coordinate of the object's mesh (top vertices)
                Vec3r max_bbox_point = obj->mesh()->boundingBox().max;
                std::vector<int> vertices_to_fix = obj->mesh()->getVerticesWithZ(max_bbox_point[2]);
                for (const auto& v : vertices_to_fix)
                {
                    obj->fixVertex(v);
                }
                
                printf("HangingSimulation: Fixed %zu top vertices at z=%.3f for object '%s'\n", 
                       vertices_to_fix.size(), max_bbox_point[2], obj->name().c_str());
            }
        };
        
        fix_top_vertices(xpbd_mesh_objs);
        fix_top_vertices(fo_xpbd_mesh_objs);
    }
}

} // namespace Sim