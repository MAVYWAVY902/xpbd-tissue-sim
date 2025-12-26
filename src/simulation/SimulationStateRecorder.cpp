#include "simulation/SimulationStateRecorder.hpp"
#include <iostream>
#include <filesystem>
#include <cstring>

namespace Sim
{

SimulationStateRecorder::SimulationStateRecorder(const std::string& output_folder, Real snapshot_interval)
    : _output_folder(output_folder)
    , _snapshot_interval(snapshot_interval)
    , _last_snapshot_time(-snapshot_interval)
{
    // Create output directory if it doesn't exist
    std::filesystem::create_directories(output_folder);
    
    std::cout << "[StateRecorder] Initialized with output folder: " << output_folder 
              << ", snapshot interval: " << snapshot_interval << "s\n";
}

SimulationStateRecorder::~SimulationStateRecorder()
{
    // Auto-save on destruction
    if (!_snapshots.empty())
    {
        std::cout << "[StateRecorder] Auto-saving " << _snapshots.size() << " snapshots...\n";
        saveToFile();
    }
}

bool SimulationStateRecorder::shouldRecord(Real current_time) const
{
    return (current_time - _last_snapshot_time) >= _snapshot_interval;
}

void SimulationStateRecorder::recordSnapshot(Real time, int frame_num, const FrameSnapshot& snapshot)
{
    _snapshots.push_back(snapshot);
    _last_snapshot_time = time;
    
    std::cout << "[StateRecorder] Recorded snapshot #" << _snapshots.size() 
              << " at t=" << time << "s"
              << " (frame " << frame_num << ")"
              << " - " << snapshot.adhesion_states.size() << " adhesions";
    
    if (!snapshot.broken_adhesion_ids.empty())
    {
        std::cout << ", " << snapshot.broken_adhesion_ids.size() << " BROKE!";
    }
    std::cout << "\n";
}

void SimulationStateRecorder::saveToFile()
{
    std::string filepath = getOutputPath();
    std::ofstream file(filepath, std::ios::binary);
    
    if (!file.is_open())
    {
        std::cerr << "[StateRecorder] ERROR: Could not open file for writing: " << filepath << "\n";
        return;
    }
    
    // Write header
    uint32_t num_snapshots = _snapshots.size();
    file.write(reinterpret_cast<const char*>(&num_snapshots), sizeof(num_snapshots));
    
    // Write each snapshot
    for (const auto& snapshot : _snapshots)
    {
        _writeBinary(file, snapshot);
    }
    
    file.close();
    std::cout << "[StateRecorder] Saved " << num_snapshots << " snapshots to: " << filepath << "\n";
    
    // Also save a human-readable summary
    std::string summary_path = _output_folder + "/summary.txt";
    std::ofstream summary(summary_path);
    summary << "Simulation State Recording Summary\n";
    summary << "===================================\n";
    summary << "Total snapshots: " << num_snapshots << "\n";
    summary << "Time range: " << (_snapshots.empty() ? 0 : _snapshots.front().time) 
            << " - " << (_snapshots.empty() ? 0 : _snapshots.back().time) << " seconds\n";
    summary << "Snapshot interval: " << _snapshot_interval << " seconds\n\n";
    
    // Count total breakages
    int total_breaks = 0;
    for (const auto& snap : _snapshots)
    {
        total_breaks += snap.broken_adhesion_ids.size();
    }
    summary << "Total adhesion breakages: " << total_breaks << "\n";
    summary.close();
}

void SimulationStateRecorder::_writeBinary(std::ofstream& file, const FrameSnapshot& snapshot)
{
    // Write time and frame number
    file.write(reinterpret_cast<const char*>(&snapshot.time), sizeof(snapshot.time));
    file.write(reinterpret_cast<const char*>(&snapshot.frame_number), sizeof(snapshot.frame_number));
    
    // Write vertex positions
    uint32_t num_vertices = snapshot.vertex_positions.size();
    file.write(reinterpret_cast<const char*>(&num_vertices), sizeof(num_vertices));
    file.write(reinterpret_cast<const char*>(snapshot.vertex_positions.data()), 
               num_vertices * sizeof(Vec3r));
    
    // Write vertex velocities
    file.write(reinterpret_cast<const char*>(snapshot.vertex_velocities.data()), 
               num_vertices * sizeof(Vec3r));
    
    // Write mesh topologies
    uint32_t num_meshes = snapshot.mesh_topologies.size();
    file.write(reinterpret_cast<const char*>(&num_meshes), sizeof(num_meshes));
    
    for (const auto& topo : snapshot.mesh_topologies)
    {
        file.write(reinterpret_cast<const char*>(&topo.vertex_offset), sizeof(topo.vertex_offset));
        file.write(reinterpret_cast<const char*>(&topo.num_vertices), sizeof(topo.num_vertices));
        
        // Write surface triangles
        uint32_t num_triangles = topo.surface_triangles.size();
        file.write(reinterpret_cast<const char*>(&num_triangles), sizeof(num_triangles));
        file.write(reinterpret_cast<const char*>(topo.surface_triangles.data()), 
                   num_triangles * sizeof(Eigen::Vector3i));
        
        // Write tetrahedra
        file.write(reinterpret_cast<const char*>(&topo.has_tets), sizeof(topo.has_tets));
        uint32_t num_tets = topo.tetrahedra.size();
        file.write(reinterpret_cast<const char*>(&num_tets), sizeof(num_tets));
        if (num_tets > 0)
        {
            file.write(reinterpret_cast<const char*>(topo.tetrahedra.data()), 
                       num_tets * sizeof(Eigen::Vector4i));
        }
    }
    
    // Write adhesion states
    uint32_t num_adhesions = snapshot.adhesion_states.size();
    file.write(reinterpret_cast<const char*>(&num_adhesions), sizeof(num_adhesions));
    
    for (const auto& adhesion : snapshot.adhesion_states)
    {
        file.write(reinterpret_cast<const char*>(&adhesion.nerve_vertex_id), sizeof(adhesion.nerve_vertex_id));
        file.write(reinterpret_cast<const char*>(&adhesion.tumor_face_id), sizeof(adhesion.tumor_face_id));
        file.write(reinterpret_cast<const char*>(&adhesion.nerve_position), sizeof(adhesion.nerve_position));
        file.write(reinterpret_cast<const char*>(&adhesion.tumor_contact_point), sizeof(adhesion.tumor_contact_point));
        file.write(reinterpret_cast<const char*>(&adhesion.current_distance), sizeof(adhesion.current_distance));
        file.write(reinterpret_cast<const char*>(&adhesion.rest_gap), sizeof(adhesion.rest_gap));
        file.write(reinterpret_cast<const char*>(&adhesion.max_distance_seen), sizeof(adhesion.max_distance_seen));
        file.write(reinterpret_cast<const char*>(&adhesion.is_broken), sizeof(adhesion.is_broken));
        file.write(reinterpret_cast<const char*>(&adhesion.break_threshold), sizeof(adhesion.break_threshold));
    }
    
    // Write deformation states (optional)
    uint32_t num_deformations = snapshot.deformation_states.size();
    file.write(reinterpret_cast<const char*>(&num_deformations), sizeof(num_deformations));
    
    for (const auto& deform : snapshot.deformation_states)
    {
        file.write(reinterpret_cast<const char*>(&deform.element_id), sizeof(deform.element_id));
        file.write(reinterpret_cast<const char*>(&deform.volumetric_strain), sizeof(deform.volumetric_strain));
        file.write(reinterpret_cast<const char*>(&deform.deviatoric_strain), sizeof(deform.deviatoric_strain));
        file.write(reinterpret_cast<const char*>(&deform.total_strain_energy), sizeof(deform.total_strain_energy));
        file.write(reinterpret_cast<const char*>(&deform.principal_strains), sizeof(deform.principal_strains));
    }
    
    // Write broken adhesion IDs
    uint32_t num_broken = snapshot.broken_adhesion_ids.size();
    file.write(reinterpret_cast<const char*>(&num_broken), sizeof(num_broken));
    if (num_broken > 0)
    {
        file.write(reinterpret_cast<const char*>(snapshot.broken_adhesion_ids.data()), 
                   num_broken * sizeof(int));
    }
}

std::vector<SimulationStateRecorder::FrameSnapshot> 
SimulationStateRecorder::loadFromFile(const std::string& filepath)
{
    std::vector<FrameSnapshot> snapshots;
    std::ifstream file(filepath, std::ios::binary);
    
    if (!file.is_open())
    {
        std::cerr << "[StateRecorder] ERROR: Could not open file for reading: " << filepath << "\n";
        return snapshots;
    }
    
    // Read header
    uint32_t num_snapshots;
    file.read(reinterpret_cast<char*>(&num_snapshots), sizeof(num_snapshots));
    
    snapshots.reserve(num_snapshots);
    
    // Read each snapshot
    for (uint32_t i = 0; i < num_snapshots; ++i)
    {
        snapshots.push_back(_readBinary(file));
    }
    
    file.close();
    std::cout << "[StateRecorder] Loaded " << num_snapshots << " snapshots from: " << filepath << "\n";
    
    return snapshots;
}

SimulationStateRecorder::FrameSnapshot 
SimulationStateRecorder::_readBinary(std::ifstream& file)
{
    FrameSnapshot snapshot;
    
    // Read time and frame number
    file.read(reinterpret_cast<char*>(&snapshot.time), sizeof(snapshot.time));
    file.read(reinterpret_cast<char*>(&snapshot.frame_number), sizeof(snapshot.frame_number));
    
    // Read vertex positions
    uint32_t num_vertices;
    file.read(reinterpret_cast<char*>(&num_vertices), sizeof(num_vertices));
    snapshot.vertex_positions.resize(num_vertices);
    file.read(reinterpret_cast<char*>(snapshot.vertex_positions.data()), 
              num_vertices * sizeof(Vec3r));
    
    // Read vertex velocities
    snapshot.vertex_velocities.resize(num_vertices);
    file.read(reinterpret_cast<char*>(snapshot.vertex_velocities.data()), 
              num_vertices * sizeof(Vec3r));
    
    // Read mesh topologies
    uint32_t num_meshes;
    file.read(reinterpret_cast<char*>(&num_meshes), sizeof(num_meshes));
    snapshot.mesh_topologies.resize(num_meshes);
    
    for (auto& topo : snapshot.mesh_topologies)
    {
        file.read(reinterpret_cast<char*>(&topo.vertex_offset), sizeof(topo.vertex_offset));
        file.read(reinterpret_cast<char*>(&topo.num_vertices), sizeof(topo.num_vertices));
        
        // Read surface triangles
        uint32_t num_triangles;
        file.read(reinterpret_cast<char*>(&num_triangles), sizeof(num_triangles));
        topo.surface_triangles.resize(num_triangles);
        file.read(reinterpret_cast<char*>(topo.surface_triangles.data()), 
                  num_triangles * sizeof(Eigen::Vector3i));
        
        // Read tetrahedra
        file.read(reinterpret_cast<char*>(&topo.has_tets), sizeof(topo.has_tets));
        uint32_t num_tets;
        file.read(reinterpret_cast<char*>(&num_tets), sizeof(num_tets));
        topo.tetrahedra.resize(num_tets);
        if (num_tets > 0)
        {
            file.read(reinterpret_cast<char*>(topo.tetrahedra.data()), 
                      num_tets * sizeof(Eigen::Vector4i));
        }
    }
    
    // Read adhesion states
    uint32_t num_adhesions;
    file.read(reinterpret_cast<char*>(&num_adhesions), sizeof(num_adhesions));
    snapshot.adhesion_states.resize(num_adhesions);
    
    for (auto& adhesion : snapshot.adhesion_states)
    {
        file.read(reinterpret_cast<char*>(&adhesion.nerve_vertex_id), sizeof(adhesion.nerve_vertex_id));
        file.read(reinterpret_cast<char*>(&adhesion.tumor_face_id), sizeof(adhesion.tumor_face_id));
        file.read(reinterpret_cast<char*>(&adhesion.nerve_position), sizeof(adhesion.nerve_position));
        file.read(reinterpret_cast<char*>(&adhesion.tumor_contact_point), sizeof(adhesion.tumor_contact_point));
        file.read(reinterpret_cast<char*>(&adhesion.current_distance), sizeof(adhesion.current_distance));
        file.read(reinterpret_cast<char*>(&adhesion.rest_gap), sizeof(adhesion.rest_gap));
        file.read(reinterpret_cast<char*>(&adhesion.max_distance_seen), sizeof(adhesion.max_distance_seen));
        file.read(reinterpret_cast<char*>(&adhesion.is_broken), sizeof(adhesion.is_broken));
        file.read(reinterpret_cast<char*>(&adhesion.break_threshold), sizeof(adhesion.break_threshold));
    }
    
    // Read deformation states
    uint32_t num_deformations;
    file.read(reinterpret_cast<char*>(&num_deformations), sizeof(num_deformations));
    snapshot.deformation_states.resize(num_deformations);
    
    for (auto& deform : snapshot.deformation_states)
    {
        file.read(reinterpret_cast<char*>(&deform.element_id), sizeof(deform.element_id));
        file.read(reinterpret_cast<char*>(&deform.volumetric_strain), sizeof(deform.volumetric_strain));
        file.read(reinterpret_cast<char*>(&deform.deviatoric_strain), sizeof(deform.deviatoric_strain));
        file.read(reinterpret_cast<char*>(&deform.total_strain_energy), sizeof(deform.total_strain_energy));
        file.read(reinterpret_cast<char*>(&deform.principal_strains), sizeof(deform.principal_strains));
    }
    
    // Read broken adhesion IDs
    uint32_t num_broken;
    file.read(reinterpret_cast<char*>(&num_broken), sizeof(num_broken));
    snapshot.broken_adhesion_ids.resize(num_broken);
    if (num_broken > 0)
    {
        file.read(reinterpret_cast<char*>(snapshot.broken_adhesion_ids.data()), 
                  num_broken * sizeof(int));
    }
    
    return snapshot;
}

} // namespace Sim
