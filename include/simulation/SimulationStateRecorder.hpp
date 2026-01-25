#ifndef __SIMULATION_STATE_RECORDER_HPP
#define __SIMULATION_STATE_RECORDER_HPP

#include "common/types.hpp"
#include <string>
#include <vector>
#include <fstream>
#include <memory>

namespace Sim
{

// Inter-deformable adhesion state (vertex-to-triangle between two deformable objects)
struct InterDeformAdhesionState
{
    int vertex_id;           // Vertex index on object A
    int triangle_id;         // Triangle index on object B (face index)
    Vec3r vertex_position;   // Current position of vertex
    Vec3r contact_point;     // Current closest point on triangle
    Real current_distance;   // Current separation distance
    Real rest_gap;           // Rest gap d_0
    Real max_distance_seen;  // Peak stretch during this timestep
    bool is_broken;          // Whether bond is broken
    Real break_threshold;    // Distance threshold for breaking
};

// Rigid-deformable adhesion state (rigid body point-to-triangle)
struct RigidDeformAdhesionState
{
    int triangle_id;         // Triangle index on deformable object
    Vec3r rigid_point;       // Attachment point on rigid body (body coords)
    Vec3r triangle_centroid; // Centroid of triangle for visualization
    Vec3r contact_point;     // Current closest point on triangle
    Real current_distance;   // Current separation distance
    Real rest_gap;           // Rest gap d_0
    Real max_distance_seen;  // Peak stretch during this timestep
    bool is_broken;          // Whether bond is broken
    Real break_threshold;    // Distance threshold for breaking
};

// Legacy struct for backward compatibility (can be removed later)
struct AdhesionState
{
    int nerve_vertex_id;
    int tumor_face_id;
    Vec3r nerve_position;
    Vec3r tumor_contact_point;
    Real current_distance;
    Real rest_gap;
    Real max_distance_seen;  // Peak stretch
    bool is_broken;
    Real break_threshold;
};

/**
 * @brief Records simulation state snapshots for offline analysis
 * 
 * This recorder captures:
 * - Vertex positions and velocities
 * - Adhesion constraint states (active/broken)
 * - Stress/strain data
 * - Deformation metrics
 * 
 * Data is saved to disk at regular intervals for post-processing.
 */
class SimulationStateRecorder
{
public:
    struct DeformationState
    {
        int element_id;
        Real volumetric_strain;
        Real deviatoric_strain;
        Real total_strain_energy;
        Vec3r principal_strains;  // 主应变
    };

    struct MeshTopology
    {
        int vertex_offset;  // Starting index of vertices for this mesh
        int num_vertices;
        std::vector<Eigen::Vector3i> surface_triangles;  // Surface triangles (for visualization)
        std::vector<Eigen::Vector4i> tetrahedra;         // Volumetric tetrahedra (if available)
        bool has_tets;  // Whether this mesh has tetrahedral elements
    };

    struct FrameSnapshot
    {
        Real time;
        int frame_number;
        
        // Vertex data
        std::vector<Vec3r> vertex_positions;
        std::vector<Vec3r> vertex_velocities;
        
        // Adhesion constraint forces (magnitude per vertex)
        // Physical meaning: |F_adhesion| = |∇C^T · λ / dt| (1st-order) or |∇C^T · λ / dt²| (2nd-order)
        // This represents the constraint force magnitude from all adhesion constraints acting on each vertex
        std::vector<Real> vertex_adhesion_force_magnitude;
        
        // Mesh topology (connectivity information)
        std::vector<MeshTopology> mesh_topologies;
        
        // Inter-deformable adhesion data (vertex-to-face between deformables)
        std::vector<InterDeformAdhesionState> inter_deform_adhesion_states;
        
        // Rigid-deformable adhesion data (rigid point-to-face)
        std::vector<RigidDeformAdhesionState> rigid_deform_adhesion_states;
        
        // Legacy adhesion data (for backward compatibility with old analysis code)
        std::vector<AdhesionState> adhesion_states;
        
        // Deformation data (optional - can be computed offline)
        std::vector<DeformationState> deformation_states;
        
        // Events in this frame
        std::vector<int> broken_adhesion_ids;  // Which adhesions broke this frame
    };

public:
    SimulationStateRecorder(const std::string& output_folder, Real snapshot_interval);
    ~SimulationStateRecorder();

    /**
     * @brief Check if it's time to record a snapshot
     */
    bool shouldRecord(Real current_time) const;

    /**
     * @brief Record a snapshot of current simulation state
     */
    void recordSnapshot(Real time, int frame_num, const FrameSnapshot& snapshot);

    /**
     * @brief Save all recorded data to disk
     */
    void saveToFile();

    /**
     * @brief Load recorded data from disk
     */
    static std::vector<FrameSnapshot> loadFromFile(const std::string& filepath);

    /**
     * @brief Get output file path
     */
    std::string getOutputPath() const { return _output_folder + "/state_snapshots.bin"; }
    
    /**
     * @brief Get reference to recorded snapshots (for checking if empty)
     */
    const std::vector<FrameSnapshot>& getSnapshots() const { return _snapshots; }

private:
    std::string _output_folder;
    Real _snapshot_interval;
    Real _last_snapshot_time;
    
    std::vector<FrameSnapshot> _snapshots;
    
    // For efficient binary writing
    void _writeBinary(std::ofstream& file, const FrameSnapshot& snapshot);
    static FrameSnapshot _readBinary(std::ifstream& file);
};

} // namespace Sim

#endif // __SIMULATION_STATE_RECORDER_HPP
