#ifndef __SIMULATION_STATE_RECORDER_HPP
#define __SIMULATION_STATE_RECORDER_HPP

#include "common/types.hpp"
#include <string>
#include <vector>
#include <fstream>
#include <memory>

namespace Sim
{

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

    struct DeformationState
    {
        int element_id;
        Real volumetric_strain;
        Real deviatoric_strain;
        Real total_strain_energy;
        Vec3r principal_strains;  // 主应变
    };

    struct FrameSnapshot
    {
        Real time;
        int frame_number;
        
        // Vertex data
        std::vector<Vec3r> vertex_positions;
        std::vector<Vec3r> vertex_velocities;
        
        // Adhesion data
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
