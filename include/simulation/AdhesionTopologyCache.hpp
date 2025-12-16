#ifndef __ADHESION_TOPOLOGY_CACHE_HPP
#define __ADHESION_TOPOLOGY_CACHE_HPP

#include "common/types.hpp"
#include <vector>
#include <string>
#include <fstream>

namespace Sim
{

/** Pre-computed adhesion constraint topology
 * Stores vertex-triangle pairs that should form adhesion bonds
 * This avoids expensive BVH queries every simulation startup
 */
struct AdhesionPair {
    int vertex_obj_idx;      // Index of vertex object (e.g., 0 for Cube1)
    int triangle_obj_idx;    // Index of triangle object (e.g., 1 for Cube2)
    int vertex_id;           // Vertex index in vertex object
    int tri_v1, tri_v2, tri_v3;  // Triangle vertex indices
    Real initial_distance;   // Distance at creation time (for reference)
};

class AdhesionTopologyCache
{
public:
    /** Load pre-computed topology from file
     * @param filename - path to topology file
     * @return true if loaded successfully
     */
    static bool load(const std::string& filename, std::vector<AdhesionPair>& pairs) {
        std::ifstream file(filename);
        if (!file.is_open()) return false;
        
        pairs.clear();
        std::string line;
        
        // Skip header
        std::getline(file, line);
        
        while (std::getline(file, line)) {
            AdhesionPair pair;
            if (sscanf(line.c_str(), "%d,%d,%d,%d,%d,%d,%lf",
                      &pair.vertex_obj_idx, &pair.triangle_obj_idx,
                      &pair.vertex_id, &pair.tri_v1, &pair.tri_v2, &pair.tri_v3,
                      &pair.initial_distance) == 7) {
                pairs.push_back(pair);
            }
        }
        
        return !pairs.empty();
    }
    
    /** Save computed topology to file for reuse
     * @param filename - path to save topology
     * @param pairs - topology pairs to save
     */
    static void save(const std::string& filename, const std::vector<AdhesionPair>& pairs) {
        std::ofstream file(filename);
        if (!file.is_open()) return;
        
        // Write header
        file << "vertex_obj,triangle_obj,vertex_id,tri_v1,tri_v2,tri_v3,initial_distance\n";
        
        // Write data
        for (const auto& pair : pairs) {
            file << pair.vertex_obj_idx << ","
                 << pair.triangle_obj_idx << ","
                 << pair.vertex_id << ","
                 << pair.tri_v1 << ","
                 << pair.tri_v2 << ","
                 << pair.tri_v3 << ","
                 << pair.initial_distance << "\n";
        }
    }
};

} // namespace Sim

#endif // __ADHESION_TOPOLOGY_CACHE_HPP
