#ifndef __NERVE_TUMOR_BOND_HPP
#define __NERVE_TUMOR_BOND_HPP

#include "common/types.hpp"

namespace Solver
{

/** Represents an adhesive bond between a nerve vertex and a tumor triangle face.
 * Used for tracking bond lifecycle (creation, maintenance, breaking) in Phase 1 implementation.
 */
struct NerveTumorBond 
{
    // Bond identification
    int nerve_vertex_id;          ///< Index of the nerve vertex
    int tumor_face_id;            ///< Index of the tumor triangle face  
    int tumor_v1, tumor_v2, tumor_v3; ///< Indices of triangle vertices
    
    // Bond state
    bool active;                  ///< Whether bond is currently active
    Real creation_time;           ///< Simulation time when bond was created
    Real age;                     ///< Current age of the bond (for fatigue modeling)
    
    // Geometric parameters  
    Vec3r barycentric_coords;     ///< Barycentric coordinates [u,v,w] of attachment point on triangle
    Real target_gap;              ///< Target separation distance d_0
    
    // Bond strength parameters
    Real break_distance;          ///< Distance threshold for breaking bond
    Real max_force;               ///< Maximum force threshold for breaking (optional, for Phase 2)
    
    // Statistics (useful for debugging/analysis)
    Real max_distance_reached;    ///< Maximum separation distance reached
    Real total_energy_dissipated; ///< Cumulative energy dissipated (optional)
    
    /** Constructor for creating new bond */
    NerveTumorBond(int nerve_v, int tumor_face, int tri_v1, int tri_v2, int tri_v3,
                   const Vec3r& bary_coords, Real target_gap, Real break_dist, Real sim_time)
        : nerve_vertex_id(nerve_v)
        , tumor_face_id(tumor_face)
        , tumor_v1(tri_v1), tumor_v2(tri_v2), tumor_v3(tri_v3)
        , active(true)
        , creation_time(sim_time)
        , age(0.0)
        , barycentric_coords(bary_coords)
        , target_gap(target_gap)
        , break_distance(break_dist)
        , max_force(1e6) // large default value
        , max_distance_reached(0.0)
        , total_energy_dissipated(0.0)
    {
    }
    
    /** Update bond age */
    void updateAge(Real sim_time) {
        age = sim_time - creation_time;
    }
    
    /** Check if bond should be broken based on distance */
    bool shouldBreak(Real current_distance) const {
        return active && (current_distance > break_distance);
    }
    
    /** Break the bond */
    void breakBond() {
        active = false;
    }
    
    /** Get attachment point on tumor surface using current triangle vertex positions */
    Vec3r getAttachmentPoint(const Vec3r& tri_p1, const Vec3r& tri_p2, const Vec3r& tri_p3) const {
        return barycentric_coords[0] * tri_p1 + 
               barycentric_coords[1] * tri_p2 + 
               barycentric_coords[2] * tri_p3;
    }
};

/** Manager class for handling multiple nerve-tumor bonds */
class NerveTumorBondManager
{
private:
    std::vector<NerveTumorBond> _bonds;
    Real _current_time;
    
    // Parameters for bond creation/breaking
    Real _bond_creation_distance;   ///< Distance threshold for creating new bonds
    Real _bond_break_distance;      ///< Distance threshold for breaking bonds
    Real _default_target_gap;       ///< Default target gap for new bonds
    
public:
    NerveTumorBondManager(Real bond_creation_dist = 3e-4, 
                         Real bond_break_dist = 7e-4,
                         Real default_gap = 1e-4)
        : _current_time(0.0)
        , _bond_creation_distance(bond_creation_dist)
        , _bond_break_distance(bond_break_dist) 
        , _default_target_gap(default_gap)
    {
    }
    
    /** Update simulation time */
    void setCurrentTime(Real time) { _current_time = time; }
    
    /** Create new bond if conditions are met */
    bool tryCreateBond(int nerve_v, int tumor_face, int tri_v1, int tri_v2, int tri_v3,
                      const Vec3r& bary_coords, Real distance) {
        // Check if bond already exists for this nerve vertex
        for (const auto& bond : _bonds) {
            if (bond.active && bond.nerve_vertex_id == nerve_v) {
                return false; // Already bonded
            }
        }
        
        // Check distance threshold
        if (distance < _bond_creation_distance) {
            _bonds.emplace_back(nerve_v, tumor_face, tri_v1, tri_v2, tri_v3,
                              bary_coords, _default_target_gap, _bond_break_distance, _current_time);
            return true;
        }
        return false;
    }
    
    /** Update existing bonds and check for breaking */
    void updateBonds(std::function<Real(const NerveTumorBond&)> distance_calculator) {
        for (auto& bond : _bonds) {
            if (!bond.active) continue;
            
            bond.updateAge(_current_time);
            Real current_distance = distance_calculator(bond);
            bond.max_distance_reached = std::max(bond.max_distance_reached, current_distance);
            
            if (bond.shouldBreak(current_distance)) {
                bond.breakBond();
            }
        }
    }
    
    /** Get all active bonds */
    std::vector<NerveTumorBond> getActiveBonds() const {
        std::vector<NerveTumorBond> active_bonds;
        for (const auto& bond : _bonds) {
            if (bond.active) {
                active_bonds.push_back(bond);
            }
        }
        return active_bonds;
    }
    
    /** Get bond count statistics */
    void getBondStatistics(int& total_bonds, int& active_bonds, int& broken_bonds) const {
        total_bonds = _bonds.size();
        active_bonds = 0;
        broken_bonds = 0;
        
        for (const auto& bond : _bonds) {
            if (bond.active) active_bonds++;
            else broken_bonds++;
        }
    }
    
    /** Clear all bonds (useful for testing) */
    void clearBonds() { _bonds.clear(); }
    
    /** Set bond creation parameters */
    void setBondParameters(Real creation_dist, Real break_dist, Real target_gap) {
        _bond_creation_distance = creation_dist;
        _bond_break_distance = break_dist;
        _default_target_gap = target_gap;
    }
};

} // namespace Solver

#endif // __NERVE_TUMOR_BOND_HPP