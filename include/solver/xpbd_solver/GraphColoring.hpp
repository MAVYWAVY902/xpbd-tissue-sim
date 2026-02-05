#ifndef __GRAPH_COLORING_HPP
#define __GRAPH_COLORING_HPP

#include <vector>
#include <set>
#include <unordered_set>
#include <algorithm>
#include <iostream>

#include "solver/constraint/Constraint.hpp"
#include "solver/xpbd_projector/ConstraintProjector.hpp"

namespace Solver
{

/**
 * @brief Graph coloring utility for parallelizing constraint solving
 * 
 * Uses greedy coloring algorithm to partition constraints into independent sets.
 * Constraints with the same color don't share vertices and can be solved in parallel.
 */
class GraphColoring
{
public:
    /**
     * @brief Result of graph coloring
     * 
     * Each color group contains indices of constraints that can be solved in parallel
     */
    struct ColoringResult
    {
        std::vector<std::vector<int>> color_groups;  // color_groups[color] = [constraint_indices]
        int num_colors;
        
        void print() const
        {
            std::cout << "[Graph Coloring] Total colors: " << num_colors << "\n";
            for (int c = 0; c < num_colors; c++) {
                std::cout << "  Color " << c << ": " << color_groups[c].size() << " constraints\n";
            }
        }
    };
    
    /**
     * @brief Color constraints by detecting vertex conflicts
     * 
     * @tparam ConstraintContainer Type of constraint container
     * @param constraints Container of constraints to color
     * @return ColoringResult with grouped constraint indices
     */
    template<typename ConstraintContainer>
    static ColoringResult colorConstraints(const ConstraintContainer& constraints)
    {
        const int num_constraints = constraints.size();
        
        std::cout << "[Graph Coloring] Coloring " << num_constraints << " constraints...\n";
        
        // Step 1: Build conflict graph
        std::vector<std::set<int>> conflict_graph(num_constraints);
        buildConflictGraph(constraints, conflict_graph);
        
        // Step 2: Greedy coloring
        std::vector<int> colors(num_constraints, -1);
        int max_color = greedyColoring(conflict_graph, colors);
        
        // Step 3: Group by color
        ColoringResult result;
        result.num_colors = max_color + 1;
        result.color_groups.resize(result.num_colors);
        
        for (int i = 0; i < num_constraints; i++) {
            if (colors[i] >= 0) {
                result.color_groups[colors[i]].push_back(i);
            }
        }
        
        result.print();
        return result;
    }

private:
    /**
     * @brief Build conflict graph - two constraints conflict if they share vertices
     */
    template<typename ConstraintContainer>
    static void buildConflictGraph(
        const ConstraintContainer& constraints,
        std::vector<std::set<int>>& conflict_graph)
    {
        const int num_constraints = constraints.size();
        
        // For each pair of constraints
        for (int i = 0; i < num_constraints; i++) {
            for (int j = i + 1; j < num_constraints; j++) {
                
                // Check if they share vertices
                if (constraintsShareVertices(constraints[i], constraints[j])) {
                    conflict_graph[i].insert(j);
                    conflict_graph[j].insert(i);
                }
            }
        }
    }
    
    /**
     * @brief Check if two constraints share any vertices
     */
    template<typename Constraint1, typename Constraint2>
    static bool constraintsShareVertices(const Constraint1& c1, const Constraint2& c2)
    {
        // Get position indices from both constraints
        const auto& positions1 = c1.constraint().positions();
        const auto& positions2 = c2.constraint().positions();
        
        // Check for shared vertex indices
        for (const auto& pos1 : positions1) {
            for (const auto& pos2 : positions2) {
                if (pos1.index == pos2.index) {
                    return true;  // Conflict found!
                }
            }
        }
        
        return false;  // No conflict
    }
    
    /**
     * @brief Greedy coloring algorithm
     * 
     * @param conflict_graph Adjacency list of conflicts
     * @param colors Output color assignment
     * @return Maximum color used
     */
    static int greedyColoring(
        const std::vector<std::set<int>>& conflict_graph,
        std::vector<int>& colors)
    {
        const int num_constraints = conflict_graph.size();
        int max_color = 0;
        
        for (int constraint_id = 0; constraint_id < num_constraints; constraint_id++) {
            
            // Find colors used by neighbors
            std::unordered_set<int> neighbor_colors;
            for (int neighbor_id : conflict_graph[constraint_id]) {
                if (colors[neighbor_id] != -1) {
                    neighbor_colors.insert(colors[neighbor_id]);
                }
            }
            
            // Assign smallest available color
            int color = 0;
            while (neighbor_colors.count(color) > 0) {
                color++;
            }
            
            colors[constraint_id] = color;
            max_color = std::max(max_color, color);
        }
        
        return max_color;
    }
};

} // namespace Solver

#endif // __GRAPH_COLORING_HPP
