#ifndef __GRAPH_COLORING_HPP
#define __GRAPH_COLORING_HPP

#include <vector>
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <iostream>

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
        int num_colors = 0;
        int num_projectors = 0;  ///< size of the projector vector when coloring was built

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
        result.num_colors     = max_color + 1;
        result.num_projectors = num_constraints;
        result.color_groups.resize(result.num_colors);

        for (int i = 0; i < num_constraints; i++)
            if (colors[i] >= 0)
                result.color_groups[colors[i]].push_back(i);

        result.print();
        return result;
    }

private:
    /**
     * @brief Build conflict graph using a vertex → constraints hash map.
     *
     * Calls projector.positions() directly — compatible with all real projector types
     * (ConstraintProjector, CombinedConstraintProjector, RigidBodyConstraintProjector)
     * because they all expose a public positions() method.
     *
     * Complexity: O(n × d²) where d ≈ average constraints sharing a vertex (~4-6 for
     * typical tet meshes), vs O(n²) for the naive pairwise approach.  For 60 k tets
     * this is ~1.5 M operations instead of 1.8 B.
     */
    template<typename ConstraintContainer>
    static void buildConflictGraph(
        const ConstraintContainer& constraints,
        std::vector<std::set<int>>& conflict_graph)
    {
        const int n = static_cast<int>(constraints.size());

        // Map each vertex index to every constraint that references it
        std::unordered_map<int, std::vector<int>> vertex_to_constraints;
        vertex_to_constraints.reserve(n * 4);

        for (int i = 0; i < n; i++)
            for (const auto& pos : constraints[i].positions())
                vertex_to_constraints[pos.index].push_back(i);

        // Any two constraints that share a vertex are conflicting
        for (const auto& kv : vertex_to_constraints)
            for (int a : kv.second)
                for (int b : kv.second)
                    if (a != b)
                        conflict_graph[a].insert(b);
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
