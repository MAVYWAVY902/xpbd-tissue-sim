#ifndef __GRAPH_COLORING_HPP
#define __GRAPH_COLORING_HPP

#include <vector>
#include <set>
#include <unordered_set>
#include <algorithm>
#include <iostream>
#include <type_traits>

#include "solver/constraint/Constraint.hpp"
#include "solver/xpbd_projector/ConstraintProjector.hpp"
#include "solver/xpbd_projector/RigidBodyConstraintProjector.hpp"

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
    
private:
    /**
     * @brief SFINAE helper to detect for_each_element method
     */
    template<typename T>
    static auto has_for_each_element(int) -> decltype(std::declval<T>().for_each_element(std::declval<std::function<void(int)>>()), std::true_type{});
    
    template<typename T>
    static std::false_type has_for_each_element(...);
    
    /**
     * @brief Extract vertex indices - for containers with for_each_element (VariadicVectorContainer)
     */
    template<typename Container>
    static typename std::enable_if<decltype(has_for_each_element<Container>(0))::value>::type
    extractVertexIndices(const Container& constraints, std::vector<std::vector<int>>& vertex_indices)
    {
        constraints.for_each_element([&](const auto& projector) {
            if (projector.isValid()) {
                std::vector<int> indices;
                const auto& positions = getProjectorPositions_helper(projector);
                for (const auto& pos : positions) {
                    indices.push_back(pos.index);
                }
                vertex_indices.push_back(std::move(indices));
            }
        });
    }
    
    /**
     * @brief Extract vertex indices - for std::vector
     */
    template<typename Container>
    static typename std::enable_if<!decltype(has_for_each_element<Container>(0))::value>::type
    extractVertexIndices(const Container& constraints, std::vector<std::vector<int>>& vertex_indices)
    {
        for (const auto& projector : constraints) {
            if (projector.isValid()) {
                std::vector<int> indices;
                const auto& positions = getProjectorPositions_helper(projector);
                for (const auto& pos : positions) {
                    indices.push_back(pos.index);
                }
                vertex_indices.push_back(std::move(indices));
            }
        }
    }

public:
    
    /**
     * @brief Helper to get positions from different projector types
     * CombinedConstraintProjector has positions() directly, ConstraintProjector has constraint()
     */
    template<typename ProjectorType>
    static auto getProjectorPositions(const ProjectorType& projector) 
        -> decltype(projector.positions())
    {
        return projector.positions();
    }
    
    template<bool IsFirstOrder, typename Constraint>
    static const auto& getProjectorPositions(const ConstraintProjector<IsFirstOrder, Constraint>& projector)
    {
        return projector.constraint().get().positions();
    }
    
    template<bool IsFirstOrder, typename... Constraints>
    static const auto& getProjectorPositions(const RigidBodyConstraintProjector<IsFirstOrder, Constraints...>& projector)
    {
        return projector.positions();
    }
    
    // Generic fallback for test mocks and other types with constraint().positions()
    template<typename ProjectorType>
    static auto getProjectorPositions_fallback(const ProjectorType& projector, int)
        -> decltype(projector.constraint().positions())
    {
        return projector.constraint().positions();
    }
    
    template<typename ProjectorType>
    static auto getProjectorPositions_fallback(const ProjectorType& projector, long)
        -> decltype(getProjectorPositions(projector))
    {
        return getProjectorPositions(projector);
    }
    
    // Wrapper to try fallback if primary overloads don't match
    template<typename ProjectorType>
    static auto getProjectorPositions_helper(const ProjectorType& projector)
        -> decltype(getProjectorPositions_fallback(projector, 0))
    {
        return getProjectorPositions_fallback(projector, 0);
    }
    
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
        // First, collect all constraint vertex indices into vectors for indexed access
        std::vector<std::vector<int>> constraint_vertex_indices;
        
        // Handle both std::vector and VariadicVectorContainer
        extractVertexIndices(constraints, constraint_vertex_indices);
        
        const int num_constraints = constraint_vertex_indices.size();
        
        std::cout << "[Graph Coloring] Coloring " << num_constraints << " constraints...\n";
        
        if (num_constraints == 0) {
            return ColoringResult{{}, 0};
        }
        
        // Step 1: Build conflict graph
        std::vector<std::set<int>> conflict_graph(num_constraints);
        buildConflictGraph(constraint_vertex_indices, conflict_graph);
        
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
    static void buildConflictGraph(
        const std::vector<std::vector<int>>& constraint_vertex_indices,
        std::vector<std::set<int>>& conflict_graph)
    {
        const int num_constraints = constraint_vertex_indices.size();
        
        // For each pair of constraints
        for (int i = 0; i < num_constraints; i++) {
            for (int j = i + 1; j < num_constraints; j++) {
                
                // Check if they share vertices
                if (constraintsShareVertices(constraint_vertex_indices[i], 
                                            constraint_vertex_indices[j])) {
                    conflict_graph[i].insert(j);
                    conflict_graph[j].insert(i);
                }
            }
        }
    }
    
    /**
     * @brief Check if two constraints share any vertices
     */
    static bool constraintsShareVertices(
        const std::vector<int>& vertices1, 
        const std::vector<int>& vertices2)
    {
        // Check for shared vertex indices
        for (int v1 : vertices1) {
            for (int v2 : vertices2) {
                if (v1 == v2) {
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
