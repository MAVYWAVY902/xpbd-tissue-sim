#pragma once

#include <vector>
#include <array>
#include <set>
#include <string>

namespace Solver {

/**
 * @brief Graph data structure for graph coloring
 * Represents a graph with nodes and edges
 */
struct Graph {
    size_t numNodes;
    
    using Edge = std::array<int, 2>;
    std::vector<Edge> edges;
    
    // Adjacency list representation (node -> neighbors)
    std::vector<std::vector<int>> adjacencyList;
    
    /**
     * @brief Build adjacency list from edges
     */
    void buildAdjacencyList() {
        adjacencyList.clear();
        adjacencyList.resize(numNodes);
        
        for (const auto& edge : edges) {
            adjacencyList[edge[0]].push_back(edge[1]);
            adjacencyList[edge[1]].push_back(edge[0]);
        }
    }
};

/**
 * @brief Base class for graph coloring algorithms
 */
class GraphColoring {
protected:
    std::vector<std::vector<int>> _graph;  // Adjacency list
    std::vector<int> _colors;              // Color for each node (-1 = uncolored)
    std::vector<std::vector<int>> _categories; // Vertices grouped by color
    
public:
    GraphColoring() = default;
    
    /**
     * @brief Construct from Graph structure
     */
    explicit GraphColoring(const Graph& graph);
    
    /**
     * @brief Construct from adjacency list
     */
    explicit GraphColoring(const std::vector<std::vector<int>>& adjacencyList);
    
    virtual ~GraphColoring() = default;
    
    /**
     * @brief Perform graph coloring (to be implemented by derived classes)
     * @return Reference to color assignment
     */
    virtual std::vector<int>& color() = 0;
    
    /**
     * @brief Get number of colors used
     */
    int getNumColors() const;
    
    /**
     * @brief Check if coloring is valid (no adjacent nodes have same color)
     */
    bool isValid() const;
    
    /**
     * @brief Convert color assignment to categories (groups of vertices with same color)
     */
    void convertToCategories();
    
    /**
     * @brief Get the color categories
     * @return Vector of vectors, where categories[i] contains all vertices with color i
     */
    const std::vector<std::vector<int>>& getCategories() const { return _categories; }
    
    /**
     * @brief Get color for a specific node
     */
    int getColor(int node) const { return _colors[node]; }
    
    /**
     * @brief Get number of nodes
     */
    size_t size() const { return _graph.size(); }
    
    /**
     * @brief Print coloring statistics
     */
    void printStats() const;
    
    // Color balancing methods (from Gaia)
    float findLargestSmallestCategories(int& biggestCategory, int& smallestCategory);
    int findChangableNodeInCategory(int sourceColor, int destinationColor);
    void changeColor(int sourceColor, int categoryId, int destinationColor);
    bool changable(int node, int destinationColor);
    void balanceColoredCategories(float goalMaxMinRatio = 1.5);
};

/**
 * @brief Ordered Greedy graph coloring algorithm
 * 
 * Based on: Ton-That, Quoc-Minh, et al. "Parallel block Neo-Hookean XPBD 
 * using graph clustering." Computers & Graphics 110 (2023): 1-10.
 * 
 * Algorithm:
 * 1. Start with uncolored graph
 * 2. Repeatedly pick node with minimum degree
 * 3. Assign smallest available color (not used by neighbors)
 * 4. Remove node and update degrees
 */
class OrderedGreedy : public GraphColoring {
private:
    std::vector<int> _degrees;    // Current degree of each node
    std::vector<bool> _colored;   // Whether node is colored
    
    /**
     * @brief Find next node to color (minimum degree)
     */
    int nextNode();
    
    /**
     * @brief Mark node as colored and reduce neighbors' degrees
     */
    void reduceDegree(int node);
    
public:
    OrderedGreedy(const Graph& graph) : GraphColoring(graph) {}
    OrderedGreedy(const std::vector<std::vector<int>>& graph) : GraphColoring(graph) {}
    
    /**
     * @brief Perform ordered greedy coloring
     */
    std::vector<int>& color() override;
};

} // namespace Solver
