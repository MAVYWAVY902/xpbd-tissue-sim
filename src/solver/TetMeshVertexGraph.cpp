#include "solver/TetMeshVertexGraph.hpp"
#include <set>

namespace Solver {

Graph TetMeshVertexGraph::buildFromMesh(Geometry::TetMesh& mesh) {
    Graph graph;
    graph.numNodes = mesh.numVertices();
    
    // Use a set to avoid duplicate edges
    std::set<Graph::Edge> edgeSet;
    
    // Debug: Let's analyze the mesh structure first
    std::cout << "[Mesh Analysis] Vertices: " << mesh.numVertices() 
              << ", Tetrahedra: " << mesh.numElements() << std::endl;
    
    // Build edge-based adjacency (not all vertices in same tet)
    // We'll create a proper edge-vertex graph by considering only mesh edges
    // First, collect all unique edges from all tetrahedra
    std::set<std::pair<int,int>> uniqueEdges;
    
    // Count degree distribution for analysis
    std::vector<int> degrees(mesh.numVertices(), 0);
    
    for (int tetIdx = 0; tetIdx < mesh.numElements(); tetIdx++) {
        const auto& tet = mesh.element(tetIdx);
        
        // Each tetrahedron has exactly 6 edges: (0,1), (0,2), (0,3), (1,2), (1,3), (2,3)
        std::vector<std::pair<int,int>> tetEdges = {
            {tet[0], tet[1]}, {tet[0], tet[2]}, {tet[0], tet[3]},
            {tet[1], tet[2]}, {tet[1], tet[3]}, {tet[2], tet[3]}
        };
        
        for (auto& edge : tetEdges) {
            int v1 = edge.first;
            int v2 = edge.second;
            
            // Ensure v1 < v2 for consistent representation
            if (v1 > v2) {
                std::swap(v1, v2);
            }
            
            // Only add if it's a new edge
            if (uniqueEdges.find({v1, v2}) == uniqueEdges.end()) {
                uniqueEdges.insert({v1, v2});
                degrees[v1]++;
                degrees[v2]++;
            }
        }
    }
    
    // Analyze degree distribution
    int minDegree = *std::min_element(degrees.begin(), degrees.end());
    int maxDegree = *std::max_element(degrees.begin(), degrees.end());
    double avgDegree = 0;
    for (int d : degrees) avgDegree += d;
    avgDegree /= degrees.size();
    
    std::cout << "[Degree Analysis] Min: " << minDegree 
              << ", Max: " << maxDegree 
              << ", Avg: " << avgDegree << std::endl;
    
    // Count vertices with high degrees (potential problematic vertices)
    int highDegreeVertices = 0;
    for (int d : degrees) {
        if (d > 20) highDegreeVertices++;
    }
    std::cout << "[High Degree Vertices] Count (>20): " << highDegreeVertices << std::endl;
    
    // Now build the graph adjacency based on shared edges only
    for (const auto& edge : uniqueEdges) {
        edgeSet.insert({edge.first, edge.second});
    }
    
    // Convert set to vector
    graph.edges.reserve(edgeSet.size());
    for (const auto& edge : edgeSet) {
        graph.edges.push_back(edge);
    }
    
    // Build adjacency list
    graph.buildAdjacencyList();
    
    std::cout << "[Graph Built] Total edges: " << graph.edges.size() << std::endl;
    
    return graph;
}

std::unique_ptr<GraphColoring> TetMeshVertexGraph::colorMesh(Geometry::TetMesh& mesh) {
    // Build vertex connectivity graph
    Graph graph = buildFromMesh(mesh);
    
    // Perform ordered greedy coloring
    auto coloring = std::make_unique<OrderedGreedy>(graph);
    coloring->color();
    coloring->convertToCategories();
    
    return coloring;
}

} // namespace Solver
