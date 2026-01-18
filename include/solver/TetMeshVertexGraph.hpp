#pragma once

#include "solver/GraphColoring.hpp"
#include "geometry/TetMesh.hpp"
#include <memory>

namespace Solver {

/**
 * @brief Build vertex connectivity graph from tetrahedral mesh
 * 
 * Two vertices are connected (have an edge) if they share at least one tetrahedron.
 * This graph is used for VBD Gauss-Seidel coloring to determine which vertices
 * can be updated in parallel.
 */
class TetMeshVertexGraph {
public:
    /**
     * @brief Build vertex graph from tetrahedral mesh
     * @param mesh The tetrahedral mesh
     * @return Graph structure representing vertex connectivity
     */
    static Graph buildFromMesh(Geometry::TetMesh& mesh);
    
    /**
     * @brief Perform graph coloring on mesh vertices
     * @param mesh The tetrahedral mesh
     * @return Unique pointer to graph coloring result
     */
    static std::unique_ptr<GraphColoring> colorMesh(Geometry::TetMesh& mesh);
};

} // namespace Solver
