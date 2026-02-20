/**
 * @file benchmark_graph_coloring.cpp
 * @brief Benchmark graph coloring performance on real brain/tumor meshes
 * 
 * This benchmark loads your actual mesh files and:
 * 1. Creates mock constraint projectors (one per tet)
 * 2. Tests graph coloring algorithm
 * 3. Simulates serial vs parallel constraint solving
 * 4. Reports speedup and recommends whether to integrate
 */

#include "geometry/TetMesh.hpp"
#include "utils/MeshUtils.hpp"
#include "solver/xpbd_solver/GraphColoring.hpp"

#include <gmsh.h>

#include <iostream>
#include <chrono>
#include <iomanip>
#include <vector>
#include <cmath>
#include <algorithm>

#ifdef _OPENMP
#include <omp.h>
#endif

/**
 * @brief Mock projector that simulates constraint projections
 */
class MockProjector
{
public:
    MockProjector(const std::vector<int>& tet_vertex_indices)
        : _vertex_indices(tet_vertex_indices)
    {}
    
    // Simulate constraint projection computational work
    void project() const {
        volatile double sum = 0;
        for (int i = 0; i < 50; i++) {
            sum += std::sin(i * 0.1) * std::cos(i * 0.1);
        }
    }
    
    bool isValid() const { return true; }
    
    // For graph coloring compatibility
    struct MockConstraint {
        struct Position {
            int index;
        };
        std::vector<Position> _positions;
        const std::vector<Position>& positions() const { return _positions; }
    };
    
    const MockConstraint& constraint() const {
        // Cache constraint to avoid dangling reference
        if (!_cached_constraint_valid) {
            _cached_constraint._positions.clear();
            for (int idx : _vertex_indices) {
                _cached_constraint._positions.push_back({idx});
            }
            _cached_constraint_valid = true;
        }
        return _cached_constraint;
    }

    // Direct positions() accessor required by GraphColoring::colorConstraints
    const std::vector<MockConstraint::Position>& positions() const {
        return constraint().positions();
    }
    
private:
    std::vector<int> _vertex_indices;
    mutable MockConstraint _cached_constraint;
    mutable bool _cached_constraint_valid = false;
};

/**
 * @brief Create mock projectors from mesh tets
 */
std::vector<MockProjector> createMockProjectors(const Geometry::TetMesh& mesh)
{
    std::vector<MockProjector> projectors;
    projectors.reserve(mesh.numElements());
    
    for (int tet_id = 0; tet_id < mesh.numElements(); tet_id++) {
        const auto& elem = mesh.element(tet_id);
        std::vector<int> indices = {
            elem[0], elem[1], 
            elem[2], elem[3]
        };
        projectors.emplace_back(indices);
    }
    
    return projectors;
}

/**
 * @brief Benchmark serial solving (Gauss-Seidel)
 */
double benchmarkSerial(const std::vector<MockProjector>& projectors, int num_iterations)
{
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int iter = 0; iter < num_iterations; iter++) {
        for (const auto& proj : projectors) {
            proj.project();
        }
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::milli>(end - start).count();
}

/**
 * @brief Benchmark colored parallel solving
 */
double benchmarkColoredParallel(
    const std::vector<MockProjector>& projectors,
    const Solver::GraphColoring::ColoringResult& coloring,
    int num_iterations)
{
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int iter = 0; iter < num_iterations; iter++) {
        // Process each color serially
        for (int color = 0; color < coloring.num_colors; color++) {
            const auto& color_group = coloring.color_groups[color];
            
            // Process constraints in this color in PARALLEL
#ifdef _OPENMP
            #pragma omp parallel for schedule(dynamic, 16)
#endif
            for (size_t i = 0; i < color_group.size(); i++) {
                int proj_idx = color_group[i];
                projectors[proj_idx].project();
            }
        }
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::milli>(end - start).count();
}

/**
 * @brief Print coloring statistics
 */
void printColoringStats(const Solver::GraphColoring::ColoringResult& coloring, int num_threads)
{
    int total_constraints = 0;
    int max_color_size = 0;
    int min_color_size = INT_MAX;
    
    for (const auto& group : coloring.color_groups) {
        int size = group.size();
        total_constraints += size;
        max_color_size = std::max(max_color_size, size);
        min_color_size = std::min(min_color_size, size);
    }
    
    double avg_color_size = static_cast<double>(total_constraints) / coloring.num_colors;
    double parallelization_eff = avg_color_size / max_color_size;
    double theoretical_speedup = std::min(
        static_cast<double>(num_threads),
        parallelization_eff * num_threads
    );
    
    std::cout << "\n========================================\n";
    std::cout << "GRAPH COLORING ANALYSIS\n";
    std::cout << "========================================\n";
    std::cout << "Colors found:              " << coloring.num_colors << "\n";
    std::cout << "Constraints per color:\n";
    std::cout << "  Average:                 " << std::fixed << std::setprecision(1) 
              << avg_color_size << "\n";
    std::cout << "  Min:                     " << min_color_size << "\n";
    std::cout << "  Max:                     " << max_color_size << "\n";
    std::cout << "Parallelization eff:       " << std::fixed << std::setprecision(1)
              << (parallelization_eff * 100) << "%\n";
    std::cout << "Theoretical speedup:       " << std::fixed << std::setprecision(2)
              << theoretical_speedup << "x (with " << num_threads << " threads)\n";
}

int main(int argc, char** argv)
{
    std::cout << "========================================\n";
    std::cout << " XPBD GRAPH COLORING BENCHMARK\n";
    std::cout << "========================================\n\n";
    
    // Configuration
    std::string mesh_path;
    int num_iterations = 50;  // Number of constraint iterations to benchmark
    
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <mesh.msh> [num_iterations]\n\n";
        std::cout << "Available meshes (.msh files):\n";
        std::cout << "  ../resource/tumor_fixed/tumor_squeezed_uv03.msh\n";
        std::cout << "  ../resource/brain_fixed/brain_MOD3_uv.msh\n\n";
        std::cout << "Using default: brain mesh\n";
        mesh_path = "../resource/brain_fixed/brain_MOD3_uv.msh";
    } else {
        mesh_path = argv[1];
    }
    
    if (argc >= 3) {
        num_iterations = std::atoi(argv[2]);
    }
    
    // Get OpenMP status
    int num_threads = 1;
#ifdef _OPENMP
    #pragma omp parallel
    {
        #pragma omp single
        {
            num_threads = omp_get_num_threads();
        }
    }
    std::cout << "OpenMP:      ENABLED (" << num_threads << " threads)\n";
#else
    std::cout << "OpenMP:      DISABLED (running serially)\n";
#endif
    
    std::cout << "Mesh file:   " << mesh_path << "\n";
    std::cout << "Iterations:  " << num_iterations << "\n\n";
    
    // Initialize Gmsh (required for loading .msh files)
    gmsh::initialize();
    
    // Load mesh
    std::cout << "Loading mesh...\n";
    
    // Convert .obj to .msh path if needed
    std::string msh_path = mesh_path;
    if (mesh_path.length() >= 4 && mesh_path.substr(mesh_path.length() - 4) == ".obj") {
        msh_path = mesh_path.substr(0, mesh_path.length() - 4) + ".msh";
    }
    
    Geometry::TetMesh mesh = MeshUtils::loadTetMeshFromGmshFile(msh_path);
    
    std::cout << "  Vertices: " << mesh.numVertices() << "\n";
    std::cout << "  Tets:     " << mesh.numElements() << "\n";
    std::cout << "  Faces:    " << mesh.numFaces() << "\n\n";
    
    // Create mock projectors (one per tet)
    std::cout << "Creating mock constraint projectors...\n";
    auto projectors = createMockProjectors(mesh);
    std::cout << "  Created " << projectors.size() << " projectors\n\n";
    
    // Perform graph coloring
    std::cout << "Running graph coloring algorithm...\n";
    auto coloring_start = std::chrono::high_resolution_clock::now();
    auto coloring = Solver::GraphColoring::colorConstraints(projectors);
    auto coloring_end = std::chrono::high_resolution_clock::now();
    double coloring_time = std::chrono::duration<double, std::milli>(coloring_end - coloring_start).count();
    
    std::cout << "  Coloring time: " << std::fixed << std::setprecision(2) 
              << coloring_time << " ms\n";
    
    printColoringStats(coloring, num_threads);
    
    // Benchmark 1: Serial (Gauss-Seidel)
    std::cout << "\n========================================\n";
    std::cout << "BENCHMARK 1: Serial Gauss-Seidel\n";
    std::cout << "========================================\n";
    std::cout << "Running " << num_iterations << " iterations...\n";
    
    double serial_time = benchmarkSerial(projectors, num_iterations);
    double serial_time_per_iter = serial_time / num_iterations;
    
    std::cout << "  Total time:       " << std::fixed << std::setprecision(2) 
              << serial_time << " ms\n";
    std::cout << "  Time per iter:    " << std::fixed << std::setprecision(3)
              << serial_time_per_iter << " ms\n";
    
    // Benchmark 2: Colored Parallel (Colored Gauss-Seidel)
    std::cout << "\n========================================\n";
    std::cout << "BENCHMARK 2: Colored Gauss-Seidel\n";
    std::cout << "========================================\n";
    std::cout << "Running " << num_iterations << " iterations...\n";
    
    double parallel_time = benchmarkColoredParallel(projectors, coloring, num_iterations);
    double parallel_time_per_iter = parallel_time / num_iterations;
    
    std::cout << "  Total time:       " << std::fixed << std::setprecision(2)
              << parallel_time << " ms\n";
    std::cout << "  Time per iter:    " << std::fixed << std::setprecision(3)
              << parallel_time_per_iter << " ms\n";
    
    // Compare results
    std::cout << "\n========================================\n";
    std::cout << "PERFORMANCE COMPARISON\n";
    std::cout << "========================================\n";
    
    double speedup = serial_time / parallel_time;
    double time_saved_per_iter = serial_time_per_iter - parallel_time_per_iter;
    double time_saved_per_timestep = time_saved_per_iter * 5;  // Assuming 5 iters per timestep
    
    std::cout << "Actual speedup:            " << std::fixed << std::setprecision(2)
              << speedup << "x\n";
    std::cout << "Time saved per iteration:  " << std::fixed << std::setprecision(3)
              << time_saved_per_iter << " ms\n";
    std::cout << "Time saved per timestep:   " << std::fixed << std::setprecision(3)
              << time_saved_per_timestep << " ms (assuming 5 solver iters)\n";
    
    // Calculate FPS improvement (assuming 60 Hz target)
    double serial_fps = 1000.0 / (serial_time_per_iter * 5);
    double parallel_fps = 1000.0 / (parallel_time_per_iter * 5);
    
    std::cout << "\nFrame rate (with 5 iters/timestep):\n";
    std::cout << "  Serial:                  " << std::fixed << std::setprecision(1)
              << serial_fps << " Hz\n";
    std::cout << "  Parallel:                " << std::fixed << std::setprecision(1)
              << parallel_fps << " Hz\n";
    std::cout << "  Improvement:             +" << std::fixed << std::setprecision(1)
              << (parallel_fps - serial_fps) << " Hz\n";
    
    // Final recommendation
    std::cout << "\n========================================\n";
    std::cout << "RECOMMENDATION\n";
    std::cout << "========================================\n";
    
    if (speedup >= 2.5) {
        std::cout << "✓ EXCELLENT speedup (" << std::fixed << std::setprecision(1) 
                  << speedup << "x)!\n";
        std::cout << "\n  STRONGLY RECOMMEND integrating graph coloring\n";
        std::cout << "  into your production XPBD framework.\n";
    } else if (speedup >= 1.5) {
        std::cout << "✓ GOOD speedup (" << std::fixed << std::setprecision(1)
                  << speedup << "x)\n";
        std::cout << "\n  RECOMMEND integrating graph coloring.\n";
        std::cout << "  Should provide noticeable performance improvement.\n";
    } else if (speedup >= 1.2) {
        std::cout << "⚠ MODERATE speedup (" << std::fixed << std::setprecision(1)
                  << speedup << "x)\n";
        std::cout << "\n  CONSIDER integration based on your priorities.\n";
        std::cout << "  May be worth it if you need every bit of performance.\n";
    } else {
        std::cout << "✗ MINIMAL speedup (" << std::fixed << std::setprecision(1)
                  << speedup << "x)\n";
        std::cout << "\n  NOT RECOMMENDED for integration.\n";
        std::cout << "  The complexity overhead may not be worth it.\n";
    }
    
    if (num_threads == 1) {
        std::cout << "\n⚠ NOTE: OpenMP is using only 1 thread!\n";
        std::cout << "  For better speedup, enable multi-threading or compile with OpenMP.\n";
    }
    
    std::cout << "\n========================================\n";
    // Cleanup Gmsh
    gmsh::finalize();
    
    std::cout << "Benchmark Complete!\n";
    std::cout << "========================================\n";
    
    return 0;
}
