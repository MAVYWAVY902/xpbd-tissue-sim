/**
 * @file benchmark_adhesion_coloring.cpp
 * @brief Benchmark graph coloring with adhesion constraints
 * 
 * Tests realistic scenario with:
 * 1. Volumetric constraints (DeviatoricConstraint + HydrostaticConstraint)
 * 2. Inter-deform adhesion constraints (tumor-brain)
 * 3. Dynamic constraint breaking (simulating knife cutting)
 * 4. Recoloring overhead measurement
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
#include <random>

#ifdef _OPENMP
#include <omp.h>
#endif

/**
 * @brief Mock constraint types
 */
enum class ConstraintType {
    VOLUMETRIC,  // Deviatoric/Hydrostatic (per-tet)
    ADHESION     // Inter-deform adhesion (between meshes)
};

/**
 * @brief Mock constraint projector with type information
 */
class MockConstraintProjector
{
public:
    MockConstraintProjector(const std::vector<int>& vertex_indices, ConstraintType type, int mesh_id = 0)
        : _vertex_indices(vertex_indices)
        , _type(type)
        , _mesh_id(mesh_id)
        , _is_valid(true)
    {}
    
    // Simulate constraint projection work
    void project() const {
        volatile double sum = 0;
        int work_amount = (_type == ConstraintType::ADHESION) ? 30 : 50;
        for (int i = 0; i < work_amount; i++) {
            sum += std::sin(i * 0.1) * std::cos(i * 0.1);
        }
    }
    
    bool isValid() const { return _is_valid; }
    void setValid(bool valid) { _is_valid = valid; }
    
    ConstraintType type() const { return _type; }
    int meshId() const { return _mesh_id; }
    
    // For graph coloring compatibility
    struct MockConstraint {
        struct Position {
            int index;
        };
        std::vector<Position> _positions;
        const std::vector<Position>& positions() const { return _positions; }
    };
    
    const MockConstraint& constraint() const {
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
    ConstraintType _type;
    int _mesh_id;
    bool _is_valid;
    mutable MockConstraint _cached_constraint;
    mutable bool _cached_constraint_valid = false;
};

/**
 * @brief Create volumetric constraints from mesh tets
 */
std::vector<MockConstraintProjector> createVolumetricConstraints(
    const Geometry::TetMesh& mesh, int mesh_id)
{
    std::vector<MockConstraintProjector> projectors;
    projectors.reserve(mesh.numElements());
    
    for (int tet_id = 0; tet_id < mesh.numElements(); tet_id++) {
        const auto& elem = mesh.element(tet_id);
        std::vector<int> indices = {elem[0], elem[1], elem[2], elem[3]};
        
        // Adjust indices for second mesh (offset by first mesh vertex count)
        if (mesh_id > 0) {
            for (auto& idx : indices) {
                idx += 10000 * mesh_id;  // Large offset to avoid conflicts
            }
        }
        
        projectors.emplace_back(indices, ConstraintType::VOLUMETRIC, mesh_id);
    }
    
    return projectors;
}

/**
 * @brief Create mock adhesion constraints between two meshes
 */
std::vector<MockConstraintProjector> createAdhesionConstraints(
    const Geometry::TetMesh& mesh1, 
    const Geometry::TetMesh& mesh2,
    int num_adhesions)
{
    std::vector<MockConstraintProjector> projectors;
    projectors.reserve(num_adhesions);
    
    std::mt19937 rng(42);  // Fixed seed for reproducibility
    std::uniform_int_distribution<int> dist1(0, mesh1.numVertices() - 1);
    std::uniform_int_distribution<int> dist2(0, mesh2.numVertices() - 1);
    
    for (int i = 0; i < num_adhesions; i++) {
        int v1 = dist1(rng);
        int v2 = 10000 + dist2(rng);  // Offset mesh2 vertices
        
        std::vector<int> indices = {v1, v2};
        projectors.emplace_back(indices, ConstraintType::ADHESION, -1);
    }
    
    return projectors;
}

/**
 * @brief Benchmark serial solving
 */
double benchmarkSerial(const std::vector<MockConstraintProjector>& projectors, int num_iterations)
{
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int iter = 0; iter < num_iterations; iter++) {
        for (const auto& proj : projectors) {
            if (proj.isValid()) {
                proj.project();
            }
        }
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::milli>(end - start).count();
}

/**
 * @brief Benchmark colored parallel solving
 */
double benchmarkColoredParallel(
    const std::vector<MockConstraintProjector>& projectors,
    const Solver::GraphColoring::ColoringResult& coloring,
    int num_iterations)
{
    auto start = std::chrono::high_resolution_clock::now();
    
    for (int iter = 0; iter < num_iterations; iter++) {
        for (int color = 0; color < coloring.num_colors; color++) {
            const auto& color_group = coloring.color_groups[color];
            
#ifdef _OPENMP
            #pragma omp parallel for schedule(dynamic, 16)
#endif
            for (size_t i = 0; i < color_group.size(); i++) {
                int proj_idx = color_group[i];
                if (projectors[proj_idx].isValid()) {
                    projectors[proj_idx].project();
                }
            }
        }
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::milli>(end - start).count();
}

/**
 * @brief Print detailed statistics
 */
void printDetailedStats(
    const std::vector<MockConstraintProjector>& projectors,
    const Solver::GraphColoring::ColoringResult& coloring)
{
    int vol_constraints = 0;
    int adhesion_constraints = 0;
    
    for (const auto& proj : projectors) {
        if (proj.type() == ConstraintType::VOLUMETRIC) vol_constraints++;
        else adhesion_constraints++;
    }
    
    std::cout << "\n========================================\n";
    std::cout << "CONSTRAINT BREAKDOWN\n";
    std::cout << "========================================\n";
    std::cout << "Volumetric constraints:    " << vol_constraints << "\n";
    std::cout << "Adhesion constraints:      " << adhesion_constraints << "\n";
    std::cout << "Total constraints:         " << projectors.size() << "\n";
    std::cout << "Adhesion ratio:            " << std::fixed << std::setprecision(1)
              << (100.0 * adhesion_constraints / projectors.size()) << "%\n";
    
    // Analyze color distribution for each type
    std::vector<int> vol_colors(coloring.num_colors, 0);
    std::vector<int> adhesion_colors(coloring.num_colors, 0);
    
    for (int color = 0; color < coloring.num_colors; color++) {
        for (int idx : coloring.color_groups[color]) {
            if (projectors[idx].type() == ConstraintType::VOLUMETRIC) {
                vol_colors[color]++;
            } else {
                adhesion_colors[color]++;
            }
        }
    }
    
    int max_vol = *std::max_element(vol_colors.begin(), vol_colors.end());
    int max_adhesion = *std::max_element(adhesion_colors.begin(), adhesion_colors.end());
    
    std::cout << "\nColor Distribution:\n";
    std::cout << "  Max volumetric/color:    " << max_vol << "\n";
    std::cout << "  Max adhesion/color:      " << max_adhesion << "\n";
    
    double vol_eff = (vol_constraints / (double)coloring.num_colors) / max_vol;
    std::cout << "  Volumetric efficiency:   " << std::fixed << std::setprecision(1)
              << (vol_eff * 100) << "%\n";
}

int main(int argc, char** argv)
{
    std::cout << "========================================\n";
    std::cout << " ADHESION + COLORING BENCHMARK\n";
    std::cout << "========================================\n\n";
    
    // Configuration
    std::string tumor_path = "../resource/tumor_fixed/tumor_squeezed_uv03.msh";
    std::string brain_path = "../resource/brain_fixed/brain_MOD3_uv.msh";
    int num_adhesions = 500;  // Number of adhesion constraints to create
    int num_iterations = 50;
    
    if (argc >= 2) num_adhesions = std::atoi(argv[1]);
    if (argc >= 3) num_iterations = std::atoi(argv[2]);
    
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
    std::cout << "OpenMP:          ENABLED (" << num_threads << " threads)\n";
#else
    std::cout << "OpenMP:          DISABLED\n";
#endif
    
    std::cout << "Tumor mesh:      " << tumor_path << "\n";
    std::cout << "Brain mesh:      " << brain_path << "\n";
    std::cout << "Adhesions:       " << num_adhesions << "\n";
    std::cout << "Iterations:      " << num_iterations << "\n\n";
    
    // Initialize Gmsh
    gmsh::initialize();
    
    // Load meshes
    std::cout << "Loading tumor mesh...\n";
    Geometry::TetMesh tumor_mesh = MeshUtils::loadTetMeshFromGmshFile(tumor_path);
    std::cout << "  Tumor: " << tumor_mesh.numVertices() << " vertices, "
              << tumor_mesh.numElements() << " tets\n";
    
    std::cout << "Loading brain mesh...\n";
    Geometry::TetMesh brain_mesh = MeshUtils::loadTetMeshFromGmshFile(brain_path);
    std::cout << "  Brain: " << brain_mesh.numVertices() << " vertices, "
              << brain_mesh.numElements() << " tets\n\n";
    
    // Create constraints
    std::cout << "Creating volumetric constraints...\n";
    auto tumor_constraints = createVolumetricConstraints(tumor_mesh, 0);
    auto brain_constraints = createVolumetricConstraints(brain_mesh, 1);
    
    std::cout << "Creating adhesion constraints...\n";
    auto adhesion_constraints = createAdhesionConstraints(tumor_mesh, brain_mesh, num_adhesions);
    
    // Combine all constraints
    std::vector<MockConstraintProjector> all_constraints;
    all_constraints.reserve(tumor_constraints.size() + brain_constraints.size() + adhesion_constraints.size());
    all_constraints.insert(all_constraints.end(), tumor_constraints.begin(), tumor_constraints.end());
    all_constraints.insert(all_constraints.end(), brain_constraints.begin(), brain_constraints.end());
    all_constraints.insert(all_constraints.end(), adhesion_constraints.begin(), adhesion_constraints.end());
    
    std::cout << "  Total constraints: " << all_constraints.size() << "\n\n";
    
    // Perform graph coloring
    std::cout << "Running graph coloring (with adhesion)...\n";
    auto coloring_start = std::chrono::high_resolution_clock::now();
    auto coloring = Solver::GraphColoring::colorConstraints(all_constraints);
    auto coloring_end = std::chrono::high_resolution_clock::now();
    double coloring_time = std::chrono::duration<double, std::milli>(coloring_end - coloring_start).count();
    
    std::cout << "  Colors: " << coloring.num_colors << "\n";
    std::cout << "  Coloring time: " << std::fixed << std::setprecision(2) 
              << coloring_time << " ms\n";
    
    printDetailedStats(all_constraints, coloring);
    
    // Benchmark 1: Serial
    std::cout << "\n========================================\n";
    std::cout << "BENCHMARK 1: Serial Gauss-Seidel\n";
    std::cout << "========================================\n";
    
    double serial_time = benchmarkSerial(all_constraints, num_iterations);
    std::cout << "  Total time:       " << std::fixed << std::setprecision(2) 
              << serial_time << " ms\n";
    std::cout << "  Time per iter:    " << std::fixed << std::setprecision(3)
              << (serial_time / num_iterations) << " ms\n";
    
    // Benchmark 2: Colored Parallel
    std::cout << "\n========================================\n";
    std::cout << "BENCHMARK 2: Colored Gauss-Seidel\n";
    std::cout << "========================================\n";
    
    double parallel_time = benchmarkColoredParallel(all_constraints, coloring, num_iterations);
    std::cout << "  Total time:       " << std::fixed << std::setprecision(2)
              << parallel_time << " ms\n";
    std::cout << "  Time per iter:    " << std::fixed << std::setprecision(3)
              << (parallel_time / num_iterations) << " ms\n";
    
    // Test 3: Dynamic constraint breaking (simulate cutting)
    std::cout << "\n========================================\n";
    std::cout << "TEST 3: Dynamic Constraint Breaking\n";
    std::cout << "========================================\n";
    std::cout << "Simulating knife cutting (breaking adhesions)...\n";
    
    // Break 20% of adhesion constraints randomly
    std::mt19937 rng(123);
    std::uniform_real_distribution<double> prob(0.0, 1.0);
    int broken_count = 0;
    
    for (auto& proj : all_constraints) {
        if (proj.type() == ConstraintType::ADHESION && prob(rng) < 0.2) {
            proj.setValid(false);
            broken_count++;
        }
    }
    
    std::cout << "  Broke " << broken_count << " adhesion constraints\n";
    
    // Test recoloring overhead
    std::cout << "  Recoloring after constraint changes...\n";
    auto recolor_start = std::chrono::high_resolution_clock::now();
    auto new_coloring = Solver::GraphColoring::colorConstraints(all_constraints);
    auto recolor_end = std::chrono::high_resolution_clock::now();
    double recolor_time = std::chrono::duration<double, std::milli>(recolor_end - recolor_start).count();
    
    std::cout << "  New colors: " << new_coloring.num_colors 
              << " (was " << coloring.num_colors << ")\n";
    std::cout << "  Recoloring time: " << std::fixed << std::setprecision(2)
              << recolor_time << " ms\n";
    
    // Benchmark after breaking
    double serial_after = benchmarkSerial(all_constraints, num_iterations);
    double parallel_after = benchmarkColoredParallel(all_constraints, new_coloring, num_iterations);
    
    std::cout << "  Serial time after:    " << std::fixed << std::setprecision(2)
              << serial_after << " ms\n";
    std::cout << "  Parallel time after:  " << std::fixed << std::setprecision(2)
              << parallel_after << " ms\n";
    std::cout << "  Speedup after break:  " << std::fixed << std::setprecision(2)
              << (serial_after / parallel_after) << "x\n";
    
    // Final comparison
    std::cout << "\n========================================\n";
    std::cout << "FINAL RESULTS\n";
    std::cout << "========================================\n";
    
    double speedup_before = serial_time / parallel_time;
    double speedup_after = serial_after / parallel_after;
    
    std::cout << "Speedup (before cutting): " << std::fixed << std::setprecision(2)
              << speedup_before << "x\n";
    std::cout << "Speedup (after cutting):  " << std::fixed << std::setprecision(2)
              << speedup_after << "x\n";
    std::cout << "Recoloring overhead:      " << std::fixed << std::setprecision(2)
              << recolor_time << " ms (= " << (recolor_time / (serial_time / num_iterations))
              << " iterations)\n";
    
    std::cout << "\n========================================\n";
    std::cout << "RECOMMENDATION\n";
    std::cout << "========================================\n";
    
    if (speedup_before >= 2.5 && speedup_after >= 2.0 && recolor_time < 50.0) {
        std::cout << "✓ EXCELLENT performance with adhesion constraints!\n\n";
        std::cout << "  Graph coloring works well with:\n";
        std::cout << "  - Mixed constraint types (volumetric + adhesion)\n";
        std::cout << "  - Dynamic constraint changes (cutting)\n";
        std::cout << "  - Recoloring overhead is acceptable\n\n";
        std::cout << "  STRONGLY RECOMMEND integration!\n";
    } else if (speedup_before >= 1.5) {
        std::cout << "⚠ MODERATE performance with adhesion constraints\n\n";
        std::cout << "  - Speedup is decent but not exceptional\n";
        std::cout << "  - May want to optimize recoloring strategy\n\n";
        std::cout << "  CONSIDER integration with monitoring\n";
    } else {
        std::cout << "✗ Performance degraded with adhesion constraints\n\n";
        std::cout << "  - Graph coloring may not be suitable for this use case\n";
        std::cout << "  - Consider alternative optimization strategies\n";
    }
    
    std::cout << "\n========================================\n";
    std::cout << "Benchmark Complete!\n";
    std::cout << "========================================\n";
    
    gmsh::finalize();
    return 0;
}
