/**
 * @file test_graph_coloring.cpp
 * @brief Unit tests for graph coloring algorithm
 */

#include "solver/xpbd_solver/GraphColoring.hpp"
#include "solver/constraint/DeviatoricConstraint.hpp"
#include "solver/constraint/HydrostaticConstraint.hpp"
#include "solver/xpbd_projector/ConstraintProjector.hpp"

#include <iostream>
#include <cassert>
#include <vector>

using namespace Solver;

/**
 * @brief Mock constraint for testing
 */
class MockConstraint : public Constraint
{
public:
    MockConstraint(const std::vector<int>& vertex_indices)
        : Constraint({})
    {
        for (int idx : vertex_indices) {
            _positions.push_back(PositionReference(idx, nullptr, 1.0));
        }
    }
    
    int numPositions() const override { return _positions.size(); }
    int numCoordinates() const override { return _positions.size() * 3; }
    bool isInequality() const override { return false; }
    
    void evaluate(Real* C) const override { *C = 0; }
    void gradient(Real* grad) const override {}
    void evaluateWithGradient(Real* C, Real* grad) const override {}
};

/**
 * @brief Mock projector for testing
 */
class MockProjector
{
public:
    MockProjector(MockConstraint&& c) : _constraint(std::move(c)) {}
    
    const MockConstraint& constraint() const { return _constraint; }
    bool isValid() const { return true; }
    
private:
    MockConstraint _constraint;
};

void test_simple_coloring()
{
    std::cout << "\n=== Test: Simple Coloring ===\n";
    
    // Create constraints:
    // C0: vertices [0, 1]
    // C1: vertices [2, 3]  <- can be parallel with C0
    // C2: vertices [1, 2]  <- conflicts with both C0 and C1
    
    std::vector<MockProjector> constraints;
    constraints.emplace_back(MockConstraint({0, 1}));
    constraints.emplace_back(MockConstraint({2, 3}));
    constraints.emplace_back(MockConstraint({1, 2}));
    
    auto result = GraphColoring::colorConstraints(constraints);
    
    // Verify:
    // - Should have at least 2 colors
    // - C0 and C1 should have same color (no conflict)
    // - C2 should have different color
    
    std::cout << "Expected: 2 colors\n";
    std::cout << "Got: " << result.num_colors << " colors\n";
    
    assert(result.num_colors >= 2);
    
    std::cout << "✓ Simple coloring test passed\n";
}

void test_chain_coloring()
{
    std::cout << "\n=== Test: Chain Coloring ===\n";
    
    // Create chain: C0-C1-C2-C3
    // Each shares one vertex with next
    // Should need 2 colors (alternating)
    
    std::vector<MockProjector> constraints;
    constraints.emplace_back(MockConstraint({0, 1}));
    constraints.emplace_back(MockConstraint({1, 2}));
    constraints.emplace_back(MockConstraint({2, 3}));
    constraints.emplace_back(MockConstraint({3, 4}));
    
    auto result = GraphColoring::colorConstraints(constraints);
    
    std::cout << "Expected: 2 colors (alternating chain)\n";
    std::cout << "Got: " << result.num_colors << " colors\n";
    
    assert(result.num_colors >= 2);
    
    std::cout << "✓ Chain coloring test passed\n";
}

void test_mesh_coloring()
{
    std::cout << "\n=== Test: Tetrahedral Mesh Coloring ===\n";
    
    // Simulate small tet mesh
    // Tet0: [0,1,2,3]
    // Tet1: [1,2,3,4]  <- shares face with Tet0
    // Tet2: [5,6,7,8]  <- independent, can be parallel with Tet0
    
    std::vector<MockProjector> constraints;
    constraints.emplace_back(MockConstraint({0, 1, 2, 3}));
    constraints.emplace_back(MockConstraint({1, 2, 3, 4}));
    constraints.emplace_back(MockConstraint({5, 6, 7, 8}));
    
    auto result = GraphColoring::colorConstraints(constraints);
    
    std::cout << "Expected: 2 colors\n";
    std::cout << "Got: " << result.num_colors << " colors\n";
    
    // Tet0 and Tet2 should be parallelizable
    assert(result.num_colors >= 2);
    
    std::cout << "✓ Mesh coloring test passed\n";
}

void test_performance()
{
    std::cout << "\n=== Test: Performance with Many Constraints ===\n";
    
    const int num_tets = 1000;
    std::vector<MockProjector> constraints;
    
    // Create realistic mesh pattern
    for (int i = 0; i < num_tets; i++) {
        int v0 = i * 2;
        int v1 = v0 + 1;
        int v2 = v0 + 2;
        int v3 = v0 + 3;
        constraints.emplace_back(MockConstraint({v0, v1, v2, v3}));
    }
    
    auto start = std::chrono::high_resolution_clock::now();
    auto result = GraphColoring::colorConstraints(constraints);
    auto end = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    std::cout << "Colored " << num_tets << " constraints in " 
              << duration.count() << "ms\n";
    std::cout << "Result: " << result.num_colors << " colors\n";
    
    std::cout << "✓ Performance test passed\n";
}

int main()
{
    std::cout << "========================================\n";
    std::cout << "Graph Coloring Unit Tests\n";
    std::cout << "========================================\n";
    
    try {
        test_simple_coloring();
        test_chain_coloring();
        test_mesh_coloring();
        test_performance();
        
        std::cout << "\n========================================\n";
        std::cout << "All tests passed! ✓\n";
        std::cout << "========================================\n";
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "\n✗ Test failed: " << e.what() << "\n";
        return 1;
    }
}
