#include "solver/constraint/NerveBendingConstraint.hpp"
#include "common/types.hpp"

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Helper function to print test results
void printTestResult(const std::string& test_name, bool passed) {
    std::cout << "[" << (passed ? "PASS" : "FAIL") << "] " << test_name << std::endl;
}

// Helper function to check if two values are approximately equal
bool isApproxEqual(Real a, Real b, Real tolerance = 1e-6) {
    return std::abs(a - b) < tolerance;
}

// Helper function to check if a value is approximately zero
bool isApproxZero(Real value, Real tolerance = 1e-6) {
    return std::abs(value) < tolerance;
}

// Finite-difference gradient checker utility function
bool checkGradientFiniteDifference(Real* positions[3], Real rest_curvature = 0.0,
                                   Real eps = 1e-8, Real tolerance = 1e-5) {
    std::cout << "  Checking gradient via finite differences (eps=" << eps << ")..." << std::endl;
    
    // Create constraint for current configuration
    static Real masses[3] = {1.0, 1.0, 1.0};
    Solver::NerveBendingConstraint constraint(
        0, positions[0], masses[0],
        1, positions[1], masses[1],
        2, positions[2], masses[2],
        rest_curvature, 0.0
    );
    
    // Get analytical gradient
    Real analytical_grad[9] = {0};
    constraint.gradient(analytical_grad);
    
    // Compute numerical gradient via finite differences
    Real numerical_grad[9] = {0};
    
    // For each coordinate (9 total: 3 points * 3 coords each)
    for (int coord = 0; coord < 9; ++coord) {
        int point_idx = coord / 3;  // Which point (0, 1, or 2)
        int coord_idx = coord % 3;  // Which coordinate (x, y, z)
        
        // Store original value
        Real original = positions[point_idx][coord_idx];
        
        // Compute C(x + eps)
        positions[point_idx][coord_idx] = original + eps;
        Solver::NerveBendingConstraint constraint_plus(
            0, positions[0], masses[0],
            1, positions[1], masses[1],
            2, positions[2], masses[2],
            rest_curvature, 0.0
        );
        Real C_plus;
        constraint_plus.evaluate(&C_plus);
        
        // Compute C(x - eps)
        positions[point_idx][coord_idx] = original - eps;
        Solver::NerveBendingConstraint constraint_minus(
            0, positions[0], masses[0],
            1, positions[1], masses[1],
            2, positions[2], masses[2],
            rest_curvature, 0.0
        );
        Real C_minus;
        constraint_minus.evaluate(&C_minus);
        
        // Restore original value
        positions[point_idx][coord_idx] = original;
        
        // Numerical gradient
        numerical_grad[coord] = (C_plus - C_minus) / (2.0 * eps);
    }
    
    // Compare analytical vs numerical gradients
    Real max_error = 0.0;
    bool all_within_tolerance = true;
    
    for (int i = 0; i < 9; ++i) {
        Real error = std::abs(analytical_grad[i] - numerical_grad[i]);
        if (error > max_error) {
            max_error = error;
        }
        if (error > tolerance) {
            all_within_tolerance = false;
            std::cout << "    Gradient[" << i << "]: analytical=" << analytical_grad[i] 
                      << ", numerical=" << numerical_grad[i] << ", error=" << error << std::endl;
        }
    }
    
    std::cout << "    Max gradient error: " << max_error << " (tolerance: " << tolerance << ")" << std::endl;
    return all_within_tolerance && max_error < tolerance;
}

// Helper function to create a NerveBendingConstraint for testing
Solver::NerveBendingConstraint createTestConstraint(const Vec3r& p0, const Vec3r& p1, const Vec3r& p2, 
                                                   Real rest_curvature = 0.0, Real alpha = 0.0) {
    // Create position arrays (these need to persist for the constraint lifetime)
    static Real pos0[3], pos1[3], pos2[3];
    static Real masses[3] = {1.0, 1.0, 1.0}; // Unit masses for testing
    
    // Copy positions
    for (int i = 0; i < 3; ++i) {
        pos0[i] = p0[i];
        pos1[i] = p1[i];
        pos2[i] = p2[i];
    }
    
    return Solver::NerveBendingConstraint(
        0, pos0, masses[0],  // vertex 0
        1, pos1, masses[1],  // vertex 1  
        2, pos2, masses[2],  // vertex 2
        rest_curvature,
        alpha
    );
}

int main() {
    std::cout << "=== Nerve Bending Constraint Unit Tests ===" << std::endl;
    std::cout << std::fixed << std::setprecision(8);
    
    int total_tests = 0;
    int passed_tests = 0;
    
    // Test 1: Colinear case (three points on a straight line)
    std::cout << "\n--- Test 1: Colinear Case ---" << std::endl;
    {
        // Three points on the x-axis: (0,0,0), (1,0,0), (2,0,0)
        Vec3r p0(0.0, 0.0, 0.0);
        Vec3r p1(1.0, 0.0, 0.0);
        Vec3r p2(2.0, 0.0, 0.0);
        
        auto constraint = createTestConstraint(p0, p1, p2, /*rest_curvature=*/0.0);
        
        // Test evaluate(): curvature should be 0 for colinear points
        Real curvature;
        constraint.evaluate(&curvature);
        
        bool curvature_test_passed = isApproxZero(curvature, 1e-8);
        std::cout << "  Curvature: " << curvature << " (expected: 0)" << std::endl;
        
        // Test gradient(): should be all zeros for colinear points
        Real grad[9] = {0};
        constraint.gradient(grad);
        
        bool gradient_test_passed = true;
        Real max_grad = 0.0;
        for (int i = 0; i < 9; ++i) {
            if (std::abs(grad[i]) > max_grad) {
                max_grad = std::abs(grad[i]);
            }
            if (!isApproxZero(grad[i], 1e-8)) {
                gradient_test_passed = false;
            }
        }
        std::cout << "  Max gradient magnitude: " << max_grad << " (expected: 0)" << std::endl;
        
        bool test_passed = curvature_test_passed && gradient_test_passed;
        printTestResult("Colinear Case", test_passed);
        
        total_tests++;
        if (test_passed) passed_tests++;
    }
    
    // Test 2: Circle arc case (known curvature)
    std::cout << "\n--- Test 2: Circle Arc Case ---" << std::endl;
    {
        // Three points on a circle of radius R=1, centered at origin
        // Use points at angles: 0°, 60°, 120° (π/3, 2π/3 radians)
        Real radius = 1.0;
        
        // For discrete curvature formula κ = 2|e1×e2|/(|e1||e2|(|e1|+|e2|))
        // with 60° angle between edges: κ = sin(60°) = √3/2 ≈ 0.866
        Real expected_curvature = std::sin(M_PI/3.0);  // sin(60°) = √3/2
        
        Vec3r p0(radius * std::cos(0.0),        radius * std::sin(0.0),        0.0);      // (1, 0, 0)
        Vec3r p1(radius * std::cos(M_PI/3.0),   radius * std::sin(M_PI/3.0),   0.0);      // (0.5, √3/2, 0)
        Vec3r p2(radius * std::cos(2.0*M_PI/3.0), radius * std::sin(2.0*M_PI/3.0), 0.0);  // (-0.5, √3/2, 0)
        
        std::cout << "  Points on circle (R=" << radius << "):" << std::endl;
        std::cout << "    p0: (" << p0[0] << ", " << p0[1] << ", " << p0[2] << ")" << std::endl;
        std::cout << "    p1: (" << p1[0] << ", " << p1[1] << ", " << p1[2] << ")" << std::endl;
        std::cout << "    p2: (" << p2[0] << ", " << p2[1] << ", " << p2[2] << ")" << std::endl;
        
        // Debug: compute edges manually to verify
        Vec3r e1 = p1 - p0;  // edge from p0 to p1
        Vec3r e2 = p2 - p1;  // edge from p1 to p2
        Real norm_e1 = e1.norm();
        Real norm_e2 = e2.norm();
        Vec3r cross = e1.cross(e2);
        Real norm_cross = cross.norm();
        
        std::cout << "  Debug calculations:" << std::endl;
        std::cout << "    e1 = p1-p0: (" << e1[0] << ", " << e1[1] << ", " << e1[2] << ")" << std::endl;
        std::cout << "    e2 = p2-p1: (" << e2[0] << ", " << e2[1] << ", " << e2[2] << ")" << std::endl;
        std::cout << "    |e1| = " << norm_e1 << std::endl;
        std::cout << "    |e2| = " << norm_e2 << std::endl;
        std::cout << "    |e1 × e2| = " << norm_cross << std::endl;
        
        Real denominator = norm_e1 * norm_e2 * (norm_e1 + norm_e2);
        Real manual_curvature = 2.0 * norm_cross / denominator;
        
        std::cout << "    Denominator = |e1|*|e2|*(|e1|+|e2|) = " << denominator << std::endl;
        std::cout << "    Manual curvature = 2*|e1×e2|/denominator = " << manual_curvature << std::endl;
        
        auto constraint = createTestConstraint(p0, p1, p2, /*rest_curvature=*/0.0);
        
        // Test evaluate(): curvature should be approximately 1/R
        Real curvature;
        constraint.evaluate(&curvature);
        
        Real curvature_error = std::abs(curvature - expected_curvature);
        bool curvature_test_passed = curvature_error < 1e-6;  // High precision since we know the exact value
        
        std::cout << "  Computed curvature: " << curvature << std::endl;
        std::cout << "  Expected curvature: " << expected_curvature << std::endl;
        std::cout << "  Error: " << curvature_error << " (tolerance: 1e-6)" << std::endl;
        
        // Test gradient: should be reasonable (non-zero but finite)
        Real grad[9] = {0};
        constraint.gradient(grad);
        
        bool gradient_test_passed = true;
        Real max_grad = 0.0;
        for (int i = 0; i < 9; ++i) {
            Real abs_grad = std::abs(grad[i]);
            if (abs_grad > max_grad) {
                max_grad = abs_grad;
            }
            // Check gradient is finite and not too large
            if (!std::isfinite(grad[i]) || abs_grad > 100.0) {
                gradient_test_passed = false;
                std::cout << "  Invalid gradient[" << i << "] = " << grad[i] << std::endl;
            }
        }
        std::cout << "  Max gradient magnitude: " << max_grad << " (should be finite and reasonable)" << std::endl;
        
        // Test gradient correctness via finite differences
        Real* test_positions[3] = {
            new Real[3]{p0[0], p0[1], p0[2]},
            new Real[3]{p1[0], p1[1], p1[2]},
            new Real[3]{p2[0], p2[1], p2[2]}
        };
        
        bool gradient_finite_diff_passed = checkGradientFiniteDifference(test_positions, 0.0);
        
        // Clean up
        delete[] test_positions[0];
        delete[] test_positions[1];
        delete[] test_positions[2];
        
        bool test_passed = curvature_test_passed && gradient_test_passed && gradient_finite_diff_passed;
        printTestResult("Circle Arc Case", test_passed);
        
        total_tests++;
        if (test_passed) passed_tests++;
    }
    
    // Test 3: Degenerate edge cases (safety test)
    std::cout << "\n--- Test 3: Degenerate Edge Cases ---" << std::endl;
    {
        bool all_degenerate_tests_passed = true;
        
        // Test 3a: Zero-length first edge (p0 == p1)
        std::cout << "  Test 3a: Zero-length first edge..." << std::endl;
        {
            Vec3r p0(0.0, 0.0, 0.0);
            Vec3r p1(0.0, 0.0, 0.0);  // Same as p0
            Vec3r p2(1.0, 0.0, 0.0);
            
            auto constraint = createTestConstraint(p0, p1, p2, /*rest_curvature=*/0.0);
            
            Real curvature;
            constraint.evaluate(&curvature);
            
            Real grad[9] = {0};
            constraint.gradient(grad);
            
            // Check that values are finite (not NaN or infinite)
            bool values_finite = std::isfinite(curvature);
            for (int i = 0; i < 9; ++i) {
                if (!std::isfinite(grad[i])) {
                    values_finite = false;
                    break;
                }
            }
            
            std::cout << "    Curvature: " << curvature << " (finite: " << (std::isfinite(curvature) ? "yes" : "no") << ")" << std::endl;
            std::cout << "    All gradients finite: " << (values_finite ? "yes" : "no") << std::endl;
            
            if (!values_finite) {
                all_degenerate_tests_passed = false;
            }
        }
        
        // Test 3b: Zero-length second edge (p1 == p2)
        std::cout << "  Test 3b: Zero-length second edge..." << std::endl;
        {
            Vec3r p0(0.0, 0.0, 0.0);
            Vec3r p1(1.0, 0.0, 0.0);
            Vec3r p2(1.0, 0.0, 0.0);  // Same as p1
            
            auto constraint = createTestConstraint(p0, p1, p2, /*rest_curvature=*/0.0);
            
            Real curvature;
            constraint.evaluate(&curvature);
            
            Real grad[9] = {0};
            constraint.gradient(grad);
            
            // Check that values are finite
            bool values_finite = std::isfinite(curvature);
            for (int i = 0; i < 9; ++i) {
                if (!std::isfinite(grad[i])) {
                    values_finite = false;
                    break;
                }
            }
            
            std::cout << "    Curvature: " << curvature << " (finite: " << (std::isfinite(curvature) ? "yes" : "no") << ")" << std::endl;
            std::cout << "    All gradients finite: " << (values_finite ? "yes" : "no") << std::endl;
            
            if (!values_finite) {
                all_degenerate_tests_passed = false;
            }
        }
        
        // Test 3c: All points coincident (p0 == p1 == p2)
        std::cout << "  Test 3c: All points coincident..." << std::endl;
        {
            Vec3r p0(0.0, 0.0, 0.0);
            Vec3r p1(0.0, 0.0, 0.0);  // Same as p0
            Vec3r p2(0.0, 0.0, 0.0);  // Same as p0
            
            auto constraint = createTestConstraint(p0, p1, p2, /*rest_curvature=*/0.0);
            
            Real curvature;
            constraint.evaluate(&curvature);
            
            Real grad[9] = {0};
            constraint.gradient(grad);
            
            // Check that values are finite
            bool values_finite = std::isfinite(curvature);
            for (int i = 0; i < 9; ++i) {
                if (!std::isfinite(grad[i])) {
                    values_finite = false;
                    break;
                }
            }
            
            std::cout << "    Curvature: " << curvature << " (finite: " << (std::isfinite(curvature) ? "yes" : "no") << ")" << std::endl;
            std::cout << "    All gradients finite: " << (values_finite ? "yes" : "no") << std::endl;
            
            if (!values_finite) {
                all_degenerate_tests_passed = false;
            }
        }
        
        printTestResult("Degenerate Edge Cases", all_degenerate_tests_passed);
        
        total_tests++;
        if (all_degenerate_tests_passed) passed_tests++;
    }
    
    std::cout << "\n=== Test Summary ===" << std::endl;
    std::cout << "Total tests: " << total_tests << std::endl;
    std::cout << "Passed: " << passed_tests << std::endl;
    std::cout << "Failed: " << (total_tests - passed_tests) << std::endl;
    
    if (passed_tests == total_tests) {
        std::cout << "All tests PASSED!" << std::endl;
        return 0;
    } else {
        std::cout << "Some tests FAILED!" << std::endl;
        return 1;
    }
}