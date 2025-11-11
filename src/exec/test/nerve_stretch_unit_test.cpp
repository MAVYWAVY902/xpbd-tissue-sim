#include "solver/constraint/NerveStretchConstraint.hpp"
#include "common/types.hpp"

#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <string>

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
bool checkGradientFiniteDifference(Real* positions[2], Real rest_length,
                                   Real eps = 1e-8, Real tolerance = 1e-5) {
    std::cout << "  Checking gradient via finite differences (eps=" << eps << ")..." << std::endl;
    
    // Create constraint for current configuration
    static Real masses[2] = {1.0, 1.0};
    Solver::NerveStretchConstraint constraint(
        0, positions[0], masses[0],
        1, positions[1], masses[1],
        rest_length, 0.0
    );
    
    // Get analytical gradient
    Real analytical_grad[6] = {0};
    constraint.gradient(analytical_grad);
    
    // Compute numerical gradient via finite differences
    Real numerical_grad[6] = {0};
    
    // For each coordinate (6 total: 2 points * 3 coords each)
    for (int coord = 0; coord < 6; ++coord) {
        int point_idx = coord / 3;  // Which point (0 or 1)
        int coord_idx = coord % 3;  // Which coordinate (x, y, z)
        
        // Store original value
        Real original = positions[point_idx][coord_idx];
        
        // Compute C(x + eps)
        positions[point_idx][coord_idx] = original + eps;
        Solver::NerveStretchConstraint constraint_plus(
            0, positions[0], masses[0],
            1, positions[1], masses[1],
            rest_length, 0.0
        );
        Real C_plus;
        constraint_plus.evaluate(&C_plus);
        
        // Compute C(x - eps)
        positions[point_idx][coord_idx] = original - eps;
        Solver::NerveStretchConstraint constraint_minus(
            0, positions[0], masses[0],
            1, positions[1], masses[1],
            rest_length, 0.0
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
    
    for (int i = 0; i < 6; ++i) {
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

// Helper function to create a NerveStretchConstraint for testing
Solver::NerveStretchConstraint createTestConstraint(const Vec3r& p0, const Vec3r& p1, 
                                                   Real rest_length, Real alpha = 0.0) {
    // Create position arrays (these need to persist for the constraint lifetime)
    static Real pos0[3], pos1[3];
    static Real masses[2] = {1.0, 1.0}; // Unit masses for testing
    
    // Copy positions
    for (int i = 0; i < 3; ++i) {
        pos0[i] = p0[i];
        pos1[i] = p1[i];
    }
    
    return Solver::NerveStretchConstraint(
        0, pos0, masses[0],  // vertex 0
        1, pos1, masses[1],  // vertex 1
        rest_length,
        alpha
    );
}

int main() {
    std::cout << "=== Nerve Stretch Constraint Unit Tests ===" << std::endl;
    std::cout << std::fixed << std::setprecision(8);
    
    int total_tests = 0;
    int passed_tests = 0;
    
    // Test 1: Rest length case (no deformation)
    std::cout << "\n--- Test 1: Rest Length Case ---" << std::endl;
    {
        // Two points at rest distance of 1.0
        Vec3r p0(0.0, 0.0, 0.0);
        Vec3r p1(1.0, 0.0, 0.0);
        Real rest_length = 1.0;
        
        auto constraint = createTestConstraint(p0, p1, rest_length);
        
        // Test evaluate(): constraint should be 0 (current_length - rest_length = 0)
        Real constraint_value;
        constraint.evaluate(&constraint_value);
        
        bool constraint_test_passed = isApproxZero(constraint_value, 1e-8);
        std::cout << "  Constraint value: " << constraint_value << " (expected: 0)" << std::endl;
        std::cout << "  Current length: " << (p1 - p0).norm() << ", Rest length: " << rest_length << std::endl;
        
        // Test gradient(): should be unit vector along the edge direction
        Real grad[6] = {0};
        constraint.gradient(grad);
        
        // For stretch constraint: C = |p1 - p0| - L0
        // dC/dp0 = -(p1-p0)/|p1-p0|  (negative unit vector)
        // dC/dp1 = +(p1-p0)/|p1-p0|  (positive unit vector)
        Vec3r edge = p1 - p0;
        Vec3r unit_edge = edge.normalized();
        
        bool gradient_test_passed = true;
        Real tolerance = 1e-8;
        
        // Check gradient for p0 (should be -unit_edge)
        for (int i = 0; i < 3; ++i) {
            if (!isApproxEqual(grad[i], -unit_edge[i], tolerance)) {
                gradient_test_passed = false;
                std::cout << "  Gradient p0[" << i << "]: got " << grad[i] << ", expected " << -unit_edge[i] << std::endl;
            }
        }
        
        // Check gradient for p1 (should be +unit_edge)
        for (int i = 0; i < 3; ++i) {
            if (!isApproxEqual(grad[3+i], unit_edge[i], tolerance)) {
                gradient_test_passed = false;
                std::cout << "  Gradient p1[" << i << "]: got " << grad[3+i] << ", expected " << unit_edge[i] << std::endl;
            }
        }
        
        std::cout << "  Expected gradient p0: (" << -unit_edge[0] << ", " << -unit_edge[1] << ", " << -unit_edge[2] << ")" << std::endl;
        std::cout << "  Expected gradient p1: (" << unit_edge[0] << ", " << unit_edge[1] << ", " << unit_edge[2] << ")" << std::endl;
        std::cout << "  Actual gradient p0:   (" << grad[0] << ", " << grad[1] << ", " << grad[2] << ")" << std::endl;
        std::cout << "  Actual gradient p1:   (" << grad[3] << ", " << grad[4] << ", " << grad[5] << ")" << std::endl;
        
        bool test_passed = constraint_test_passed && gradient_test_passed;
        printTestResult("Rest Length Case", test_passed);
        
        total_tests++;
        if (test_passed) passed_tests++;
    }
    
    // Test 2: Stretched case (positive strain)
    std::cout << "\n--- Test 2: Stretched Case ---" << std::endl;
    {
        // Two points stretched beyond rest length
        Vec3r p0(0.0, 0.0, 0.0);
        Vec3r p1(2.0, 0.0, 0.0);  // Current length = 2.0
        Real rest_length = 1.0;   // Rest length = 1.0
        Real expected_strain = 1.0;  // Strain = 2.0 - 1.0 = 1.0
        
        auto constraint = createTestConstraint(p0, p1, rest_length);
        
        // Test evaluate(): constraint should be current_length - rest_length = 1.0
        Real constraint_value;
        constraint.evaluate(&constraint_value);
        
        Real current_length = (p1 - p0).norm();
        bool constraint_test_passed = isApproxEqual(constraint_value, expected_strain, 1e-8);
        
        std::cout << "  Current length: " << current_length << std::endl;
        std::cout << "  Rest length: " << rest_length << std::endl;
        std::cout << "  Expected strain: " << expected_strain << std::endl;
        std::cout << "  Computed constraint: " << constraint_value << std::endl;
        std::cout << "  Error: " << std::abs(constraint_value - expected_strain) << std::endl;
        
        // Test gradient(): should be unit vector along edge direction
        Real grad[6] = {0};
        constraint.gradient(grad);
        
        Vec3r edge = p1 - p0;
        Vec3r unit_edge = edge.normalized();
        
        bool gradient_test_passed = true;
        Real tolerance = 1e-8;
        
        // Check that gradients are unit vectors in correct directions
        Vec3r grad_p0(grad[0], grad[1], grad[2]);
        Vec3r grad_p1(grad[3], grad[4], grad[5]);
        
        Real grad_p0_norm = grad_p0.norm();
        Real grad_p1_norm = grad_p1.norm();
        
        std::cout << "  |grad_p0|: " << grad_p0_norm << " (expected: 1.0)" << std::endl;
        std::cout << "  |grad_p1|: " << grad_p1_norm << " (expected: 1.0)" << std::endl;
        
        if (!isApproxEqual(grad_p0_norm, 1.0, tolerance) || 
            !isApproxEqual(grad_p1_norm, 1.0, tolerance)) {
            gradient_test_passed = false;
        }
        
        // Check directions
        for (int i = 0; i < 3; ++i) {
            if (!isApproxEqual(grad[i], -unit_edge[i], tolerance)) {
                gradient_test_passed = false;
            }
            if (!isApproxEqual(grad[3+i], unit_edge[i], tolerance)) {
                gradient_test_passed = false;
            }
        }
        
        // Test gradient correctness via finite differences
        Real* test_positions[2] = {
            new Real[3]{p0[0], p0[1], p0[2]},
            new Real[3]{p1[0], p1[1], p1[2]}
        };
        
        bool gradient_finite_diff_passed = checkGradientFiniteDifference(test_positions, rest_length);
        
        // Clean up
        delete[] test_positions[0];
        delete[] test_positions[1];
        
        bool test_passed = constraint_test_passed && gradient_test_passed && gradient_finite_diff_passed;
        printTestResult("Stretched Case", test_passed);
        
        total_tests++;
        if (test_passed) passed_tests++;
    }
    
    // Test 3: Compressed case (negative strain)
    std::cout << "\n--- Test 3: Compressed Case ---" << std::endl;
    {
        // Two points compressed below rest length
        Vec3r p0(0.0, 0.0, 0.0);
        Vec3r p1(0.5, 0.0, 0.0);  // Current length = 0.5
        Real rest_length = 1.0;   // Rest length = 1.0
        Real expected_strain = -0.5;  // Strain = 0.5 - 1.0 = -0.5
        
        auto constraint = createTestConstraint(p0, p1, rest_length);
        
        // Test evaluate(): constraint should be current_length - rest_length = -0.5
        Real constraint_value;
        constraint.evaluate(&constraint_value);
        
        Real current_length = (p1 - p0).norm();
        bool constraint_test_passed = isApproxEqual(constraint_value, expected_strain, 1e-8);
        
        std::cout << "  Current length: " << current_length << std::endl;
        std::cout << "  Rest length: " << rest_length << std::endl;
        std::cout << "  Expected strain: " << expected_strain << std::endl;
        std::cout << "  Computed constraint: " << constraint_value << std::endl;
        std::cout << "  Error: " << std::abs(constraint_value - expected_strain) << std::endl;
        
        // Test gradient via finite differences
        Real* test_positions[2] = {
            new Real[3]{p0[0], p0[1], p0[2]},
            new Real[3]{p1[0], p1[1], p1[2]}
        };
        
        bool gradient_finite_diff_passed = checkGradientFiniteDifference(test_positions, rest_length);
        
        // Clean up
        delete[] test_positions[0];
        delete[] test_positions[1];
        
        bool test_passed = constraint_test_passed && gradient_finite_diff_passed;
        printTestResult("Compressed Case", test_passed);
        
        total_tests++;
        if (test_passed) passed_tests++;
    }
    
    // Test 4: Different orientations (non-axis-aligned)
    std::cout << "\n--- Test 4: Non-Axis-Aligned Cases ---" << std::endl;
    {
        bool all_orientation_tests_passed = true;
        
        // Test 4a: Diagonal direction
        std::cout << "  Test 4a: Diagonal direction (45 degrees)..." << std::endl;
        {
            Vec3r p0(0.0, 0.0, 0.0);
            Vec3r p1(1.0, 1.0, 0.0);  // Length = √2
            Real rest_length = 1.0;
            Real expected_strain = std::sqrt(2.0) - 1.0;  // ≈ 0.414
            
            auto constraint = createTestConstraint(p0, p1, rest_length);
            
            Real constraint_value;
            constraint.evaluate(&constraint_value);
            
            bool diagonal_test_passed = isApproxEqual(constraint_value, expected_strain, 1e-8);
            std::cout << "    Expected strain: " << expected_strain << std::endl;
            std::cout << "    Computed strain: " << constraint_value << std::endl;
            std::cout << "    Error: " << std::abs(constraint_value - expected_strain) << std::endl;
            
            if (!diagonal_test_passed) {
                all_orientation_tests_passed = false;
            }
        }
        
        // Test 4b: 3D direction
        std::cout << "  Test 4b: 3D direction..." << std::endl;
        {
            Vec3r p0(0.0, 0.0, 0.0);
            Vec3r p1(1.0, 1.0, 1.0);  // Length = √3
            Real rest_length = 2.0;
            Real current_length = std::sqrt(3.0);
            Real expected_strain = current_length - rest_length;  // √3 - 2 ≈ -0.268
            
            auto constraint = createTestConstraint(p0, p1, rest_length);
            
            Real constraint_value;
            constraint.evaluate(&constraint_value);
            
            bool threed_test_passed = isApproxEqual(constraint_value, expected_strain, 1e-8);
            std::cout << "    Current length: " << current_length << std::endl;
            std::cout << "    Expected strain: " << expected_strain << std::endl;
            std::cout << "    Computed strain: " << constraint_value << std::endl;
            std::cout << "    Error: " << std::abs(constraint_value - expected_strain) << std::endl;
            
            if (!threed_test_passed) {
                all_orientation_tests_passed = false;
            }
            
            // Also test gradient for this case
            Real* test_positions[2] = {
                new Real[3]{p0[0], p0[1], p0[2]},
                new Real[3]{p1[0], p1[1], p1[2]}
            };
            
            bool gradient_test_passed = checkGradientFiniteDifference(test_positions, rest_length);
            if (!gradient_test_passed) {
                all_orientation_tests_passed = false;
            }
            
            // Clean up
            delete[] test_positions[0];
            delete[] test_positions[1];
        }
        
        printTestResult("Non-Axis-Aligned Cases", all_orientation_tests_passed);
        
        total_tests++;
        if (all_orientation_tests_passed) passed_tests++;
    }
    
    // Test 5: Degenerate edge cases (safety test)
    std::cout << "\n--- Test 5: Degenerate Edge Cases ---" << std::endl;
    {
        bool all_degenerate_tests_passed = true;
        
        // Test 5a: Zero-length edge (p0 == p1)
        std::cout << "  Test 5a: Zero-length edge (coincident points)..." << std::endl;
        {
            Vec3r p0(0.0, 0.0, 0.0);
            Vec3r p1(0.0, 0.0, 0.0);  // Same as p0
            Real rest_length = 1.0;
            
            auto constraint = createTestConstraint(p0, p1, rest_length);
            
            Real constraint_value;
            constraint.evaluate(&constraint_value);
            
            Real grad[6] = {0};
            constraint.gradient(grad);
            
            // Check that values are finite (not NaN or infinite)
            bool values_finite = std::isfinite(constraint_value);
            for (int i = 0; i < 6; ++i) {
                if (!std::isfinite(grad[i])) {
                    values_finite = false;
                    break;
                }
            }
            
            std::cout << "    Constraint value: " << constraint_value << " (finite: " << (std::isfinite(constraint_value) ? "yes" : "no") << ")" << std::endl;
            std::cout << "    All gradients finite: " << (values_finite ? "yes" : "no") << std::endl;
            
            // For zero-length case, constraint should be -rest_length
            Real expected_value = -rest_length;
            bool value_correct = isApproxEqual(constraint_value, expected_value, 1e-8);
            std::cout << "    Expected value: " << expected_value << std::endl;
            std::cout << "    Value correct: " << (value_correct ? "yes" : "no") << std::endl;
            
            if (!values_finite || !value_correct) {
                all_degenerate_tests_passed = false;
            }
        }
        
        // Test 5b: Very small edge (near-zero length)
        std::cout << "  Test 5b: Very small edge..." << std::endl;
        {
            Vec3r p0(0.0, 0.0, 0.0);
            Vec3r p1(1e-10, 0.0, 0.0);  // Very small edge
            Real rest_length = 1.0;
            
            auto constraint = createTestConstraint(p0, p1, rest_length);
            
            Real constraint_value;
            constraint.evaluate(&constraint_value);
            
            Real grad[6] = {0};
            constraint.gradient(grad);
            
            // Check that values are finite
            bool values_finite = std::isfinite(constraint_value);
            for (int i = 0; i < 6; ++i) {
                if (!std::isfinite(grad[i])) {
                    values_finite = false;
                    break;
                }
            }
            
            std::cout << "    Constraint value: " << constraint_value << " (finite: " << (std::isfinite(constraint_value) ? "yes" : "no") << ")" << std::endl;
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