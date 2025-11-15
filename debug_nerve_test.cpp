#include "solver/constraint/NerveTumorAdhesionConstraint.hpp"
#include "common/types.hpp"
#include <iostream>

int main() {
    std::cout << "=== Debug NerveTumorAdhesion ===" << std::endl;
    
    // Simple test configuration
    Real nerve_pos[3] = {0.0, 0.0, 0.5};     // above origin
    Real tri_p1[3] = {-1.0, -1.0, 0.0};      // triangle in XY plane
    Real tri_p2[3] = {1.0, -1.0, 0.0};
    Real tri_p3[3] = {0.0, 1.0, 0.0};
    Real target_gap = 0.1;
    
    static Real masses[4] = {1.0, 1.0, 1.0, 1.0};
    Solver::NerveTumorAdhesionConstraint constraint(
        0, nerve_pos, masses[0],
        1, tri_p1, masses[1],
        2, tri_p2, masses[2],
        3, tri_p3, masses[3],
        target_gap, 0.0
    );
    
    // Test evaluate
    Real C;
    constraint.evaluate(&C);
    std::cout << "C = " << C << " (expected: 0.4)" << std::endl;
    
    // Test gradient
    Real grad[12] = {0};
    constraint.gradient(grad);
    std::cout << "Nerve gradient: [" << grad[0] << ", " << grad[1] << ", " << grad[2] << "]" << std::endl;
    std::cout << "Expected: [0, 0, 1]" << std::endl;
    
    return 0;
}