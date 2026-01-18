#include "utils/LinearSolver.hpp"
#include <iostream>
#include <iomanip>

int main()
{
    std::cout << "=== Testing 3x3 PSD Solver ===" << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    
    // Test 1: Simple diagonal matrix
    {
        std::cout << "\n--- Test 1: Diagonal Matrix ---" << std::endl;
        Real H[9] = {
            2.0, 0.0, 0.0,
            0.0, 3.0, 0.0,
            0.0, 0.0, 4.0
        };
        Real b[3] = {4.0, 9.0, 16.0};
        Real x[3] = {0.0, 0.0, 0.0};
        
        bool success = Utils::solve3x3PSD(H, b, x);
        std::cout << "Success: " << (success ? "YES" : "NO") << std::endl;
        std::cout << "Expected: [2.0, 3.0, 4.0]" << std::endl;
        std::cout << "Got:      [" << x[0] << ", " << x[1] << ", " << x[2] << "]" << std::endl;
    }
    
    // Test 2: Symmetric positive definite matrix (from Neo-Hookean)
    {
        std::cout << "\n--- Test 2: Symmetric PD Matrix ---" << std::endl;
        // H = [4  1  0]
        //     [1  3  1]
        //     [0  1  5]
        Real H[9] = {
            4.0, 1.0, 0.0,
            1.0, 3.0, 1.0,
            0.0, 1.0, 5.0
        };
        Real b[3] = {1.0, 2.0, 3.0};
        Real x[3] = {0.0, 0.0, 0.0};
        
        bool success = Utils::solve3x3PSD(H, b, x);
        std::cout << "Success: " << (success ? "YES" : "NO") << std::endl;
        
        // Verify: H * x should equal b
        Real result[3];
        result[0] = H[0]*x[0] + H[3]*x[1] + H[6]*x[2];
        result[1] = H[1]*x[0] + H[4]*x[1] + H[7]*x[2];
        result[2] = H[2]*x[0] + H[5]*x[1] + H[8]*x[2];
        
        std::cout << "H * x = [" << result[0] << ", " << result[1] << ", " << result[2] << "]" << std::endl;
        std::cout << "b     = [" << b[0] << ", " << b[1] << ", " << b[2] << "]" << std::endl;
        
        Real error = std::abs(result[0] - b[0]) + std::abs(result[1] - b[1]) + std::abs(result[2] - b[2]);
        std::cout << "Total error: " << error << std::endl;
    }
    
    // Test 3: Compare custom solver with Eigen
    {
        std::cout << "\n--- Test 3: Custom vs Eigen ---" << std::endl;
        Real H[9] = {
            5.0, 2.0, 1.0,
            2.0, 4.0, 2.0,
            1.0, 2.0, 6.0
        };
        Real b[3] = {10.0, 20.0, 30.0};
        Real x_custom[3] = {0.0, 0.0, 0.0};
        Real x_eigen[3] = {0.0, 0.0, 0.0};
        
        bool success_custom = Utils::solve3x3PSD(H, b, x_custom);
        bool success_eigen = Utils::solve3x3PSD_Eigen(H, b, x_eigen);
        
        std::cout << "Custom success: " << (success_custom ? "YES" : "NO") << std::endl;
        std::cout << "Eigen success:  " << (success_eigen ? "YES" : "NO") << std::endl;
        std::cout << "Custom: [" << x_custom[0] << ", " << x_custom[1] << ", " << x_custom[2] << "]" << std::endl;
        std::cout << "Eigen:  [" << x_eigen[0] << ", " << x_eigen[1] << ", " << x_eigen[2] << "]" << std::endl;
        
        Real diff = std::abs(x_custom[0] - x_eigen[0]) + 
                    std::abs(x_custom[1] - x_eigen[1]) + 
                    std::abs(x_custom[2] - x_eigen[2]);
        std::cout << "Difference: " << diff << std::endl;
    }
    
    // Test 4: Near-singular matrix (should handle gracefully)
    {
        std::cout << "\n--- Test 4: Near-Singular Matrix ---" << std::endl;
        Real H[9] = {
            1e-10, 0.0, 0.0,
            0.0, 2.0, 0.0,
            0.0, 0.0, 3.0
        };
        Real b[3] = {1.0, 2.0, 3.0};
        Real x[3] = {0.0, 0.0, 0.0};
        
        bool success = Utils::solve3x3PSD(H, b, x);
        std::cout << "Success: " << (success ? "YES (regularized)" : "NO") << std::endl;
        std::cout << "Solution: [" << x[0] << ", " << x[1] << ", " << x[2] << "]" << std::endl;
    }
    
    // Test 5: Fallback mechanism
    {
        std::cout << "\n--- Test 5: Fallback to Gradient Descent ---" << std::endl;
        Real H[9] = {
            -1.0, 0.0, 0.0,  // Negative definite!
            0.0, 2.0, 0.0,
            0.0, 0.0, 3.0
        };
        Real b[3] = {5.0, 10.0, 15.0};
        Real x[3] = {0.0, 0.0, 0.0};
        
        bool success = Utils::solve3x3PDSWithFallback(H, b, x, false);
        std::cout << "Newton success: " << (success ? "YES" : "NO (used fallback)") << std::endl;
        std::cout << "Solution: [" << x[0] << ", " << x[1] << ", " << x[2] << "]" << std::endl;
        std::cout << "Expected: [" << b[0] << ", " << b[1] << ", " << b[2] << "] (gradient descent)" << std::endl;
    }
    
    std::cout << "\n=== All Tests Complete ===" << std::endl;
    return 0;
}
