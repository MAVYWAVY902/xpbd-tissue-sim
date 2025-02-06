#ifndef __FIRST_ORDER_XPBD_MESH_OBJECT_CONFIG_HPP
#define __FIRST_ORDER_XPBD_MESH_OBJECT_CONFIG_HPP

#include "config/XPBDMeshObjectConfig.hpp"

/** Dictates different 'solve modes' for XPBD
 * SIMULTANEOUS: solves the hydrostatic and deviatoric constraints for each element simultaneously as a 2x2 system. Still iterates sequentially through each element.
 * SEQUENTIAL: solves the hydrostatic and deviatoric constraints for each element sequentially in true Gauss-Seidel fashion
 */
enum class FirstOrderXPBDSolveMode
{
    SIMULTANEOUS=0,
    SIMULTANEOUS_CONSTANT_B,
    SIMULTANEOUS_AVERAGE_EDGE_B,
    SIMULTANEOUS_SMALLEST_EDGE_B,
    SEQUENTIAL,
    SIMULTANEOUS_JACOBI,
    SIMULTANEOUS_CONVERGENT_JACOBI
};

enum class FirstOrderXPBDResidualPolicy
{
    NEVER=0,
    EVERY_SUBSTEP,
    EVERY_ITERATION
};

class FirstOrderXPBDMeshObjectConfig : public XPBDMeshObjectConfig
{
    /** Static predefined default for the number of solver iterations */
    // static std::optional<unsigned>& DEFAULT_NUM_SOLVER_ITERS() { static std::optional<unsigned> num_solver_iters(1); return num_solver_iters; }
    /** Static predefined default for solve mode */
    // static std::optional<FirstOrderXPBDSolveMode>& DEFAULT_SOLVE_MODE() { static std::optional<FirstOrderXPBDSolveMode> solve_mode(FirstOrderXPBDSolveMode::SEQUENTIAL); return solve_mode; }
    /** Static predefined default for residual policy */
    // static std::optional<FirstOrderXPBDResidualPolicy>& DEFAULT_RESIDUAL_POLICY() { static std::optional<FirstOrderXPBDResidualPolicy> residual_policy(FirstOrderXPBDResidualPolicy::EVERY_SUBSTEP); return residual_policy; }

    static std::optional<Real>& DEFAULT_DAMPING_MULTIPLIER() { static std::optional<Real> damping(1); return damping; }

    // static std::map<std::string, FirstOrderXPBDSolveMode>& SOLVE_MODE_OPTIONS() 
    // {
    //     static std::map<std::string, FirstOrderXPBDSolveMode> solve_mode_options{{"Simultaneous", FirstOrderXPBDSolveMode::SIMULTANEOUS},
    //                                                                    {"Simultaneous-Constant-B", FirstOrderXPBDSolveMode::SIMULTANEOUS_CONSTANT_B},
    //                                                                    {"Simultaneous-Average-Edge-B", FirstOrderXPBDSolveMode::SIMULTANEOUS_AVERAGE_EDGE_B},
    //                                                                    {"Simultaneous-Smallest-Edge-B", FirstOrderXPBDSolveMode::SIMULTANEOUS_SMALLEST_EDGE_B},
    //                                                                    {"Sequential", FirstOrderXPBDSolveMode::SEQUENTIAL},
    //                                                                    {"Simultaneous-Jacobi", FirstOrderXPBDSolveMode::SIMULTANEOUS_JACOBI},
    //                                                                    {"Simultaneous-Convergent-Jacobi", FirstOrderXPBDSolveMode::SIMULTANEOUS_CONVERGENT_JACOBI}};
    //     return solve_mode_options;
    // }

    // static std::map<std::string, FirstOrderXPBDResidualPolicy>& RESIDUAL_POLICY_OPTIONS()
    // {
    //     static std::map<std::string, FirstOrderXPBDResidualPolicy> residual_policy_options{{"Never", FirstOrderXPBDResidualPolicy::NEVER},
    //                                                                              {"Every-Substep", FirstOrderXPBDResidualPolicy::EVERY_SUBSTEP},
    //                                                                              {"Every-Iteration", FirstOrderXPBDResidualPolicy::EVERY_ITERATION}};
    //     return residual_policy_options;
    // }

    public:
    /** Creates a Config from a YAML node, which consists of the specialized parameters needed for XPBDMeshObject.
     * @param node : the YAML node (i.e. dictionary of key-value pairs) that information is pulled from
     */
    explicit FirstOrderXPBDMeshObjectConfig(const YAML::Node& node)
        : XPBDMeshObjectConfig(node)
    {
        // extract parameters
        // _extractParameter("num-solver-iters", node, _num_solver_iters, DEFAULT_NUM_SOLVER_ITERS());
        // _extractParameterWithOptions("solve-mode", node, _solve_mode, SOLVE_MODE_OPTIONS(), DEFAULT_SOLVE_MODE());
        // _extractParameterWithOptions("residual-policy", node, _residual_policy, RESIDUAL_POLICY_OPTIONS(), DEFAULT_RESIDUAL_POLICY());
        _extractParameter("damping-multiplier", node, _damping_multiplier, DEFAULT_DAMPING_MULTIPLIER());        
    }

    // Getters
    // std::optional<unsigned> numSolverIters() const { return _num_solver_iters.value; }
    // std::optional<FirstOrderXPBDSolveMode> solveMode() const { return _solve_mode.value; }
    // std::optional<Real> dampingStiffness() const { return _damping_stiffness.value; }
    // std::optional<FirstOrderXPBDResidualPolicy> residualPolicy() const { return _residual_policy.value; }
    std::optional<Real> dampingMultiplier() const { return _damping_multiplier.value; }
    // std::optional<Real> gScaling() const { return _g_scaling.value; }

    protected:
    // Parameters
    // ConfigParameter<unsigned> _num_solver_iters;
    // ConfigParameter<FirstOrderXPBDSolveMode> _solve_mode;
    // ConfigParameter<Real> _damping_stiffness;
    // ConfigParameter<FirstOrderXPBDResidualPolicy> _residual_policy;
    ConfigParameter<Real> _damping_multiplier;
};

#endif // __FIRST_ORDER_XPBD_MESH_OBJECT_CONFIG_HPP