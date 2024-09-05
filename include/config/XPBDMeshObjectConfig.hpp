#ifndef __XPBD_MESH_OBJECT_CONFIG_HPP
#define __XPBD_MESH_OBJECT_CONFIG_HPP

#include "config/ElasticMeshObjectConfig.hpp"

/** Dictates different 'solve modes' for XPBD
 * SIMULTANEOUS: solves the hydrostatic and deviatoric constraints for each element simultaneously as a 2x2 system. Still iterates sequentially through each element.
 * SEQUENTIAL: solves the hydrostatic and deviatoric constraints for each element sequentially in true Gauss-Seidel fashion
 */
enum XPBDSolveMode
{
    SIMULTANEOUS,
    SEQUENTIAL,
    CONSTANTX,
    SEQUENTIAL_RANDOMIZED,
    SEQUENTIAL_INIT_LAMBDA,
    SIMULTANEOUS_INIT_LAMBDA,
    RUCKER_FULL,
    SPLIT_DEVIATORIC_SEQUENTIAL
};

class XPBDMeshObjectConfig : public ElasticMeshObjectConfig
{
    /** Static predefined default for the number of solver iterations */
    static std::optional<unsigned>& DEFAULT_NUM_SOLVER_ITERS() { static std::optional<unsigned> num_solver_iters(1); return num_solver_iters; }
    /** Static predefined default for solve mode */
    static std::optional<XPBDSolveMode>& DEFAULT_SOLVE_MODE() { static std::optional<XPBDSolveMode> solve_mode(XPBDSolveMode::SEQUENTIAL); return solve_mode; }
    /** Static predefined default for damping stiffness */
    static std::optional<double>& DEFAULT_DAMPING_STIFFNESS() { static std::optional<double> damping_stiffness(0); return damping_stiffness; }

    static std::map<std::string, XPBDSolveMode>& SOLVE_MODE_OPTIONS() 
    {
        static std::map<std::string, XPBDSolveMode> solve_mode_options{{"Simultaneous", XPBDSolveMode::SIMULTANEOUS},
                                                                       {"Sequential", XPBDSolveMode::SEQUENTIAL},
                                                                       {"ConstantX", XPBDSolveMode::CONSTANTX},
                                                                       {"Sequential-Randomized", XPBDSolveMode::SEQUENTIAL_RANDOMIZED},
                                                                       {"Sequential-Init-Lambda", XPBDSolveMode::SEQUENTIAL_INIT_LAMBDA},
                                                                       {"Simultaneous-Init-Lambda", XPBDSolveMode::SIMULTANEOUS_INIT_LAMBDA},
                                                                       {"Rucker-Full", XPBDSolveMode::RUCKER_FULL},
                                                                       {"Split-Deviatoric-Sequential", XPBDSolveMode::SPLIT_DEVIATORIC_SEQUENTIAL}};
        return solve_mode_options;
    }

    public:
    /** Creates a Config from a YAML node, which consists of the specialized parameters needed for XPBDMeshObject.
     * @param node : the YAML node (i.e. dictionary of key-value pairs) that information is pulled from
     */
    explicit XPBDMeshObjectConfig(const YAML::Node& node)
        : ElasticMeshObjectConfig(node)
    {
        // extract parameters
        _extractParameter("num-solver-iters", node, _num_solver_iters, DEFAULT_NUM_SOLVER_ITERS());
        _extractParameterWithOptions("solve-mode", node, _solve_mode, SOLVE_MODE_OPTIONS(), DEFAULT_SOLVE_MODE());
        _extractParameter("damping-stiffness", node, _damping_stiffness, DEFAULT_DAMPING_STIFFNESS());
    }

    // Getters
    std::optional<unsigned> numSolverIters() const { return _num_solver_iters.value; }
    std::optional<XPBDSolveMode> solveMode() const { return _solve_mode.value; }
    std::optional<double> dampingStiffness() const { return _damping_stiffness.value; }

    protected:
    // Parameters
    ConfigParameter<unsigned> _num_solver_iters;
    ConfigParameter<XPBDSolveMode> _solve_mode;
    ConfigParameter<double> _damping_stiffness;
};

#endif // __XPBD_MESH_OBJECT_CONFIG_HPP