#ifndef __MATH_UTILS_HPP
#define __MATH_UTILS_HPP

#include "common/types.hpp"

#include <functional>

/** General RK4 integration
 * @param state0 - the initial state at t[0]
 * @param t - a vector of integration points (t[0] corresponds to the time at state0)
 * @param params - additional constant parameters for the integration
 * @param ode_func - a function that, given (time, state, params) returns the time derivative of the state
 * @returns the states at each of the integration points
 */
template<typename StateType, typename ParamType>
std::vector<StateType> RK4(const StateType& state0, const std::vector<Real>& t, const ParamType& params, std::function<StateType (Real, const StateType&, const ParamType&)> ode_func)
{
    std::vector<StateType> states(t.size());
    states[0] = state0;

    // run through each integration point
    for (unsigned i = 1; i < t.size(); i++)
    {
        const Real h = t[i] - t[i-1];
        
        // RK4 algorithm
        const StateType k1 = ode_func(t[i], states[i-1], params);
        const StateType k2 = ode_func(t[i] + 0.5*h, states[i-1] + 0.5*h*k1, params);
        const StateType k3 = ode_func(t[i] + 0.5*h, states[i-1] + 0.5*h*k2, params);
        const StateType k4 = ode_func(t[i] + h, states[i-1] + h*k3, params);

        states[i] = states[i-1] + h*(k1 + 2*k2 + 2*k3 + k4) / 6.0;
    }

    return states;

}

Mat3r Skew3(const Vec3r& vec)
{
    Mat3r mat;
    mat << 0,       -vec(2),    vec(1),
           vec(2),  0,          -vec(0),
           -vec(1), vec(0),     0;
    return mat;
}

#endif // __MATH_UTILS_HPP