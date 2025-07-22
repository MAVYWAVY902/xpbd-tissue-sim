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

/** General RK4 integration - but with iterator ranges
 * @param state0 - the initial state at t_start
 * @param t_start - starting iterator of the times t - *t_start should correspond to the time at state0
 * @param t_end - ending iterator of the times t - IT IS NOT INCLUDED in t
 * @param params - additional constant parameters for the integration
 * @param ode_func - a function that, given (time, state, params) returns the time derivative of the state
 * @param state_output_start - (OUTPUT) an iterator pointing to the start of an output states vector (state0 will go here)
 */
template<typename StateType, typename ParamType, typename TIterator, typename StateIterator>
void RK4(const StateType& state0, TIterator t_start, TIterator t_end,
    const ParamType& params, std::function<StateType (Real, const StateType&, const ParamType&)> ode_func,
    StateIterator state_output_start)
{
    int num_states = std::distance(t_start, t_end);
    std::vector<StateType> states(num_states);
    *(state_output_start) = state0;

    // run through each integration point
    for (int i = 1; i < num_states; i++)
    {
        const Real cur_t = *(t_start + i);
        const Real last_t = *(t_start + i-1);
        const Real h = cur_t - last_t;
        
        // RK4 algorithm
        const StateType k1 = ode_func(cur_t, *(state_output_start + i-1), params);
        const StateType k2 = ode_func(cur_t + 0.5*h, *(state_output_start + i-1) + 0.5*h*k1, params);
        const StateType k3 = ode_func(cur_t + 0.5*h, *(state_output_start + i-1) + 0.5*h*k2, params);
        const StateType k4 = ode_func(cur_t + h, *(state_output_start + i-1) + h*k3, params);

        *(state_output_start + i) = *(state_output_start + i-1) + h*(k1 + 2*k2 + 2*k3 + k4) / 6.0;
    }

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