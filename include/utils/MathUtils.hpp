#ifndef __MATH_UTILS_HPP
#define __MATH_UTILS_HPP

#include "common/types.hpp"

#include <functional>

template<typename StateType, typename ParamType>
StateType RK4(Real t, Real h, const StateType& cur_state, const ParamType& params, std::function<StateType (Real, const StateType&, const ParamType&)> ode_func)
{
    const StateType k1 = ode_func(t, cur_state, params);
    const StateType k2 = ode_func(t + 0.5*h, cur_state + 0.5*h*k1, params);
    const StateType k3 = ode_func(t + 0.5*h, cur_state + 0.5*h*k2, params);
    const StateType k4 = ode_func(t + h, cur_state + h*k3, params);

    return cur_state + h*(k1 + 2*k2 + 2*k3 + k4) / 6.0;
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