#ifndef __CONSTRAINT_PROJECTOR_DECORATOR_HPP
#define __CONSTRAINT_PROJECTOR_DECORATOR_HPP

#include "solver/ConstraintProjector.hpp"
#include "simobject/FirstOrderXPBDMeshObject.hpp"
#include <memory>
#include <iomanip>

namespace Solver
{

class ConstraintProjectorDecorator : public ConstraintProjector
{
    public:
    explicit ConstraintProjectorDecorator(std::unique_ptr<ConstraintProjector> component)
        : ConstraintProjector(*component), 
        _component(std::move(component))
    {
    }

    virtual void setLambda(const unsigned index, const double val) override { _component->setLambda(index, val); _lambda[index] = val; }

    protected:
    void _componentLHS(const double* delC_ptr, const double* M_inv_ptr, const double* alpha_tilde_ptr, double* lhs_ptr) const
    {
        return _component->_LHS(delC_ptr, M_inv_ptr, alpha_tilde_ptr, lhs_ptr);
    }

    void _componentRHS(const double* C_ptr, const double* delC_ptr, const double* alpha_tilde_ptr, double* rhs_ptr) const
    {
        return _component->_RHS(C_ptr, delC_ptr, alpha_tilde_ptr, rhs_ptr);
    }

    void _componentGetPositionUpdate(const unsigned position_index, const double* delC_ptr, const double inv_m, const double* dlam_ptr, double* pos_update_ptr) const
    {
        return _component->_getPositionUpdate(position_index, delC_ptr, inv_m, dlam_ptr, pos_update_ptr);
    }

    std::unique_ptr<ConstraintProjector> _component;
};


/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////


class WithDamping : public ConstraintProjectorDecorator
{ 
    public:
    WithDamping(std::unique_ptr<ConstraintProjector> component, double gamma)
        : ConstraintProjectorDecorator(std::move(component)), _damping_gamma(gamma)
    {
    }

    virtual bool usesDamping() const override { return true; }

    protected:
    inline virtual void _LHS(const double* delC_ptr, const double* M_inv_ptr, const double* alpha_tilde_ptr, double* lhs_ptr) override
    {
        _componentLHS(delC_ptr, M_inv_ptr, alpha_tilde_ptr, lhs_ptr);
        for (unsigned ci = 0; ci < numConstraints(); ci++)          // ci is row index (LHS matrix is column-major)
        {
            for (unsigned cj = 0; cj < numConstraints(); cj++)      // cj is column index
            {
                // if gammas are different for each constraint, this will be the damping gamma for constraint ci (gammas are the same across a row of LHS)
                const double gamma = _damping_gamma;    // right now, gamma is the same for all constraints
                if (ci == cj)
                {
                    lhs_ptr[cj*numConstraints() + ci] = (1 + gamma) * (lhs_ptr[cj*numConstraints() + ci] - alpha_tilde_ptr[ci]) + alpha_tilde_ptr[ci];
                }
                else
                {
                    lhs_ptr[cj*numConstraints() + ci] *= (1 + gamma);
                }
            }
        }
    }

    inline virtual void _RHS(const double* C_ptr, const double* delC_ptr, const double* alpha_tilde_ptr, double* rhs_ptr) override
    {
        _componentRHS(C_ptr, delC_ptr, alpha_tilde_ptr, rhs_ptr);

        
        for (unsigned ci = 0; ci < numConstraints(); ci++)
        {
            double delC_x_prev = 0;
            const double* delC_i = delC_ptr + numCoordinates()*ci;
            for (unsigned pi = 0; pi < numPositions(); pi++)
            {
                const double* pos = _positions[pi].position_ptr;
                const double* prev_pos = _positions[pi].prev_position_ptr;
                delC_x_prev += delC_i[3*pi]*(pos[0] - prev_pos[0]) + delC_i[3*pi+1]*(pos[1] - prev_pos[1]) + delC_i[3*pi+2]*(pos[2] - prev_pos[2]);
            }

            rhs_ptr[ci] -= _damping_gamma * delC_x_prev;
        }
    }

    private:
    double _damping_gamma;
};


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////


class WithPrimaryResidual : public ConstraintProjectorDecorator
{
    
    public:
    WithPrimaryResidual(std::unique_ptr<ConstraintProjector> component)
        : ConstraintProjectorDecorator(std::move(component))
    {
    }

    void setPrimaryResidual(const double* res_ptr) { _res_ptr = res_ptr; }
    virtual bool usesPrimaryResidual() const override { return true; }

    protected:
    inline virtual void _RHS(const double* C_ptr, const double* delC_ptr, const double* alpha_tilde_ptr, double* rhs_ptr) override
    {
        _componentRHS(C_ptr, delC_ptr, alpha_tilde_ptr, rhs_ptr);
        for (unsigned ci = 0; ci < numConstraints(); ci++)
        {
            const double* delC_i = delC_ptr + numCoordinates()*ci;        // pointer to the delC vector of the ith constraint (1 x numCoordinates)
            for (unsigned pi = 0; pi < numPositions(); pi++)
            {
                const double inv_m = _positions[pi].inv_mass;
                const unsigned index = _positions[pi].index;
                const double g_scaling = static_cast<double>(_positions[pi].num_constraints);
                rhs_ptr[ci] += inv_m * (delC_ptr[3*pi]*_res_ptr[3*index]/g_scaling + delC_ptr[3*pi+1]*_res_ptr[3*index+1]/g_scaling + delC_ptr[3*pi+2]*_res_ptr[3*index+2]/g_scaling);
            }
        }
    }

    /** Computes the position update (i.e. Delta x = M^-1 * delC^T * Delta lambda) for the given position index. Expects that Delta lambda, delC, and M^-1 have been computed already.
     * @param position_index - the index of the position to calculate the update for in the _positions member vector.
     * @param delC_ptr - the pointer to the delC matrix. Expects it to be row-major and numConstraints x numCoordinates.
     * @param inv_m - the inverse mass associated with this position.
     * @param dlam_ptr - the pointer to the Delta lambda vector. Expects it to be numConstraints x 1.
     * @param pos_update_ptr (OUTPUT) - the pointer to the (currently empty) position update vector. Expects it to be a 3-vector.
     */
    inline virtual void _getPositionUpdate(const unsigned position_index, const double* delC_ptr, const double inv_m, const double* dlam_ptr, double* pos_update_ptr) const override
    {
        _componentGetPositionUpdate(position_index, delC_ptr, inv_m, dlam_ptr, pos_update_ptr);
        const unsigned index = _positions[position_index].index;
        const double g_scaling = static_cast<double>(_positions[position_index].num_constraints);
        pos_update_ptr[0] -= inv_m * _res_ptr[3*index]/g_scaling;
        pos_update_ptr[1] -= inv_m * _res_ptr[3*index+1]/g_scaling;
        pos_update_ptr[2] -= inv_m * _res_ptr[3*index+2]/g_scaling;
    }

    private:
    const double* _res_ptr;
};


////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////


class FirstOrder : public ConstraintProjectorDecorator
{
    public:
    FirstOrder(std::unique_ptr<ConstraintProjector> component)
        : ConstraintProjectorDecorator(std::move(component))
    {
        // make sure all positions are part of FirstOrderXPBDMeshObject, and store their objs for future use
        _fo_objs.resize(numPositions());
        for (unsigned i = 0; i < numPositions(); i++)
        {
            if (FirstOrderXPBDMeshObject* fo_obj = dynamic_cast<FirstOrderXPBDMeshObject*>(_positions[i].obj))
            {
                _fo_objs.at(i) = fo_obj;
                _positions[i].inv_mass = fo_obj->vertexInvDamping(_positions[i].index);
            }
            else
            {
                std::cout << _positions[i].obj->name() << " of type " << _positions[i].obj->type() << " is not a FirstOrderXPBDMeshObject!" << std::endl;
                assert(0);
            }
        }
    }

    inline virtual void alphaTilde(double* alpha_tilde_ptr) const override
    {
        for (unsigned i = 0; i < numConstraints(); i++)
        {
            alpha_tilde_ptr[i] = _constraints[i]->alpha() / _dt;
        }
    }

    private:
    std::vector<FirstOrderXPBDMeshObject*> _fo_objs;
};

} // namespace Solver


#endif // __CONSTRAINT_DECORATOR_HPP