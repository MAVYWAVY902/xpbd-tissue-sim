#ifndef __CONSTRAINT_PROJECTOR_DECORATOR_HPP
#define __CONSTRAINT_PROJECTOR_DECORATOR_HPP

#include "solver/ConstraintProjector.hpp"
#include "simobject/FirstOrderXPBDMeshObject.hpp"
#include <memory>
#include <iomanip>

namespace Solver
{

// class ConstraintProjectorDecorator : public ConstraintProjector
// {
//     public:
//     explicit ConstraintProjectorDecorator(std::unique_ptr<ConstraintProjector> component)
//         : ConstraintProjector(*component), 
//         _component(std::move(component))
//     {
//     }

//     virtual void setLambda(const Eigen::VectorXd& new_lambda) override { _component->setLambda(new_lambda); _lambda = new_lambda; }

//     protected:
//     void _componentLHS(const double* delC_ptr, const double* M_inv_ptr, const double* alpha_tilde_ptr, double* lhs_ptr) const
//     {
//         return _component->_LHS(delC_ptr, M_inv_ptr, alpha_tilde_ptr, lhs_ptr);
//     }

//     void _componentRHS(const double* C_ptr, const double* alpha_tilde_ptr, double* rhs_ptr) const
//     {
//         return _component->_RHS(C_ptr, alpha_tilde_ptr, rhs_ptr);
//     }

//     void _componentGetPositionUpdate(const unsigned position_index, const double* delC_ptr, const double inv_m, const double* dlam_ptr, double* pos_update_ptr) const
//     {
//         return _component->_getPositionUpdate(position_index, delC_ptr, inv_m, dlam_ptr, pos_update_ptr);
//     }

//     std::unique_ptr<ConstraintProjector> _component;
// };


/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////


// class WithDamping : public ConstraintDecorator
// { 
//     public:
//     WithDamping(std::unique_ptr<Constraint> component, double gamma)
//         : ConstraintDecorator(std::move(component)), _damping_gamma(gamma)
//     {
//     }

//     virtual bool usesDamping() const override { return true; }

//     protected:
//     double _LHS(const ValueAndGradient& val_and_grad) const
//     {
//         const double alpha_tilde = alphaTilde();
//         const double delC_Minv_delC = _componentLHS(val_and_grad) - alpha_tilde;
//         return (1 + _damping_gamma) * delC_Minv_delC + alpha_tilde;
//     }

//     double _RHS(const ValueAndGradient& val_and_grad) const
//     {
//         const double alpha_tilde = alphaTilde();

//         const Eigen::VectorXd& grad = val_and_grad.second;
//         double delC_x_prev = 0;
//         for (unsigned i = 0; i < _positions.size(); i++)
//         {
//             delC_x_prev += grad(Eigen::seq(3*i, 3*i+2)).dot(_positions[i].position() - _positions[i].previousPosition());
//         }

//         double rhs = _componentRHS(val_and_grad) - _damping_gamma * delC_x_prev;
//         return rhs;
//     }

//     private:
//     double _damping_gamma;
// };


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////


// class WithPrimaryResidual : public ConstraintDecorator
// {
    
//     public:
//     WithPrimaryResidual(std::unique_ptr<Constraint> component)
//         : ConstraintDecorator(std::move(component))
//     {
//     }

//     void setPrimaryResidual(Eigen::VectorXd const* res_ptr) { _res_ptr = res_ptr; }
//     virtual bool usesPrimaryResidual() const override { return true; }

//     protected:
//     inline virtual double _RHS(const ValueAndGradient& val_and_grad) const
//     {
//         const Eigen::VectorXd& grad = val_and_grad.second;
//         double delC_g = 0;
//         for (unsigned i = 0; i < _positions.size(); i++)
//         {
//             const double inv_m = positionInvMass(i);
//             delC_g += inv_m*(grad(Eigen::seq(3*i, 3*i+2)).dot(_scaledResidual(i)));
//         }

//         const double rhs = _componentRHS(val_and_grad) + delC_g;
//         return rhs;
//     }

//     inline virtual PositionUpdate _getPositionUpdate(const unsigned position_index, const double dlam, const Eigen::VectorXd& grad) const
//     {
//         PositionUpdate position_update = _componentGetPositionUpdate(position_index, dlam, grad);
//         position_update.second -= positionInvMass(position_index) * _scaledResidual(position_index);
//         return position_update;
//     }

//     private:
//     inline virtual Eigen::Vector3d _scaledResidual(const unsigned position_index) const
//     {
//         assert(_res_ptr);
//         unsigned v_ind = _positions[position_index].index;
//         const Eigen::Vector3d& g = (*_res_ptr)(Eigen::seq(3*v_ind,3*v_ind+2));

//         // TODO: replace number of attached elements with number of "attached" constraints for more generality
//         return g / (2*_positions[position_index].attachedElements());
//     }

//     private:
//     Eigen::VectorXd const* _res_ptr;
// };


////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////


// class FirstOrder : public ConstraintDecorator
// {
//     public:
//     FirstOrder(std::unique_ptr<Constraint> component)
//         : ConstraintDecorator(std::move(component))
//     {
//         // make sure all positions are part of FirstOrderXPBDMeshObject, and store their objs for future use
//         const std::vector<PositionReference>& positions = _component->positions();
//         _fo_objs.resize(positions.size());
//         for (unsigned i = 0; i < positions.size(); i++)
//         {
//             if (FirstOrderXPBDMeshObject* fo_obj = dynamic_cast<FirstOrderXPBDMeshObject*>(positions[i].obj))
//             {
//                 _fo_objs.at(i) = fo_obj;
//             }
//             else
//             {
//                 std::cout << positions[i].obj->name() << " of type " << positions[i].obj->type() << " is not a FirstOrderXPBDMeshObject!" << std::endl;
//                 assert(0);
//             }
//         }
//     }

//     inline virtual double alphaTilde() const override
//     {
//         return _alpha / _dt;
//     }

//     inline virtual double positionInvMass(const unsigned position_index) const override
//     {
//         return _fo_objs[position_index]->vertexInvDamping(_positions[position_index].index);
//     }

//     private:
//     std::vector<FirstOrderXPBDMeshObject*> _fo_objs;
// };

} // namespace Solver


#endif // __CONSTRAINT_DECORATOR_HPP