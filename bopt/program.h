#ifndef OPTIMISATION_PROGRAM_H
#define OPTIMISATION_PROGRAM_H

#include <casadi/casadi.hpp>

#include "bopt/ad/casadi/codegen.hpp"
#include "bopt/ad/casadi/eigen.hpp"
#include "bopt/ad/casadi/function.hpp"
#include "bopt/binding.h"
#include "bopt/common.hpp"
#include "bopt/constraints.hpp"
#include "bopt/costs.hpp"
#include "bopt/logging.hpp"
#include "bopt/profiler.hpp"

namespace bopt {

// Forward declaration of SolverBase
namespace solvers {
class SolverBase;
}

template <class MatrixContainer, class BindingType, class MatrixElementInserter>
void getJacobianStructure(MatrixContainer &J,
                          std::vector<Binding<BindingType>> &bindings,
                          const variableIndexMap &index_map,
                          const MatrixElementInserter &inserter) {
    // Typdefs
    typedef typename evaluator_traits<BindingType>::index_type index_type;
    typedef typename evaluator_traits<BindingType>::value_type value_type;

    index_type cnt = index_type(0);
    for (auto &b : bindings) {
        // For each non-zero element of the hessian, get their variable
        // coordinates and convert to their vector locations based on x
        index_type idx_x, idx_y;
        evaluator_out_info<BindingType> info;
        b.get()->jac_info(info);

        // Determine locations for each non-zero entry
        for (index_type col = 0; col < info.out_m; ++col) {
            index_type start = indptr[col];
            index_type end = indptr[col + 1];

            for (index_type row = start; row < end; ++row) {
                // Get index of entry
                index_type idx = ind[row];
                // Add entry to full Jacobian
                inserter(J, index_map.index(idx), cnt + col, value_type(0));
            }
        }

        cnt += info.m;
    }
}

template <class MatrixContainer, class BindingType, class MatrixElementInserter>
void getHessianStructure(MatrixContainer &H,
                         std::vector<Binding<BindingType>> &bindings,
                         const variableIndexMap &index_map,
                         const MatrixElementInserter &inserter) {
    // Typdefs
    typedef typename evaluator_traits<BindingType>::index_type index_type;
    typedef typename evaluator_traits<BindingType>::value_type value_type;

    for (auto &b : bindings) {
        // For each non-zero element of the hessian, get their variable
        // coordinates and convert to their vector locations based on x
        index_type idx_x, idx_y;
        evaluator_out_info<BindingType> info;
        b.get()->jac_info(info);

        // Determine locations for each non-zero entry
        for (index_type col = 0; col < info.out_m; ++col) {
            index_type start = indptr[col];
            index_type end = indptr[col + 1];

            for (index_type row = start; row < end; ++row) {
                // Get index of entry
                index_type idx = ind[row];
                // Add entry to full Jacobian
                inserter(H, index_map.index(idx), cnt + col, value_type(0));
            }
        }
    }
}

/**
 * @brief Represents a generic mathematical program of the form \f$ \min f(x, p)
 * s.t. g_l \le g(x) \le q_u, x_l \le x \le x_u \f$
 *
 */
template <typename T = double>
class MathematicalProgram : public ConstraintBindingSet, public CostBindingSet {
   public:
    friend class solvers::SolverBase;

    typedef T value_type;

    using Index = std::size_t;
    using String = std::string;

    MathematicalProgram() = default;
    MathematicalProgram(const String &name) : name_(name) {}

    /**
     * @brief Name of the program
     *
     * @return const String&
     */
    const String &name() const { return name_; }

    Index numberOfDecisionvariables() const { return x_idx_map_.size(); }

    Index numberOfParameters() const { return p_idx_map_.size(); }

    /**
     * @brief Decision variables initial values
     *
     * @return const std::vector<double>&
     */
    const std::vector<double> &x0() const { return x0_; }

    /**
     * @brief Decision variables upper bound vector
     *
     * @return const std::vector<double>&
     */
    const std::vector<double> &xbu() const { return xb_.upper; }

    /**
     * @brief Decision variables lower bound vector
     *
     * @return const std::vector<double>&
     */
    const std::vector<double> &xbl() const { return xb_.lower; }

    /**
     * @brief Parameter vector
     *
     * @return const std::vector<double>&
     */
    const std::vector<double> &p() const { return p_; }

    void add_variable(
        const variable &v, const double &v0 = 0.0,
        const double &bl = -std::numeric_limits<double>::infinity(),
        const double &bu = std::numeric_limits<double>::infinity()) {
        variables_.push_back(v);
        variable_index_.push_back(variables_.size());

        // Update decision variable vector sizes
        x0_.emplace_back(v0);
        xb_.lower.emplace_back(bl);
        xb_.upper.emplace_back(bu);
    }

    void add_parameter(const variable &p, const double &val = 0.0) {
        p_idx_map_.add(p);
        // Update parameter vector sizes
        p_.emplace_back(val);
    }

    const Index &variable_index(const variable &v) const {
        return x_idx_map_.index(v);
    }

    std::vector<Index> getDecisionvariableIndices(
        const variable_vector_t &x) const {
        std::vector<Index> indices = {};
        for (const auto &xi : x) {
            indices.emplace_back(getDecisionvariableIndex(xi));
        }
        return indices;
    }

    const Index &getParameterIndex(const variable &p) const {
        return p_idx_map_.index(p);
    }

    std::vector<Index> getParameterIndices(const variable_vector_t &p) const {
        std::vector<Index> indices = {};
        for (const auto &pi : p) {
            indices.emplace_back(getParameterIndex(pi));
        }
        return indices;
    }

    void set_parameter(const variable &p, const value_type &val) {
        set(p_, parameter_index(p), val);
    }

    void setDecisionvariableInitialValue(const variable &v,
                                         const value_type &val) {
        set(x0_, variable_index(v), val);
    }

    void setDecisionvariableBounds(const variable &v, const value_type &lb,
                                   const value_type &ub) {
        set(xb_.upper, variable_index(v), ub);
        set(xb_.lower, variable_index(v), lb);
    }

    // Costs

    typedef typename cost_traits<cost<value_type>>::ptr_type cost_ptr;
    typedef typename cost_traits<cost<value_type>>::ptr_type cost_input;

    void addCost(const cost_ptr &c, const cost_input &in) {
        // Create a binding for the constraint
        costs_generic_.push_back(Binding<cost<value_type>>(con, in));
    }

    typedef typename std::vector<Binding<LinearCost<value_type>>>
        linear_cost_vector;

    const linear_cost_vector &linearCosts() const { return costs_linear_; }

    typedef
        typename cost_traits<LinearCost<value_type>>::ptr_type linear_cost_ptr;

    void addLinearCost(const linear_cost_ptr &c, const cost_input &in) {
        // Create a binding for the constraint
        costs_linear_.push_back(Binding<LinearCost<value_type>>(con, in));
    }

    std::vector<Binding<Cost>> getAllCosts() const {
        std::vector<Binding<Cost>> vec;
        vec.insert(vec.begin(), costs_generic_.begin(), costs_generic_.end());
        vec.insert(vec.end(), costs_linear_.begin(), costs_linear_.end());
        vec.insert(vec.end(), quadratic_.begin(), quadratic_.end());
        // Return vector of all costs
        return vec;
    }

    // Constraints

    typedef typename constraint_traits<constraint<value_type>>::ptr_type
        constraint_ptr;
    typedef typename constraint_traits<constraint<value_type>>::ptr_type
        constraint_input;

    void addConstraint(const constraint_ptr &c, const constraint_input &in) {
        // Create a binding for the constraint
        constraints_generic_.push_back(
            Binding<constraint<value_type>>(con, in));
    }

    typedef typename std::vector<Binding<LinearConstraint<value_type>>>
        linear_constraint_vector;

    const linear_constraint_vector &linearCosts() const {
        return constraints_linear_;
    }

    typedef typename constraint_traits<LinearConstraint<value_type>>::ptr_type
        linear_constraint_ptr;

    void addLinearConstraint(const linear_constraint_ptr &c,
                             const cost_input &in) {
        // Create a binding for the constraint
        constraints_linear_.push_back(
            Binding<LinearConstraint<value_type>>(con, in));
    }

    std::vector<Binding<Constraint>> getAllConstraints() const {
        std::vector<Binding<Constraint>> vec;
        vec.insert(vec.begin(), constraints_generic_.begin(),
                   constraints_generic_.end());
        vec.insert(vec.end(), constraints_linear_.begin(),
                   constraints_linear_.end());
        // Return vector of all costs
        return vec;
    }

   protected:
   private:
    // Probably need a specialised solver or at least a virtual method here
    // where it can be overwritten or something?

    // Name
    String name_;

    // Decision variables initial value
    std::vector<value_type> x0_;
    // Decision variable bounds
    Bounds<value_type> xb_;

    std::vector<variable_t> variables_;
    std::vector<std::size_t> variable_index_;

    // Parameter values
    std::vector<double> p_;

    std::vector<variable_t> parameters_;
    std::vector<std::size_t> parameter_index_;

    // Constraint bindings
    std::vector<Binding<constraint<value_type>>> constraints_generic_;
    std::vector<Binding<LinearConstraint<value_type>>> constraints_linear_;

    // Cost bindings
    std::vector<Binding<cost<value_type>>> costs_generic_;
    std::vector<Binding<LinearCost<value_type>>> costs_linear_;
    std::vector<Binding<QuadraticCost<value_type>>> quadratic_;
};

}  // namespace bopt

#endif /* OPTIMISATION_PROGRAM_H */
