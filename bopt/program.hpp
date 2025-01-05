#ifndef OPTIMISATION_PROGRAM_H
#define OPTIMISATION_PROGRAM_H

#include <casadi/casadi.hpp>

#include "bopt/ad/casadi/codegen.hpp"
#include "bopt/binding.hpp"
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

template <class ValueType>
void getJacobianStructure(
    Eigen::SparseMatrix<ValueType> &J,
    std::vector<binding<constraint_base_tpl<ValueType>>> &bindings) {
    bopt_index cnt = bopt_index(0);
    for (auto &b : bindings) {
        // For each non-zero element of the jacobian, get their variable
        // coordinates and convert to their vector locations based on x

        bopt_index idx_x, idx_y;
        Eigen::SparseMatrix<ValueType> tmp;
        b.get()->get_sparsity_jacobian(tmp);

        // TODO - update jacobian pattern

        cnt += info.m;
    }
}

template <class MatrixContainer, class EvaluatorType,
          class MatrixElementInserter>
void getHessianStructure(MatrixContainer &H,
                         std::vector<binding<EvaluatorType>> &bindings,
                         const MatrixElementInserter &inserter) {
    // Typdefs
    typedef typename evaluator_traits<EvaluatorType>::index_type index_type;
    typedef typename evaluator_traits<EvaluatorType>::value_type value_type;
    typedef evaluator_out_info<EvaluatorType> evaluator_out_info_t;

    for (auto &b : bindings) {
        // For each non-zero element of the hessian, get their variable
        // coordinates and convert to their vector locations based on x
        index_type idx_x, idx_y;
        evaluator_out_info_t info;
        b.get()->jac_info(info);

        // Determine locations for each non-zero entry
        for (index_type col = 0; col < info.out_m; ++col) {
            index_type start =
                ccs_traits<evaluator_out_info_t>::indptr(info)[col];
            index_type end =
                ccs_traits<evaluator_out_info_t>::indptr(info)[col + 1];

            for (index_type row = start; row < end; ++row) {
                // Get index of entry
                index_type idx =
                    ccs_traits<evaluator_out_info_t>::indices(info)[row];
                // Add entry to full Jacobian
                inserter(H, b.get()->input_indices[0][idx],
                         b.get()->input_indices[0][col], value_type(0));
            }
        }
    }
}

/**
 * @brief Represents a generic mathematical program with constraints and costs.
 *
 * This class represents an optimisation problem of the form:
 * \f$ \min f(x) \text{ s.t. } g_l \le g(x) \le q_u, x_l \le x \le x_u \f$
 *
 * @tparam ValueType Type of values in the program (e.g., double).
 */
template <typename ValueType, typename VectorType, typename MatrixType>
class mathematical_program {
   public:
    friend class solvers::SolverBase;

    typedef ValueType value_type;
    typedef IntegerType integer_type;
    typedef IndexType index_type;

    typedef std::string string_type;

    typedef constraint_tpl<ValueType, IntegerType, IndexType, SparsityDataType>
        constraint_type;
    typedef cost_tpl<ValueType, IntegerType, IndexType, SparsityDataType>
        cost_type;

    /**
     * @brief Default constructor for the mathematical program.
     */
    mathematical_program() = default;

    /**
     * @brief Constructs a mathematical program with a specified name.
     *
     * @param name Name of the mathematical program.
     */
    mathematical_program(const string_type &name) : name_(name) {}

    /**
     * @brief Gets the name of the mathematical program.
     *
     * @return const string_type& Reference to the program's name.
     */
    const string_type &name() const { return name_; }

    /**
     * @brief Gets the number of decision variables in the program.
     *
     * @return index_type Number of decision variables.
     */
    index_type n_variables() const { return variables_.size(); }

    /**
     * @brief Gets the number of cost functions in the program.
     *
     * @return index_type Number of cost functions.
     */
    index_type n_costs() const { return get_all_costs().size(); }
    /**
     * @brief Gets the number of constraints in the program.
     *
     * @return index_type Number of constraints.
     */
    index_type n_constraints() const {
        index_type n = 0;
        for (const auto &c : get_all_constraints()) {
            typename constraint<value_type>::out_info_t info;
            c.get()->info(info);
            n += info.m;
        }
        return n;
    }

    /**
     * @brief Gets the initial values of the decision variables.
     *
     * @return const std::vector<value_type>& Reference to the vector of initial
     * values.
     */
    const std::vector<value_type> &variable_initial_values() const {
        return x0_;
    }

    /**
     * @brief Gets the bounds for the decision variables.
     *
     * @return const vector_bounds<value_type>& Reference to the variable
     * bounds.
     */
    const vector_bounds<value_type> &variable_bounds() const { return xb_; }

    /**
     * @brief Adds a decision variable to the program.
     *
     * @param v Variable to add.
     * @param v0 Initial value for the variable.
     * @param bl Lower bound for the variable.
     * @param bu Upper bound for the variable.
     */
    void add_variable(
        const variable &v, const value_type &v0 = value_type(0),
        const value_type &bl = -std::numeric_limits<value_type>::infinity(),
        const value_type &bu = std::numeric_limits<value_type>::infinity()) {
        variables_.push_back(v);
        variable_index_.push_back(variables_.size());

        // Update decision variable vector sizes
        x0_.emplace_back(v0);
        xb_.m_values.emplace_back(bound_element<value_type>(bl, bu));
    }

    void add_variables(const Eigen::VectorX<variable> &variables,
                       const Eigen::VectorX<value_type> &v0 = {},
                       const Eigen::VectorX<value_type> &bl = {},
                       const Eigen::VectorX<value_type> &bu = {}) {
        constexpr value_type inf = std::numeric_limits<value_type>::infinity();
        for (auto it = variables.begin(); it != variables.end(); ++it) {
            index_type i = std::distance(variables.begin(), it);
            value_type v0_i = value_type(0), bl_i = -inf, bu_i = inf;
            if (!v0.empty()) v0_i = v0[i];
            if (!bl.empty()) bl_i = bl[i];
            if (!bu.empty()) bu_i = bu[i];

            add_variable(*it, v0_i, bl_i, bu_i);
        }
    }

    index_type variable_index(const variable &v) const {
        const auto &it = std::find(variables_.begin(), variables_.end(), v);
        if (it != variables_.end()) {
            return std::distance(variables_.begin(), it);
        }
        throw std::runtime_error("Variable does not exist");
    }

    std::vector<index_type> variable_indices(
        const Eigen::VectorX<variable> &v) const {
        std::vector<index_type> indices = {};
        for (const auto &vi : v) {
            indices.emplace_back(variable_index(vi));
        }
        return indices;
    }

    vector_bounds<value_type> &variable_bounds() { return xb_; }

    void setDecisionvariableInitialValue(const variable &v,
                                         const value_type &val) {
        set(x0_, variable_index(v), val);
    }

    void setDecisionvariableBounds(const variable &v, const value_type &lb,
                                   const value_type &ub) {
        set(xb_.upper, variable_index(v), ub);
        set(xb_.lower, variable_index(v), lb);
    }

    input_index_vector create_input_index_vector(
        const Eigen::Ref<const Eigen::VectorX<variable>> &x) {
        input_index_vector indices = {};
        for (Eigen::Index i = 0; i < x.size(); ++i) {
            indices.push_back(variable_indices(x[i]));
        }

        return indices;
    }

    // costs
    void add_cost(const typename cost_type::shared_ptr &cost,
                  const input_variable_vector &x) {
        // Create binding
        input_index_vector indices = create_input_index_vector(x);
        // Create binding
        costs_generic_.push_back(binding<cost_t>(cost, indices));
    }

    void add_linear_cost(const typename linear_cost_type::shared_ptr &cost,
                         const input_variable_vector &x) {
        // Create binding
        input_index_vector indices = create_input_index_vector(x);
        // Create binding
        costs_linear_.push_back(binding<linear_cost_type>(cost, indices));
    }

    void add_quadratic_cost(
        const typename quadratic_cost_type::shared_ptr &cost,
        const input_variable_vector &x) {
        // Create binding
        input_index_vector indices = create_input_index_vector(x);
        // Create binding
        costs_quadratic_.push_back(binding<quadratic_cost_type>(cost, indices));
    }

    std::vector<binding<cost_type>> &generic_costs() { return costs_generic_; }

    std::vector<binding<linear_cost_type>> &linear_costs() {
        return costs_linear_;
    }

    std::vector<binding<quadratic_cost_type>> &quadratic_costs() {
        return costs_quadratic_;
    }

    std::vector<binding<cost_t>> get_all_costs() const {
        std::vector<binding<cost_t>> vec;
        vec.insert(vec.begin(), costs_generic_.begin(), costs_generic_.end());
        vec.insert(vec.end(), costs_linear_.begin(), costs_linear_.end());
        vec.insert(vec.end(), costs_quadratic_.begin(), costs_quadratic_.end());
        // Return vector of all costs
        return vec;
    }

    // constraints

    typedef constraint<value_type> constraint_t;
    typedef linear_constraint<value_type> linear_constraint_t;
    typedef bounding_box_constraint<value_type> bounding_box_constraint_t;

    void add_constraint(
        const typename constraint<value_type>::shared_ptr &constraint,
        const input_variable_vector &x) {
        // Create binding
        input_index_vector indices = create_input_index_vector(x);
        // Create binding
        add_constraint_binding(binding<constraint_t>(constraint, indices));
    }

    void add_linear_constraint(
        const typename linear_constraint<value_type>::shared_ptr &constraint,
        const input_variable_vector &x) {
        // Create binding
        input_index_vector indices = create_input_index_vector(x);
        // Create binding
        constraints_linear_.push_back(
            binding<linear_constraint_t>(constraint, indices));
    }

    void add_bounding_box_constraint(
        const typename bounding_box_constraint<value_type>::shared_ptr
            &constraint,
        const input_variable_vector &x) {
        // Create binding
        input_index_vector indices = create_input_index_vector(x);
        // Create binding
        constraints_bounding_box_.push_back(
            binding<bounding_box_constraint_t>(constraint, indices));
    }

    std::vector<binding<constraint_t>> &generic_constraints() {
        return constraints_generic_;
    }
    std::vector<binding<linear_constraint_t>> &linear_constraints() {
        return constraints_linear_;
    }

    std::vector<binding<bounding_box_constraint_t>> &
    bounding_box_constraints() {
        return constraints_bounding_box_;
    }

    std::vector<binding<constraint_t>> get_all_constraints() const {
        std::vector<binding<constraint_t>> vec;
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
    string_type name_;

    // Decision variables initial value
    std::vector<value_type> x0_;
    // Decision variable bounds
    vector_bounds<value_type> xb_;

    std::vector<variable> variables_;
    std::vector<index_type> variable_index_;
    std::vector<value_type> variable_value_;

    // constraint bindings
    std::vector<binding<constraint_t>> constraints_generic_ = {};
    std::vector<binding<linear_constraint_t>> constraints_linear_ = {};
    std::vector<binding<bounding_box_constraint_t>> constraints_bounding_box_ =
        {};

    // cost bindings
    std::vector<binding<cost_t>> costs_generic_ = {};
    std::vector<binding<linear_cost_t>> costs_linear_ = {};
    std::vector<binding<quadratic_cost_t>> costs_quadratic_ = {};
};

}  // namespace bopt

#endif /* OPTIMISATION_PROGRAM_H */
