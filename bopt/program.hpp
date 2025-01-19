#pragma once

#include "bopt/binding.hpp"
#include "bopt/common.hpp"
#include "bopt/constraints.hpp"
#include "bopt/costs.hpp"
#include "bopt/logging.hpp"
#include "bopt/profiler.hpp"

namespace bopt {

template <class ValueType>
void get_constraint_jacobian(
    Eigen::SparseMatrix<ValueType> &jacobian, const bopt_index &cols,
    const std::vector<binding<constraint_tpl<ValueType>>> &bindings) {
    bopt_index cnt = bopt_index(0);
    std::vector<Eigen::Triplet<ValueType>> triplets;
    for (auto &b : bindings) {
        // todo - check if there is a sparse implementation, if not, assume it
        // todo - is dense

        // For each non-zero element of the jacobian, get their variable
        // coordinates and convert to their vector locations based on x
        typename constraint_tpl<ValueType>::sparse_matrix_t &jac =
            b.get()->buffer_jacobian().sparse;
        b.get()->sparsity_jacobian(jac);

        // Iterate over non-zeros
        for (int k = 0; k < jac.outerSize(); ++k) {
            for (Eigen::SparseMatrix<double>::InnerIterator it(jac, k); it;
                 ++it) {
                triplets.push_back(Eigen::Triplet<double>(
                    cnt + it.row(), b.indices().indices()[it.col()], 1.0));
            }
        }

        cnt += b.get()->sz_out();
    }
    // Create constraint jacobian
    jacobian.resize(cnt, cols);
    jacobian.setFromTriplets(triplets.begin(), triplets.end());
}

template <class ValueType>
void eval_constraint_jacobian(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    Eigen::SparseMatrix<ValueType> &jacobian,
    const std::vector<binding<constraint_tpl<ValueType>>> &bindings) {
    bopt_index cnt = bopt_index(0);
    for (auto &b : bindings) {
        typename constraint_tpl<ValueType>::sparse_matrix_t &jac =
            b.get()->buffer_jacobian().sparse;

        Eigen::Ref<const Eigen::VectorXd> xi = x(b.indices().indices());

        if (b.get()->eval_jacobian(xi, jac) ==
            evaluator::return_status::NotImplemented) {
        }

        // Iterate over non-zeros
        for (int k = 0; k < jac.outerSize(); ++k) {
            for (Eigen::SparseMatrix<double>::InnerIterator it(jac, k); it;
                 ++it) {
                // todo - speed this up
                jacobian.coeffRef(cnt + it.row(),
                                  b.indices().indices()[it.col()]) = it.value();
            }
        }

        cnt += b.get()->sz_out();
    }
}

/**
 * @brief Constructs the hessian of a programs lagrangian, of the form \f$ H +
 * \sum \lambda_i^T g(x)
 *
 * @tparam ValueType
 * @param hessian
 * @param sz Size of rows and columns of the hessian
 * @param cost_bindings
 * @param constraint_bindings
 */
template <class ValueType>
void get_lagrangian_hessian(
    Eigen::SparseMatrix<ValueType> &hessian, const bopt_index &sz,
    const std::vector<binding<cost_tpl<ValueType>>> &cost_bindings,
    const std::vector<binding<constraint_tpl<ValueType>>>
        &constraint_bindings) {
    // Populate cost entries
    std::vector<Eigen::Triplet<ValueType>> triplets;

    for (auto &b : cost_bindings) {
        // todo - check if there is a sparse implementation, if not, assume it
        // todo - is dense

        // For each non-zero element of the hessian, get their variable
        // coordinates and convert to their vector locations based on x
        typename cost_tpl<ValueType>::sparse_matrix_t &hes =
            b.get()->buffer_hessian().sparse;
        b.get()->sparsity_hessian(hes);

        // Iterate over non-zeros, only populate lower triangular entries
        for (int k = 0; k < hes.outerSize(); ++k) {
            for (Eigen::SparseMatrix<double>::InnerIterator it(hes, k); it;
                 ++it) {
                Eigen::Index row = b.indices().indices()[it.row()],
                             col = b.indices().indices()[it.col()];
                if (col > row) continue;
                // Add entry to hessian
                triplets.push_back(Eigen::Triplet<double>(row, col, 1.0));
            }
        }
    }

    bopt_index cnt = bopt_index(0);
    for (auto &b : constraint_bindings) {
        // todo - check if there is a sparse implementation, if not, assume it
        // todo - is dense

        // For each non-zero element of the hessian, get their variable
        // coordinates and convert to their vector locations based on x
        typename constraint_tpl<ValueType>::sparse_matrix_t &hes =
            b.get()->buffer_hessian().sparse;
        b.get()->sparsity_hessian(hes);

        // Iterate over non-zeros, only populate lower triangular entries
        for (int k = 0; k < hes.outerSize(); ++k) {
            for (Eigen::SparseMatrix<double>::InnerIterator it(hes, k); it;
                 ++it) {
                Eigen::Index row = b.indices().indices()[it.row()],
                             col = b.indices().indices()[it.col()];
                if (col > row) continue;
                // Add entry to hessian
                triplets.push_back(Eigen::Triplet<double>(row, col, 1.0));
            }
        }
    }
    // Create constraint hessian
    hessian.resize(sz, sz);
    hessian.setFromTriplets(triplets.begin(), triplets.end());
}

template <class ValueType>
void eval_lagrangian_hessian(
    const Eigen::Ref<const Eigen::VectorXd> &x,
    const Eigen::Ref<const Eigen::VectorXd> &lambda,
    Eigen::SparseMatrix<ValueType> &hessian,
    const std::vector<binding<cost_tpl<ValueType>>> &cost_bindings,
    const std::vector<binding<constraint_tpl<ValueType>>>
        &constraint_bindings) {
    // for (auto &b : cost_bindings) {
    //     typename constraint_tpl<ValueType>::sparse_matrix_t &hes =
    //         b.get()->buffer_jacobian().sparse;

    //     Eigen::Ref<const Eigen::VectorXd> xi = x(b.indices().indices());

    //     if (b.get()->eval_hessian(xi, hes) ==
    //         evaluator::return_status::NotImplemented) {
    //     }

    //     // Iterate over non-zeros
    //     for (int k = 0; k < hes.outerSize(); ++k) {
    //         for (Eigen::SparseMatrix<double>::InnerIterator it(hes, k); it;
    //              ++it) {
    //             // todo - speed this up
    //             hessian.coeffRef(cnt + it.row(),
    //                               b.indices().indices()[it.col()]) = it.value();
    //         }
    //     }

    //     cnt += b.get()->sz_out();
    // }
}

/**
 * @brief Represents a generic mathematical program with constraints and
 costs.
 *
 * This class represents an optimisation problem of the form:
 * \f$ \min f(x) \text{ s.t. } g_l \le g(x) \le q_u, x_l \le x \le x_u \f$
 *
 * @tparam ValueType Type of values in the program (e.g., double).
 */
template <typename ValueType>
class mathematical_program {
   public:

    typedef ValueType value_type;
    typedef std::string string_t;

    typedef constraint_tpl<ValueType> constraint_t;
    typedef linear_constraint_tpl<ValueType> linear_constraint_t;
    typedef bounding_box_constraint_tpl<ValueType> bounding_box_constraint_t;

    typedef cost_tpl<ValueType> cost_t;
    typedef linear_cost_tpl<ValueType> linear_cost_t;
    typedef quadratic_cost_tpl<ValueType> quadratic_cost_t;

    using dense_vector_t = Eigen::VectorX<ValueType>;
    // using dense_vector_t = Eigen::VectorX<ValueType>;

    /**
     * @brief Default constructor for the mathematical program.
     */
    mathematical_program() : name_("mathematical_program") {}

    /**
     * @brief Constructs a mathematical program with a specified name.
     *
     * @param name Name of the mathematical program.
     */
    mathematical_program(const string_t &name) : name_(name) {}

    /**
     * @brief Gets the name of the mathematical program.
     *
     * @return const string_t& Reference to the program's name.
     */
    const string_t &name() const { return name_; }

    /**
     * @brief Gets the number of decision variables in the program.
     *
     * @return bopt_index Number of decision variables.
     */
    bopt_index n_variables() const { return variables_.size(); }

    /**
     * @brief Gets the number of cost functions in the program.
     *
     * @return bopt_index Number of cost functions.
     */
    bopt_index n_costs() const { return get_all_costs().size(); }

    /**
     * @brief Gets the number of constraints in the program.
     *
     * @return bopt_index Number of constraints.
     */
    bopt_index n_constraints() const {
        bopt_index n = 0;
        for (const auto &c : get_all_constraints()) {
            n += c.get()->sz_out();
        }
        return n;
    }

    /**
     * @brief Gets the initial values of the decision variables.
     *
     * @return const dense_vector_t& Reference to the vector of
     initial
     * values.
     */
    const dense_vector_t &variables_initial_value() const { return x_iv_; }

    /**
     * @brief Gets the bounds for the decision variables.
     *
     * @return const vector_bounds<value_type>& Reference to the variable
     * bounds.
     */
    const dense_vector_t &variables_lower_bound() const { return x_lb_; }

    const dense_vector_t &variables_upper_bound() const { return x_ub_; }

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
        // Ensure variable isn't already added
        if (variable_index_map_.find(v.id()) != variable_index_map_.end())
            return;

        variable_index_map_.insert({v.id(), variables_.size()});
        variables_.push_back(v);

        // Update decision variable vector sizes
        x_iv_.conservativeResize(x_iv_.size() + 1);
        x_lb_.conservativeResize(x_lb_.size() + 1);
        x_ub_.conservativeResize(x_ub_.size() + 1);
        x_iv_.tail(1) << v0;
        x_lb_.tail(1) << bl;
        x_ub_.tail(1) << bu;
    }

    void add_variables(
        const Eigen::Ref<const variable_vector> &variables,
        const Eigen::Ref<const dense_vector_t> &v0 = dense_vector_t(),
        const Eigen::Ref<const dense_vector_t> &bl = dense_vector_t(),
        const Eigen::Ref<const dense_vector_t> &bu = dense_vector_t()) {
        constexpr value_type inf = std::numeric_limits<value_type>::infinity();
        for (auto it = variables.begin(); it != variables.end(); ++it) {
            bopt_index i = std::distance(variables.begin(), it);
            value_type v0_i = value_type(0), bl_i = -inf, bu_i = inf;
            if (v0.size() != 0) v0_i = v0[i];
            if (bl.size() != 0) bl_i = bl[i];
            if (bu.size() != 0) bu_i = bu[i];

            add_variable(*it, v0_i, bl_i, bu_i);
        }
    }

    Eigen::Index variable_index(const variable &v) const {
        const auto &it = variable_index_map_.find(v.id());
        if (it != variable_index_map_.end()) {
            return it->second;
        }
        std::ostringstream ss;
        ss << "Variable \'" << v << "\' does not exist in program: \'"
           << this->name() << '\'';
        throw std::runtime_error(ss.str());
    }

    std::vector<Eigen::Index> variable_indices(
        const Eigen::Ref<const variable_vector> &v) const {
        std::vector<Eigen::Index> indices = {};
        for (const auto &vi : v) {
            indices.emplace_back(variable_index(vi));
        }
        return indices;
    }

    // costs
    void add_cost(const typename std::shared_ptr<cost_t> &cost,
                  const Eigen::Ref<const variable_vector> &x) {
        // Create binding
        costs_generic_.push_back(binding<cost_t>(cost, variable_indices(x)));
    }

    void add_linear_cost(const typename std::shared_ptr<linear_cost_t> &cost,
                         const Eigen::Ref<const variable_vector> &x) {
        // Create binding
        costs_linear_.push_back(
            binding<linear_cost_t>(cost, variable_indices(x)));
    }

    void add_quadratic_cost(
        const typename std::shared_ptr<quadratic_cost_t> &cost,
        const Eigen::Ref<const variable_vector> &x) {
        // Create binding
        costs_quadratic_.push_back(
            binding<quadratic_cost_t>(cost, variable_indices(x)));
    }

    std::vector<binding<cost_t>> &generic_costs() { return costs_generic_; }

    std::vector<binding<linear_cost_t>> &linear_costs() {
        return costs_linear_;
    }

    std::vector<binding<quadratic_cost_t>> &quadratic_costs() {
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
    void add_constraint(const std::shared_ptr<constraint_t> &constraint,
                        const Eigen::Ref<const variable_vector> &x) {
        // Create binding
        constraints_generic_.push_back(
            binding<constraint_t>(constraint, variable_indices(x)));
    }

    void add_linear_constraint(
        const std::shared_ptr<linear_constraint_t> &constraint,
        const Eigen::Ref<const variable_vector> &x) {
        // Create binding
        constraints_linear_.push_back(
            binding<linear_constraint_t>(constraint, variable_indices(x)));
    }

    void add_bounding_box_constraint(
        const std::shared_ptr<bounding_box_constraint_t> &constraint,
        const Eigen::Ref<const variable_vector> &x) {
        // Create binding
        constraints_bounding_box_.push_back(binding<bounding_box_constraint_t>(
            constraint, variable_indices(x)));
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
    // Name
    string_t name_;

    dense_vector_t x_;
    // Decision variables initial value
    dense_vector_t x_iv_;
    // Decision variable bounds
    dense_vector_t x_lb_;
    dense_vector_t x_ub_;

    std::vector<variable> variables_;
    std::unordered_map<variable::id_type, Eigen::Index> variable_index_map_;

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
