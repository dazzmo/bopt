#pragma once

#include <variant>

#include "bopt/Binding.hpp"
#include "bopt/Common.hpp"
#include "bopt/Constraints.hpp"
#include "bopt/Costs.hpp"
#include "bopt/Logging.hpp"

namespace bopt {

/**
 * @brief Represents a generic mathematical program with constraints and
 costs. Can either be of a dense or sparse nature.
 *
 * This class represents an optimisation problem of the form:
 * \f$ \min f(x) \text{ s.t. } g_l \le g(x) \le q_u, x_l \le x \le x_u \f$
 *
 * @tparam ValueType Type of values in the program (e.g., double).
 */
class MathematicalProgram {
   private:
    using CostVariant =
        std::variant<Binding<DenseCost>, Binding<SparseCost>,
                     Binding<DenseLinearCost>, Binding<SparseLinearCost>>;

    using ConstraintVariant =
        std::variant<Binding<DenseConstraint>, Binding<SparseConstraint>,
                     Binding<DenseLinearConstraint>,
                     Binding<SparseLinearConstraint>>;

    using VectorXd = typename MathTypes<Real>::VectorX;

   public:
    /**
     * @brief Default constructor for the mathematical program.
     */
    MathematicalProgram() : name_(""), numConstraints_(0) {}

    /**
     * @brief Constructs a mathematical program with a specified name.
     *
     * @param name Name of the mathematical program.
     */
    MathematicalProgram(const String &name) : name_(name), numConstraints_(0) {}

    /**
     * @brief Gets the name of the mathematical program.
     *
     * @return const String& Reference to the program's name.
     */
    const String &name() const { return name_; }

    /**
     * @brief Gets the number of decision variables in the program.
     *
     * @return Size Number of decision variables.
     */
    Size numVariables() const { return variables_.size(); }

    /**
     * @brief Gets the number of cost functions in the program.
     *
     * @return Size Number of cost functions.
     */
    Size numCosts() const { return cost_bindings_.size(); }

    /**
     * @brief Gets the number of constraints in the program.
     *
     * @return Size Number of constraints.
     */
    Size numConstraints() const {
        // Iterate through each constraint and count the number
        Index m = 0;
        for (const auto &constraint : constraint_bindings_) {
            std::visit(
                [&](auto &&binding) { m += binding.get()->outputSize(); },
                constraint);
        }
        return m;
    }

    /**
     * @brief Gets the initial values of the decision variables.
     *
     * @return const VectorXd& Reference to the vector of
     initial
     * values.
     */
    const VectorXd &variableInitialValues() const { return x_iv_; }

    /**
     * @brief Gets the bounds for the decision variables.
     *
     * @return const vector_bounds<value_type>& Reference to the variable
     * bounds.
     */
    const VectorXd &variableLowerBounds() const { return x_lb_; }

    const VectorXd &variableUpperBounds() const { return x_ub_; }

    Variable addVariable(const String &name, const double &initial_value = 0.0,
                         const double &lower_bound = -kInf,
                         const double &upper_bound = kInf) {
        // Create variable
        Variable v(name);
        // Add variable to map
        variable_index_map_.insert({v.getId(), variables_.size()});
        variables_.push_back(v);

        // Update decision variable vector sizes
        x_iv_.conservativeResize(x_iv_.size() + 1);
        x_lb_.conservativeResize(x_lb_.size() + 1);
        x_ub_.conservativeResize(x_ub_.size() + 1);
        // Insert variable data
        x_iv_.tail(1) << initial_value;
        x_lb_.tail(1) << lower_bound;
        x_ub_.tail(1) << upper_bound;

        return v;
    }

    VariableVector addVariables(const String &name, const Index &n) {
        VariableVector x(n);
        for (Index i = 0; i < n; ++i) {
            x[i] = addVariable(name + std::to_string(i));
        }
        return x;
    }

    const std::vector<Variable> &getAllVariables() const { return variables_; }

    Index getVariableIndex(const Variable &v) const {
        const auto &it = variable_index_map_.find(v.getId());
        if (it != variable_index_map_.end()) {
            return it->second;
        }
        LOG(FATAL) << "Variable \'" << v << "\' does not exist in program: \'"
                   << this->name() << '\'';
        return -1;
    }

    std::vector<Index> getVariableIndices(
        const Eigen::Ref<const VariableVector> &v) const {
        std::vector<Index> indices = {};
        for (const auto &vi : v) {
            indices.emplace_back(getVariableIndex(vi));
        }
        return indices;
    }

    /**
     * @brief Add a cost to the program, bound to the provided variables.
     *
     * @param cost
     * @param x
     */
    template <typename CostType>
    void addCost(const std::shared_ptr<CostType> &cost,
                 const std::shared_ptr<typename CostType::Data> &data,
                 const Eigen::Ref<const VariableVector> &x) {
        // Create binding
        cost_bindings_.emplace_back(
            Binding<CostType>(cost, data, getVariableIndices(x)));
    }

    /**
     * @brief Add a dense linear cost to the program
     *
     * @param cost
     * @param data
     * @param x
     */
    template <typename EvaluatorTraits>
    void addLinearCost(
        const std::shared_ptr<LinearCostTpl<EvaluatorTraits>> &cost,
        const std::shared_ptr<typename LinearCostTpl<EvaluatorTraits>::Data>
            &data,
        const Eigen::Ref<const VariableVector> &x) {
        // Create binding
        this->addCost<LinearCostTpl<EvaluatorTraits>>(cost, data, x);
    }

    // /**
    //  * @brief Add a dense linear cost to the program
    //  *
    //  * @param cost
    //  * @param data
    //  * @param x
    //  */
    // void addQuadraticCost(
    //     const std::shared_ptr<DenseQuadraticCost> &cost,
    //     const std::shared_ptr<typename DenseQuadraticCost::Data> &data,
    //     const Eigen::Ref<const VariableVector> &x) {
    //     // Create binding
    //     this->addCost<DenseQuadraticCost>(cost, data, x);
    // }

    // /**
    //  * @brief Add a sparse Quadratic cost to the program
    //  *
    //  * @param cost
    //  * @param data
    //  * @param x
    //  */
    // void addQuadraticCost(
    //     const std::shared_ptr<SparseQuadraticCost> &cost,
    //     const std::shared_ptr<typename SparseQuadraticCost::Data> &data,
    //     const Eigen::Ref<const VariableVector> &x) {
    //     this->addCost<SparseQuadraticCost>(cost, data, x);
    // }

    /**
     * @brief Add a constraint to the program, bound to the provided variables.
     *
     * @param constraint
     * @param x
     */
    template <typename ConstraintType>
    void addConstraint(
        const std::shared_ptr<ConstraintType> &constraint,
        const std::shared_ptr<typename ConstraintType::Data> &data,
        const Eigen::Ref<const VariableVector> &x) {
        // Create binding
        constraint_bindings_.emplace_back(
            Binding<ConstraintType>(constraint, data, getVariableIndices(x)));
    }

    /**
     * @brief Add a dense linear cost to the program
     *
     * @param cost
     * @param data
     * @param x
     */
    void addLinearConstraint(
        const std::shared_ptr<DenseLinearConstraint> &constraint,
        const std::shared_ptr<typename DenseLinearConstraint::Data> &data,
        const Eigen::Ref<const VariableVector> &x) {
        // Create binding
        this->addConstraint<DenseLinearConstraint>(constraint, data, x);
    }

    /**
     * @brief Add a sparse linear constraint to the program
     *
     * @param constraint
     * @param data
     * @param x
     */
    void addLinearConstraint(
        const std::shared_ptr<SparseLinearConstraint> &constraint,
        const std::shared_ptr<typename SparseLinearConstraint::Data> &data,
        const Eigen::Ref<const VariableVector> &x) {
        this->addConstraint<SparseLinearConstraint>(constraint, data, x);
    }

    // void addBoundingBoxConstraint(
    //     const std::shared_ptr<DenseBoundingBoxConstraint> &constraint,
    //     const std::shared_ptr<typename DenseBoundingBoxConstraint::Data> &data,
    //     const Eigen::Ref<const VariableVector> &x) {
    //     // Create binding
    //     bb_constraint_bindings_.push_back(Binding<DenseBoundingBoxConstraint>(
    //         constraint, data, getVariableIndices(x)));
    // }

    // void addBoundingBoxConstraint(const Eigen::Ref<const VariableVector> &x,
    //                               const Eigen::Ref<const Eigen::VectorXd> &lb,
    //                               const Eigen::Ref<const Eigen::VectorXd> &ub) {
    //     // Create binding
    //     auto constraint =
    //         std::make_shared<DenseBoundingBoxConstraint>(x.rows(), lb, ub);
    //     auto data = constraint->createData();
    //     bb_constraint_bindings_.push_back(Binding<DenseBoundingBoxConstraint>(
    //         constraint, data, getVariableIndices(x)));
    // }

    /**
     * @brief Get all cost bindings of a specific type, note that this will
     * return all bindings which are of this type, as well as any bindings that
     * have a base of this given type.
     *
     * @tparam BindingType
     * @return std::vector<Binding<BindingType>>
     */
    template <typename BindingType>
    std::vector<Binding<BindingType>> getCosts() {
        std::vector<Binding<BindingType>> vec;

        for (const auto &cost : cost_bindings_) {
            std::visit(
                [&](auto &&binding) {
                    using T = std::decay_t<decltype(binding)>;
                    if constexpr (std::is_base_of_v<BindingType,
                                                    typename T::Evaluator>) {
                        vec.push_back(binding);
                    }
                },
                cost);
        }

        // Return vector of all costs
        return vec;
    }

    /**
     * @brief Get all constraint bindings of a specific type, note that this
     * will return all bindings which are of this type, as well as any bindings
     * that have a base of this given type.
     *
     * @tparam BindingType
     * @return std::vector<Binding<BindingType>>
     */
    template <typename BindingType>
    std::vector<Binding<BindingType>> getConstraints() const {
        std::vector<Binding<BindingType>> vec;

        for (const auto &constraint : constraint_bindings_) {
            std::visit(
                [&](auto &&binding) {
                    using T = std::decay_t<decltype(binding)>;
                    if constexpr (std::is_base_of_v<BindingType,
                                                    typename T::Evaluator>) {
                        vec.push_back(binding);
                    }
                },
                constraint);
        }

        // Return vector of all costs
        return vec;
    }

    // std::vector<Binding<DenseBoundingBoxConstraint>> getBoundingBoxConstraints()
    //     const {
    //     return bb_constraint_bindings_;
    // }

   protected:
   private:
    // Name
    String name_;

    Index numConstraints_;

    VectorXd x_;
    // Decision variables initial value
    VectorXd x_iv_;
    // Decision variable bounds
    VectorXd x_lb_;
    VectorXd x_ub_;

    std::vector<Variable> variables_;
    std::unordered_map<Variable::Id, Index> variable_index_map_;

    std::vector<CostVariant> cost_bindings_ = {};
    std::vector<ConstraintVariant> constraint_bindings_ = {};
    // std::vector<Binding<DenseBoundingBoxConstraint>> bb_constraint_bindings_ =
    //     {};
};

std::ostream &operator<<(std::ostream &os, const MathematicalProgram &program);

}  // namespace bopt
