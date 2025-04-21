#pragma once

#include <variant>

#include "bopt/binding.hpp"
#include "bopt/common.hpp"
// #include "bopt/constraints.hpp"
#include "bopt/costs.hpp"
#include "bopt/logging.hpp"
// #include "bopt/profiler.hpp"

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
    using CostVariant = std::variant<Binding<DenseCostTpl<double>>,
                                     Binding<SparseCostTpl<double>>,
                                     Binding<DenseLinearCostTpl<double>>,
                                     Binding<SparseLinearCostTpl<double>>>;

   public:
    /**
     * @brief Default constructor for the mathematical program.
     */
    MathematicalProgram() : name_(""), n_constraints_(0) {}

    /**
     * @brief Constructs a mathematical program with a specified name.
     *
     * @param name Name of the mathematical program.
     */
    MathematicalProgram(const std::string &name)
        : name_(name), n_constraints_(0) {}

    /**
     * @brief Gets the name of the mathematical program.
     *
     * @return const std::string& Reference to the program's name.
     */
    const std::string &name() const { return name_; }

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
    // todo bopt_index n_costs() const { return getAllCosts().size(); }

    /**
     * @brief Gets the number of constraints in the program.
     *
     * @return bopt_index Number of constraints.
     */
    bopt_index n_constraints() const { return n_constraints_; }

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

    Variable addVariable(const std::string &name,
                         const double &initial_value = 0.0,
                         const double &lower_bound = -kInf,
                         const double &upper_bound = kInf) {
        // Create variable
        Variable v(name);
        // Add variable to map
        variable_index_map_.insert({v.id(), variables_.size()});
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

    const std::vector<Variable> &getAllVariables() const { return variables_; }

    Eigen::Index getVariableIndex(const Variable &v) const {
        const auto &it = variable_index_map_.find(v.id());
        if (it != variable_index_map_.end()) {
            return it->second;
        }
        LOG(FATAL) << "Variable \'" << v << "\' does not exist in program: \'"
                   << this->name() << '\'';
        return -1;
    }

    std::vector<Eigen::Index> getVariableIndices(
        const Eigen::Ref<const VariableVector> &v) const {
        std::vector<Eigen::Index> indices = {};
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

    std::vector<Binding<DenseCostTpl<double>>> getDenseCosts() {
        std::vector<Binding<DenseCostTpl<double>>> vec;

        for (const auto &cost : cost_bindings_) {
            std::visit(
                [&](auto &&binding) {
                    using T = std::decay_t<decltype(binding)>;
                    if constexpr (std::is_same_v<typename T::Evaluator,
                                                 DenseCostTpl<double>>) {
                        vec.push_back(binding);
                    }
                },
                cost);
        }

        // Return vector of all costs
        return vec;
    }

    // std::vector<Binding<Cost>> &genericCosts() { return costs_generic_; }

    // std::vector<Binding<LinearCost>> &linearCosts() { return costs_linear_; }

    // std::vector<Binding<QuadraticCost>> &quadraticCosts() {
    //     return costs_quadratic_;
    // }

    // std::vector<Binding<Cost>> getAllCosts() const {
    //     std::vector<Binding<Cost>> vec;
    //     vec.insert(vec.begin(), costs_generic_.begin(),
    //     costs_generic_.end()); vec.insert(vec.end(), costs_linear_.begin(),
    //     costs_linear_.end()); vec.insert(vec.end(), costs_quadratic_.begin(),
    //     costs_quadratic_.end());
    //     // Return vector of all costs
    //     return vec;
    // }

    // // constraints
    // void addConstraint(const std::shared_ptr<Constraint> &constraint,
    //                    const Eigen::Ref<const VariableVector> &x) {
    //     n_constraints_ += constraint->getOuptutDimension();
    //     // Create binding
    //     constraints_generic_.push_back(
    //         Binding<Constraint>(constraint, getVariableIndices(x)));
    // }

    // void addLinearConstraint(
    //     const std::shared_ptr<LinearConstraint> &constraint,
    //     const Eigen::Ref<const VariableVector> &x) {
    //     n_constraints_ += constraint->getOuptutDimension();
    //     // Create binding
    //     constraints_linear_.push_back(
    //         Binding<LinearConstraint>(constraint, getVariableIndices(x)));
    // }

    // void addBoundingBoxConstraint(
    //     const std::shared_ptr<BoundingBoxConstraint> &constraint,
    //     const Eigen::Ref<const VariableVector> &x) {
    //     // Create binding
    //     constraints_bounding_box_.push_back(
    //         Binding<BoundingBoxConstraint>(constraint,
    //         getVariableIndices(x)));
    // }

    // std::vector<Binding<Constraint>> &getConstraints() {
    //     return constraints_generic_;
    // }

    // std::vector<Binding<LinearConstraint>> &linearConstraints() {
    //     return constraints_linear_;
    // }

    // std::vector<Binding<BoundingBoxConstraint>> &boundingBoxConstraints() {
    //     return constraints_bounding_box_;
    // }

    // /**
    //  * @brief Returns a vector of all constraint bindings.
    //  *
    //  * @note This does not include BoundingBox constraints, or any
    //  matrix-based
    //  * constraints
    //  * @return std::vector<Binding<Constraint>>
    //  */
    // std::vector<Binding<Constraint>> getAllConstraints() const {
    //     std::vector<Binding<Constraint>> vec;
    //     vec.insert(vec.begin(), constraints_generic_.begin(),
    //                constraints_generic_.end());
    //     vec.insert(vec.end(), constraints_linear_.begin(),
    //                constraints_linear_.end());
    //     // Return vector of all costs
    //     return vec;
    // }

   protected:
   private:
    // Name
    std::string name_;

    Index n_constraints_;

    VectorXd x_;
    // Decision variables initial value
    VectorXd x_iv_;
    // Decision variable bounds
    VectorXd x_lb_;
    VectorXd x_ub_;

    std::vector<Variable> variables_;
    std::unordered_map<Variable::Id, Eigen::Index> variable_index_map_;

    // constraint bindings
    // std::vector<Binding<Constraint>> constraints_generic_ = {};
    // std::vector<Binding<LinearConstraint>> constraints_linear_ = {};
    // std::vector<Binding<BoundingBoxConstraint>> constraints_bounding_box_ =
    // {};

    // cost bindings
    std::vector<CostVariant> cost_bindings_ = {};
    // std::vector<Binding<Cost>> costs_generic_ = {};
    // std::vector<Binding<LinearCost>> costs_linear_ = {};
};

//    template <typename T>
//    void addConstraint(const Binding<T>& binding) {
//        constraint_bindings_.emplace_back(binding);
//    }

//    template <typename T>
//    void addConstraint(const T& constraint) {
//        addConstraint(Binding<T>(constraint));
//    }

std::ostream &operator<<(std::ostream &os, const MathematicalProgram &program);

}  // namespace bopt
