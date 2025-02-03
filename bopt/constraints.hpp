#pragma once

#include "bopt/evaluator.hpp"
#include "bopt/logging.hpp"

namespace bopt {

/**
 * @brief Constraint of the form y = fₚ(x) ∈ ℝᵐ
 *
 */
class Constraint : public EvaluatorBase {
   public:
    enum class Type {
        // Constraint of the form lower_bound() = c(x) = upper_bound()
        Equality,
        // Constraint of the form lower_bound() ≤ c(x) ≤ upper_bound()
        Inequality
    };

    // Constraint() : EvaluatorBase(), type_(Type::Equality), name_("") {}

    const Type &type() const { return type_; }

    /**
     * @brief Set the constraint to a particular type
     *
     * @return const Type&
     */
    void setType(const Type &type) { type_ = type; }

    /**
     * @brief Name of the constraint
     *
     * @return const std::string&
     */
    const std::string &name() const { return name_; }

    /**
     * @brief Sets the name of the constraint.
     *
     * @param name
     */
    void setName(const std::string &name) { name_ = name; }

    /**
     * @brief The lower bound vector of the constraint of size (\ref
     * dim_output() x 1).
     *
     * @return const VectorXd&
     */
    const VectorXd &lowerBound() const { return lower_bound_; }
    void setLowerBound(const Eigen::Ref<const VectorXd> &bound) {
        BOPT_ASSERT(bound.size() == dim_output());
        lower_bound_ = bound;
    }

    /**
     * @brief The upper bound vector of the constraint of size (\ref
     * dim_output() x 1).
     *
     * @return const VectorXd&
     */
    const VectorXd &upperBound() const { return upper_bound_; }
    void setUpperBound(const Eigen::Ref<const VectorXd> &bound) {
        BOPT_ASSERT(bound.size() == dim_output());
        upper_bound_ = bound;
    }

    /**
     * @brief Whether the constraints of the system are satisfied to a given
     * tolerance.
     *
     * @param value The current value of the constraint
     * @param epsilon Tolerance
     * @return true
     * @return false
     */
    bool isSatisfied(const Eigen::Ref<const VectorXd> &value,
                     const double &epsilon = kEpsilon) const {
        BOPT_ASSERT(value.size() == dim_output());
        for (int i = 0; i < dim_output(); ++i) {
            if (lowerBound()[i] - value[i] > epsilon ||
                upperBound()[i] - value[i] < -epsilon)
                return false;
        }
        return true;
    }

    // Derivatives with respect to parameters

    const std::optional<SparsityPattern> &
    lower_bound_jacobian_sparsity_pattern() const {
        return lb_jacobian_sparsity_pattern_;
    }
    const std::optional<SparsityPattern> &
    upper_bound_jacobian_sparsity_pattern() const {
        return ub_jacobian_sparsity_pattern_;
    }

    const std::optional<SparsityPattern> &lower_bound_hessian_sparsity_pattern()
        const {
        return lb_hessian_sparsity_pattern_;
    }
    const std::optional<SparsityPattern> &upper_bound_hessian_sparsity_pattern()
        const {
        return ub_hessian_sparsity_pattern_;
    }

    void setBoundJacobianSparsityPattern(
        const std::optional<SparsityPattern> &lower_bound_pattern,
        const std::optional<SparsityPattern> &upper_bound_pattern) {
        lb_jacobian_sparsity_pattern_ = lower_bound_pattern;
        ub_jacobian_sparsity_pattern_ = upper_bound_pattern;
    }

    void setLowerBoundHessianSparsityPattern(
        const std::optional<SparsityPattern> &lower_bound_pattern,
        const std::optional<SparsityPattern> &upper_bound_pattern) {
        lb_hessian_sparsity_pattern_ = lower_bound_pattern;
        ub_hessian_sparsity_pattern_ = upper_bound_pattern;
    }

   protected:
    Constraint(const Index &dim_input, const Index &dim_output)
        : EvaluatorBase(dim_input, dim_output),
          name_(""),
          type_(Type::Equality),
          lower_bound_(VectorXd::Zero(dim_output)),
          upper_bound_(VectorXd::Zero(dim_output)) {}

    void setBoundsJacobianNonZeroOnly(bool lower_bound_p, bool upper_bound_p) {}
    void setBoundsHessianNonZeroOnly(bool lower_bound_p, bool upper_bound_p);

   private:
    std::string name_;
    Type type_;

    VectorXd lower_bound_;
    VectorXd upper_bound_;

    // Behaviour of constraint with respect to parameters
    bool ub_jacobian_p_nz_only_;
    bool lb_jacobian_p_nz_only_;

    bool ub_hessian_pp_nz_only_;
    bool lb_hessian_pp_nz_only_;

    std::optional<SparsityPattern> lb_jacobian_sparsity_pattern_;
    std::optional<SparsityPattern> ub_jacobian_sparsity_pattern_;

    std::optional<SparsityPattern> lb_hessian_sparsity_pattern_;
    std::optional<SparsityPattern> ub_hessian_sparsity_pattern_;
};

/**
 * @brief Constraint of the form lb ≤ Ax ≤ ub
 *
 */
class LinearConstraint : public Constraint {
   public:
    void evalA(Eigen::Ref<MatrixXd> A) { evalAImpl(A); }

    const std::optional<SparsityPattern> &A_sparsity_pattern() const {
        return A_sparsity_pattern_;
    }
    void setASparsityPattern(const SparsityPattern &pattern) {
        A_sparsity_pattern_ = pattern;
        setJacobianSparsityPattern(pattern);
    }

    bool A_has_nz_only() const { return A_has_nz_only_; }

   protected:
    LinearConstraint(const Index &dim_input, const Index &dim_output)
        : Constraint(dim_input, dim_output),
          A_has_nz_only_(false),
          A_sparsity_pattern_(std::nullopt) {
        setName("linear_constraint");
    }

    virtual void evalAImpl(Eigen::Ref<MatrixXd> A) {}

    /**
     * @brief Indicate whether the evaluation of the jacobians will return only
     * the non-zero elements. If false, evaluation expects the full jacobian to
     * be computed.
     *
     * @param flag
     */
    void setANonZeroOnly(bool flag) {
        A_has_nz_only_ = flag;
        setJacobianNonZeroOnly(flag);
    }

    void evalJacobianImpl(const Eigen::Ref<const VectorXd> &x,
                          Eigen::Ref<MatrixXd> jacobian) override {
        evalA(jacobian);
    }

   private:
    bool A_has_nz_only_;
    std::optional<SparsityPattern> A_sparsity_pattern_;
};

/**
 * @brief Constraint of the form lower_bound() <= x <= upper_bound()
 *
 */
class BoundingBoxConstraint : public LinearConstraint {
   public:
    BoundingBoxConstraint(const Index &dim_input,
                          const Eigen::Ref<const VectorXd> &lower_bound,
                          const Eigen::Ref<const VectorXd> &upper_bound)
        : LinearConstraint(dim_input, dim_input) {
        SparsityPattern pattern = {};
        for (Index i = 0; i < this->dim_input(); ++i) pattern.push_back({i, i});
        this->setASparsityPattern(pattern);
    }

    BoundingBoxConstraint(const Index &dim_input, const double &lower_bound,
                          const double &upper_bound)
        : LinearConstraint(dim_input, dim_input) {
        SparsityPattern pattern = {};
        for (Index i = 0; i < this->dim_input(); ++i) pattern.push_back({i, i});
        this->setASparsityPattern(pattern);
    }

    static std::shared_ptr<BoundingBoxConstraint> create(
        const Index &dim_input, const Eigen::Ref<const VectorXd> &lower_bound,
        const Eigen::Ref<const VectorXd> &upper_bound) {
        return std::make_shared<BoundingBoxConstraint>(dim_input, lower_bound,
                                                       upper_bound);
    }

   protected:
    void evalImpl(const Eigen::Ref<const VectorXd> &x,
                  Eigen::Ref<VectorXd> y) override {
        y = x;
    }

   private:
};

}  // namespace bopt
