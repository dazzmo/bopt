#pragma once

#include "bopt/evaluator.hpp"
#include "bopt/logging.hpp"

namespace bopt {

enum class ConstraintType {
    /// Constraint of the form lower_bound() = c(x) = upper_bound()
    Equality,
    /// Constraint of the form lower_bound() ≤ c(x) ≤ upper_bound()
    Inequality
};

template <typename Scalar>
struct ConstraintDataTpl;

/**
 * @brief Constraint of the form y = fₚ(x) ∈ ℝᵐ
 *
 */
template <typename Scalar>
class ConstraintTpl : public EvaluatorTpl<Scalar> {
    using ConstraintData = ConstraintDataTpl<Scalar>;

   public:
    const ConstraintType &type() const { return type_; }

    /**
     * @brief Set the constraint to a particular type
     *
     * @return const Type&
     */
    void setType(const ConstraintType &type) { type_ = type; }

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

    // Derivatives with respect to parameters
    void evalBounds(ConstraintData &data) const { evalBoundsImpl(data); }

    void evalBoundJacobians(ConstraintData &data) const {
        evalBoundJacobiansImpl(data);
    }

    void evalBoundSparseJacobians(ConstraintData &data) const {
        evalBoundSparseJacobiansImpl(data);
    }

    void evalBoundHessians(const Eigen::Ref<const VectorX<Scalar>> &lambda,
                           ConstraintData &data) const {
        evalBoundHessiansImpl(lambda, data);
    }

    void evalBoundSparseHessians(
        const Eigen::Ref<const VectorX<Scalar>> &lambda,
        ConstraintData &data) const {
        evalBoundSparseHessiansImpl(lambda, data);
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
    bool isSatisfied(ConstraintData &data,
                     const double &epsilon = kEpsilon) const {
        for (int i = 0; i < this->dim_output(); ++i) {
            if (data.lb[i] - data.y[i] > epsilon ||
                data.ub[i] - data.y[i] < -epsilon)
                return false;
        }
        return true;
    }

   protected:
    ConstraintTpl(const Index &dim_input, const Index &dim_output)
        : EvaluatorTpl<Scalar>(dim_input, dim_output),
          name_(""),
          type_(ConstraintType::Equality),
          ptr_(nullptr) {}

    /**
     * @brief Construct a constraint from an existing evaluator
     *
     * @param evaluator
     */
    ConstraintTpl(const std::shared_ptr<EvaluatorTpl<Scalar>> &evaluator)
        : EvaluatorTpl<Scalar>(evaluator),
          name_(""),
          type_(ConstraintType::Equality),
          ptr_(nullptr) {}

    /**
     * @brief Construct a constraint from an existing constraint
     *
     * @param evaluator
     */
    ConstraintTpl(const std::shared_ptr<ConstraintTpl<Scalar>> &constraint)
        : EvaluatorTpl<Scalar>(constraint),
          name_(""),
          type_(ConstraintType::Equality),
          ptr_(constraint) {}

    virtual void evalBoundsImpl(ConstraintData &data) const {
        if (ptr_) ptr_->evalBounds(data);
    }

    virtual void evalBoundJacobiansImpl(ConstraintData &data) const {
        if (ptr_) ptr_->evalBoundJacobians(data);
    }
    virtual void evalBoundSparseJacobiansImpl(ConstraintData &data) const {
        if (ptr_) ptr_->evalBoundSparseJacobians(data);
    }

    virtual void evalBoundHessiansImpl(
        const Eigen::Ref<const VectorX<Scalar>> &lambda,
        ConstraintData &data) const {
        if (ptr_) ptr_->evalBoundHessians(lambda, data);
    }
    virtual void evalBoundSparseHessiansImpl(
        const Eigen::Ref<const VectorX<Scalar>> &lambda,
        ConstraintData &data) const {
        if (ptr_) ptr_->evalBoundSparseHessians(lambda, data);
    }

   private:
    std::string name_;
    ConstraintType type_;

    std::shared_ptr<ConstraintTpl> ptr_;
};

typedef ConstraintTpl<double> Constraint;

template <typename Scalar>
struct ConstraintDataTpl : public EvaluatorDataTpl<Scalar> {
    ConstraintDataTpl(const ConstraintTpl<Scalar> &c)
        : EvaluatorDataTpl<Scalar>(c),
          lb(VectorX<Scalar>::Zero(c.dim_output())),
          ub(VectorX<Scalar>::Zero(c.dim_output())),
          Jlb_p(MatrixX<Scalar>::Zero(c.dim_output(), c.dim_parameter())),
          Jub_p(MatrixX<Scalar>::Zero(c.dim_output(), c.dim_parameter())),
          Hlb_pp(MatrixX<Scalar>::Zero(c.dim_parameter(), c.dim_parameter())),
          Hub_pp(MatrixX<Scalar>::Zero(c.dim_parameter(), c.dim_parameter())) {}

    /// Lower bound of the constraint
    VectorX<Scalar> lb;
    /// Upper bound of the constraint
    VectorX<Scalar> ub;

    /// Jacobian of the lower bound with respect to the parameters
    MatrixX<Scalar> Jlb_p;
    /// Jacobian of the upper bound with respect to the parameters
    MatrixX<Scalar> Jub_p;

    /// Hessian of the lower bound vector product with respect to the parameters
    MatrixX<Scalar> Hlb_pp;
    /// Hessian of the upper bound vector product with respect to the parameters
    MatrixX<Scalar> Hub_pp;

    /// Sparse Jacobian of the lower bound with respect to the parameters
    SparseMatrix<Scalar> Jlb_p_s;
    /// Sparse Jacobian of the upper bound with respect to the parameters
    SparseMatrix<Scalar> Jub_p_s;

    /// Sparse Hessian of the lower bound vector product with respect to the
    /// parameters
    SparseMatrix<Scalar> Hlb_pp_s;
    /// Sparse Hessian of the upper bound vector product with respect to the
    /// parameters
    SparseMatrix<Scalar> Hub_pp_s;
};

typedef ConstraintDataTpl<double> ConstraintData;

template <typename Scalar>
struct LinearConstraintDataTpl;

/**
 * @brief Constraint of the form lb ≤ Ax ≤ ub
 *
 */
template <typename Scalar>
class LinearConstraintTpl : public ConstraintTpl<Scalar> {
    using LinearConstraintData = LinearConstraintDataTpl<Scalar>;

   public:
    /**
     * @brief Evaluates the vector coeffcient vector bₚ for the constraint lb ≤
     * Ax ≤ ub
     *
     * @param A Coefficient matrix Aₚ
     */
    void evalCoefficients(LinearConstraintData &data) const {
        evalCoefficientsImpl(data);
    }

    void evalSparseCoefficients(LinearConstraintData &data) const {
        evalSparseCoefficientsImpl(data);
    }

    void setCoefficientSparsityPatterns(LinearConstraintData &data) const {
        setCoefficientSparsityPatternsImpl(data);
    }

   protected:
    LinearConstraintTpl(const Index &dim_input, const Index &dim_output)
        : ConstraintTpl<Scalar>(dim_input, dim_output) {
        this->setName("linear_constraint");
    }

    LinearConstraintTpl(
        const std::shared_ptr<ConstraintTpl<Scalar>> &constraint)
        : ConstraintTpl<Scalar>(constraint) {
        this->setName("linear_constraint");
    }

    virtual void evalCoefficientsImpl(LinearConstraintData &data) const {}

    virtual void evalSparseCoefficientsImpl(LinearConstraintData &data) const {}

    virtual void setCoefficientSparsityPatternsImpl(
        LinearConstraintData &data) const {}

   private:
};

typedef LinearConstraintTpl<double> LinearConstraint;

template <typename Scalar>
struct LinearConstraintDataTpl : public ConstraintDataTpl<Scalar> {
    LinearConstraintDataTpl(const LinearConstraintTpl<Scalar> &c)
        : ConstraintDataTpl<Scalar>(c),
          A(MatrixX<Scalar>::Zero(c.dim_output(), c.dim_tangent_space())),
          A_s(c.dim_output(), c.dim_tangent_space()) {
        c.setCoefficientSparsityPatterns(*this);
    }

    MatrixX<Scalar> A;
    SparseMatrix<Scalar> A_s;
};

typedef LinearConstraintDataTpl<double> LinearConstraintData;

/**
 * @brief Constraint of the form lower_bound() <= x <= upper_bound()
 *
 */
template <typename Scalar>
class BoundingBoxConstraintTpl : public LinearConstraintTpl<Scalar> {
   public:
    BoundingBoxConstraintTpl(const Index &dim_input,
                             const Eigen::Ref<const VectorXd> &lower_bound,
                             const Eigen::Ref<const VectorXd> &upper_bound)
        : LinearConstraintTpl<Scalar>(dim_input, dim_input) {}

    BoundingBoxConstraintTpl(const Index &dim_input, const double &lower_bound,
                             const double &upper_bound)
        : LinearConstraintTpl<Scalar>(dim_input, dim_input) {}

    static std::shared_ptr<BoundingBoxConstraintTpl> create(
        const Index &dim_input, const Eigen::Ref<const VectorXd> &lower_bound,
        const Eigen::Ref<const VectorXd> &upper_bound) {
        return std::make_shared<BoundingBoxConstraintTpl>(
            dim_input, lower_bound, upper_bound);
    }

   protected:
    void evalImpl(const Eigen::Ref<const VectorXd> &x,
                  EvaluatorDataTpl<Scalar> &data) const override {
        data.y = x;
    }

   private:
};

typedef BoundingBoxConstraintTpl<double> BoundingBoxConstraint;

}  // namespace bopt
