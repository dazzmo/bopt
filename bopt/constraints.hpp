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
class ConstraintTpl : public EvaluatorBaseTpl<Scalar> {
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
    void evalBounds(ConstraintData &data) { evalBoundsImpl(data); }

    void evalBoundJacobians(ConstraintData &data) {
        evalBoundJacobiansImpl(data);
    }

    void evalBoundHessians(ConstraintData &data) {
        evalBoundHessiansImpl(data);
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
        : EvaluatorBaseTpl<Scalar>(dim_input, dim_output),
          name_(""),
          type_(ConstraintType::Equality) {}

    virtual void evalBoundsImpl(ConstraintData &data) {}

    virtual void evalBoundJacobiansImpl(ConstraintData &data) {}

    virtual void evalBoundHessiansImpl(ConstraintData &data) {}

   private:
    std::string name_;
    ConstraintType type_;
};

typedef ConstraintTpl<double> Constraint;

template <typename Scalar>
struct ConstraintDataTpl : public EvaluatorBaseDataTpl<Scalar> {
    ConstraintDataTpl(const ConstraintTpl<Scalar> &c)
        : EvaluatorBaseDataTpl<Scalar>(c),
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
    void evalCoefficientMatrix(LinearConstraintData &data) {
        evalCoefficientMatrixImpl(data);
    }

    void evalSparseCoefficientMatrix(LinearConstraintData &data) {
        evalSparseCoefficientMatrixImpl(data);
    }

    void evalConstantVector(LinearConstraintData &data) {
        evalConstantVectorImpl(data);
    }

    void evalSparseConstantVector(LinearConstraintData &data) {
        evalSparseConstantVectorImpl(data);
    }

    virtual void setCoefficientMatrixSparsityPatterns(
        LinearConstraintData &data) {}
    virtual void seConstantVectorSparsityPatterns(LinearConstraintData &data) {}

   protected:
    LinearConstraintTpl(const Index &dim_input, const Index &dim_output)
        : ConstraintTpl<Scalar>(dim_input, dim_output) {
        this->setName("linear_constraint");
    }

    virtual void evalCoefficientMatrixImpl(LinearConstraintData &data) {}
    virtual void evalSparseCoefficientMatrixImpl(LinearConstraintData &data) {}

    virtual void evalConstantVectorImpl(LinearConstraintData &data) {}
    virtual void evalSparseConstantVectorImpl(LinearConstraintData &data) {}

   private:
};

typedef LinearConstraintTpl<double> LinearConstraint;

template <typename Scalar>
struct LinearConstraintDataTpl : public ConstraintDataTpl<Scalar> {
    LinearConstraintDataTpl(const LinearConstraintTpl<Scalar> &c)
        : ConstraintDataTpl<Scalar>(c),
          A(MatrixX<Scalar>::Zero(c.dim_output(), c.dim_tangent_space())),
          b(VectorX<Scalar>::Zero(c.dim_output())),
          A_s(c.dim_output(), c.dim_tangent_space()),
          b_s(c.dim_output()) {}

    MatrixX<Scalar> A;
    VectorX<Scalar> b;

    SparseMatrix<Scalar> A_s;
    SparseVector<Scalar> b_s;
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
                  EvaluatorBaseDataTpl<Scalar> &data) override {
        data.y = x;
    }

   private:
};

typedef BoundingBoxConstraintTpl<double> BoundingBoxConstraint;

}  // namespace bopt
