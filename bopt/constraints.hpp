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
template <typename InputTraits>
class ConstraintTpl : public EvaluatorTpl<InputTraits> {
    using Base = EvaluatorTpl<InputTraits>;

    using Scalar = typename Base::Scalar;

    using InputVector = typename Base::InputVector;
    using InputVectorConstRef = typename Base::InputVectorConstRef;

    using Vector = typename Base::Vector;
    using Matrix = typename Base::Matrix;

    using VectorInput = typename Base::VectorInput;
    using MatrixInput = typename Base::MatrixInput;

    using VectorConstInput = typename Base::VectorConstInput;
    using MatrixConstInput = typename Base::MatrixConstInput;

    using Data = ConstraintDataTpl<InputTraits>;

   public:
    const ConstraintType &type() const { return type_; }

    virtual std::shared_ptr<Data> createData() = 0;

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
    void evalBounds(Data &data) const { evalBoundsImpl(data); }

    void evalBoundJacobians(Data &data) const { evalBoundJacobiansImpl(data); }

    void evalBoundSparseJacobians(Data &data) const {
        evalBoundSparseJacobiansImpl(data);
    }

    void evalBoundHessians(const Eigen::Ref<const VectorX<Scalar>> &lambda,
                           Data &data) const {
        evalBoundHessiansImpl(lambda, data);
    }

    void evalBoundSparseHessians(
        const Eigen::Ref<const VectorX<Scalar>> &lambda, Data &data) const {
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
    bool isSatisfied(Data &data, const double &epsilon = kEpsilon) const {
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

    virtual void evalBoundsImpl(Data &data) const {
        if (ptr_) ptr_->evalBounds(data);
    }

    virtual void evalBoundJacobiansImpl(Data &data) const {
        if (ptr_) ptr_->evalBoundJacobians(data);
    }
    virtual void evalBoundSparseJacobiansImpl(Data &data) const {
        if (ptr_) ptr_->evalBoundSparseJacobians(data);
    }

    virtual void evalBoundHessiansImpl(
        const Eigen::Ref<const VectorX<Scalar>> &lambda, Data &data) const {
        if (ptr_) ptr_->evalBoundHessians(lambda, data);
    }
    virtual void evalBoundSparseHessiansImpl(
        const Eigen::Ref<const VectorX<Scalar>> &lambda, Data &data) const {
        if (ptr_) ptr_->evalBoundSparseHessians(lambda, data);
    }

   private:
    std::string name_;
    ConstraintType type_;

    std::shared_ptr<ConstraintTpl> ptr_;
};

template <typename Scalar>
using DenseConstraintTpl = ConstraintTpl<DenseInputTraits<Scalar>>;

template <typename Scalar>
using SparseConstraintTpl = ConstraintTpl<SparseInputTraits<Scalar>>;

typedef ConstraintTpl<double> Constraint;

template <typename InputTraitType>
struct ConstraintDataTpl : public EvaluatorDataTpl<InputTraitType> {
    using Base = EvaluatorDataTpl<InputTraitType>;

    using Vector = typename Base::Vector;
    using Matrix = typename Base::Matrix;

    using VectorInput = typename Base::VectorInput;
    using MatrixInput = typename Base::MatrixInput;

    using VectorConstInput = typename Base::VectorConstInput;
    using MatrixConstInput = typename Base::MatrixConstInput;

    ConstraintDataTpl(const ConstraintTpl<Scalar> &c)
        : EvaluatorDataTpl<Scalar>(c),
          lb(Vector::Zero(c.dim_output())),
          ub(Vector::Zero(c.dim_output())),
          Jlb_p(Matrix::Zero(c.dim_output(), c.dim_parameter())),
          Jub_p(Matrix::Zero(c.dim_output(), c.dim_parameter())),
          Hlb_pp(Matrix::Zero(c.dim_parameter(), c.dim_parameter())),
          Hub_pp(Matrix::Zero(c.dim_parameter(), c.dim_parameter())) {}

    /// Lower bound of the constraint
    Vector lb;
    /// Upper bound of the constraint
    Vector ub;

    /// Jacobian of the lower bound with respect to the parameters
    Matrix Jlb_p;
    /// Jacobian of the upper bound with respect to the parameters
    Matrix Jub_p;

    /// Hessian of the lower bound vector product with respect to the parameters
    Matrix Hlb_pp;
    /// Hessian of the upper bound vector product with respect to the parameters
    Matrix Hub_pp;
};

typedef ConstraintDataTpl<double> ConstraintData;

template <typename Scalar>
struct LinearConstraintDataTpl;

/**
 * @brief Constraint of the form lb ≤ Ax ≤ ub
 *
 */
template <typename InputTraits>
class LinearConstraintTpl : public ConstraintTpl<InputTraits> {
    using LinearConstraintData = LinearConstraintDataTpl<InputTraits>;

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

    virtual std::shared_ptr<Data> createData() = 0;

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

   private:
};

template <typename Scalar>
using DenseLinearConstraint = LinearConstraintTpl<DenseInputTraits<Scalar>>;

template <typename Scalar>
using SparseLinearConstraint = LinearConstraintTpl<SparseInputTraits<Scalar>>;

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

// /**
//  * @brief Constraint of the form lower_bound() <= x <= upper_bound()
//  *
//  */
// template <typename Scalar>
// class BoundingBoxConstraintTpl : public LinearConstraintTpl<Scalar> {
//    public:
//     BoundingBoxConstraintTpl(
//         const Index &dim_input,
//         const Eigen::Ref<const VectorX<Scalar>> &lower_bound,
//         const Eigen::Ref<const VectorX<Scalar>> &upper_bound)
//         : LinearConstraintTpl<Scalar>(dim_input, dim_input),
//           lb_(lower_bound),
//           ub_(upper_bound) {}

//     BoundingBoxConstraintTpl(const Index &dim_input, const Scalar &lower_bound,
//                              const Scalar &upper_bound)
//         : LinearConstraintTpl<Scalar>(dim_input, dim_input),
//           lb_(VectorX<Scalar>::Constant(dim_input, lower_bound)),
//           ub_(VectorX<Scalar>::Constant(dim_input, upper_bound)) {}

//     static std::shared_ptr<BoundingBoxConstraintTpl> create(
//         const Index &dim_input,
//         const Eigen::Ref<const VectorX<Scalar>> &lower_bound,
//         const Eigen::Ref<const VectorX<Scalar>> &upper_bound) {
//         return std::make_shared<BoundingBoxConstraintTpl>(
//             dim_input, lower_bound, upper_bound);
//     }

//     static std::shared_ptr<BoundingBoxConstraintTpl> create(
//         const Index &dim_input, const Scalar &lower_bound,
//         const Scalar &upper_bound) {
//         return std::make_shared<BoundingBoxConstraintTpl>(
//             dim_input, lower_bound, upper_bound);
//     }

//    protected:
//     void evalImpl(const Eigen::Ref<const VectorX<Scalar>> &x,
//                   EvaluatorDataTpl<Scalar> &data) const override {
//         data.y = x;
//     }

//     void evalJacobiansImpl(const Eigen::Ref<const VectorX<Scalar>> &x,
//                            EvaluatorDataTpl<Scalar> &data, bool compute_x,
//                            bool compute_p) const override {
//         data.Jx.setIdentity();
//     }

//     void evalBoundsImpl(ConstraintDataTpl<Scalar> &data) const override {
//         data.lb = lb_;
//         data.ub = ub_;
//     }

//    private:
//     /// Constant bounds that are set at initialisation
//     VectorX<Scalar> lb_;
//     VectorX<Scalar> ub_;
// };

// typedef BoundingBoxConstraintTpl<double> BoundingBoxConstraint;

// template <typename Scalar>
// struct MatrixConstraintDataTpl;
// /**
//  * @brief Matrix constraint of the form x₁ A₁ + x₂ A₂ + ... + xₙ Aₙ ≽ 0
//  *
//  * @tparam Scalar
//  */
// template <typename Scalar>
// class MatrixInequalityConstraintTpl {
//    public:
//     void eval(const Eigen::Ref<const VectorX<Scalar>> &x,
//               MatrixConstraintDataTpl<Scalar> &data) const {
//         // todo CHECK();
//         evalImpl(x, data);
//     }

//    private:
//     virtual void evalImpl(const Eigen::Ref<const VectorX<Scalar>> &x,
//                           MatrixConstraintDataTpl<Scalar> &data) const {}
// };

// template <typename Scalar>
// struct MatrixConstraintDataTpl {
//     /// @brief Series of matrices for the inequality x₁ A₁ + x₂ A₂ + ... + xₙ
//     /// Aₙ ≽ 0
//     std::vector<MatrixX<Scalar>> A;
// };

}  // namespace bopt
