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

/**
 * @brief Constraint bounds typical of many constraint types
 *
 */
enum class ConstraintBounds {
    /// Constraint of the form 0 = c(x) = 0
    ZERO,
    /// Constraint of the form 0 <= c(x) = inf
    POSITIVE,
    /// Constraint of the form -inf <= c(x) <= 0
    NEGATIVE,
    /// Constraint of the form 0 < c(x) < inf
    STRICTLY_POSITIVE,
    /// Constraint of the form -inf < c(x) < 0
    STRICTLY_NEGATIVE,
    /// Constraint of the form lb <= c(x) <= ub
    CUSTOM
};

template <typename Scalar>
struct ConstraintDataTpl;

/**
 * @brief Constraint of the form y = fₚ(x) ∈ ℝᵐ
 *
 */
template <typename FunctionTraits>
class ConstraintTpl : public EvaluatorTpl<FunctionTraits> {
    using Base = EvaluatorTpl<FunctionTraits>;

   public:
    using Scalar = typename Base::Scalar;

    using InputVector = typename Base::InputVector;
    using InputVectorConstRef = typename Base::InputVectorConstRef;

    using Vector = typename Base::Vector;
    using Matrix = typename Base::Matrix;

    using EvaluatorData = typename Base::Data;
    using Data = ConstraintDataTpl<FunctionTraits>;

    /**
     * @brief Construct a constraint from an existing evaluator and specifying
     * the bound type
     *
     * @param evaluator
     */
    ConstraintTpl(
        const std::shared_ptr<EvaluatorTpl<FunctionTraits>> &evaluator,
        const ConstraintBounds &bounds)
        : EvaluatorTpl<FunctionTraits>(evaluator),
          name_(""),
          type_(ConstraintType::Equality),
          bounds_(bounds),
          lb_(InputVector::Zero(0)),
          ub_(InputVector::Zero(0)),
          ptr_(nullptr) {
        assert(bounds != ConstraintBounds::CUSTOM);
    }

    /**
     * @brief Construct a constraint from an existing evaluator, adding fixed
     * bounds from scalar values
     *
     * @param evaluator
     */
    ConstraintTpl(
        const std::shared_ptr<EvaluatorTpl<FunctionTraits>> &evaluator,
        const Scalar &lb, const Scalar &ub)
        : EvaluatorTpl<FunctionTraits>(evaluator),
          name_(""),
          type_(ConstraintType::Equality),
          bounds_(ConstraintBounds::CUSTOM),
          lb_(InputVector::Constant(evaluator->getOutputDimension(), lb)),
          ub_(InputVector::Constant(evaluator->getOutputDimension(), ub)),
          ptr_(nullptr) {}

    /**
     * @brief Construct a constraint from an existing evaluator and fixed bound
     * vectors
     *
     * @param evaluator
     */
    ConstraintTpl(
        const std::shared_ptr<EvaluatorTpl<FunctionTraits>> &evaluator,
        const InputVectorConstRef &lb, const InputVectorConstRef &ub)
        : EvaluatorTpl<FunctionTraits>(evaluator),
          name_(""),
          type_(ConstraintType::Equality),
          bounds_(ConstraintBounds::CUSTOM),
          lb_(lb),
          ub_(ub),
          ptr_(nullptr) {}

    /**
     * @brief Construct a constraint from an existing constraint
     *
     * @param evaluator
     */
    ConstraintTpl(
        const std::shared_ptr<ConstraintTpl<FunctionTraits>> &constraint)
        : EvaluatorTpl<FunctionTraits>(constraint),
          name_(""),
          type_(ConstraintType::Equality),
          bounds_(constraint->getBounds()),
          lb_(InputVector::Zero(0)),
          ub_(InputVector::Zero(0)),
          ptr_(constraint) {}

    const ConstraintType &type() const { return type_; }

    std::shared_ptr<Data> createData() const {
        return std::shared_ptr<Data>(this->createDataImpl());
    }

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

    void evalBoundHessians(const InputVectorConstRef &lambda,
                           Data &data) const {
        evalBoundHessiansImpl(lambda, data);
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
        for (int i = 0; i < this->getOutputDimension(); ++i) {
            if (data.lb[i] - data.y[i] > epsilon ||
                data.ub[i] - data.y[i] < -epsilon)
                return false;
        }
        return true;
    }

    void setBounds(const ConstraintBounds &bounds) { bounds_ = bounds; }

    void setBounds(const Scalar &lb, const Scalar &ub) {
        bounds_ = ConstraintBounds::CUSTOM;
        lb_.setConstant(lb);
        ub_.setConstant(ub);
    }

    void setBounds(const InputVectorConstRef &lb,
                   const InputVectorConstRef &ub) {
        bounds_ = ConstraintBounds::CUSTOM;
        lb_ = lb;
        ub_ = ub;
    }

   protected:
    ConstraintTpl(const Index &dim_input, const Index &dim_output,
                  const ConstraintBounds &bounds = ConstraintBounds::ZERO)
        : EvaluatorTpl<FunctionTraits>(dim_input, dim_output),
          name_(""),
          type_(ConstraintType::Equality),
          bounds_(bounds),
          ptr_(nullptr) {}

    virtual Data *createDataImpl() const {
        Data *data = new Data(*this);
        return data;
    }

    virtual void evalBoundsImpl(Data &data) const {
        if (ptr_) {
            ptr_->evalBounds(data);
        } else {
            // Based off given type
            switch (bounds_) {
                case ConstraintBounds::ZERO:
                    data.lb.setZero();
                    data.ub.setZero();
                    break;
                case ConstraintBounds::POSITIVE:
                    data.lb.setZero();
                    data.ub.setConstant(kInf);
                    break;
                case ConstraintBounds::NEGATIVE:
                    data.lb.setConstant(-kInf);
                    data.ub.setZero();
                    break;
                case ConstraintBounds::STRICTLY_POSITIVE:
                    data.lb.setConstant(kEpsilon);
                    data.ub.setConstant(kInf);
                    break;
                case ConstraintBounds::STRICTLY_NEGATIVE:
                    data.lb.setConstant(-kInf);
                    data.ub.setConstant(-kEpsilon);
                    break;
                case ConstraintBounds::CUSTOM:
                    data.lb = lb_;
                    data.ub = ub_;
                default:
                    break;
            }
        }
    }

    virtual void evalBoundJacobiansImpl(Data &data) const {
        if (ptr_) ptr_->evalBoundJacobians(data);
    }

    virtual void evalBoundHessiansImpl(const InputVectorConstRef &lambda,
                                       Data &data) const {
        if (ptr_) ptr_->evalBoundHessians(lambda, data);
    }

   private:
    std::string name_;
    ConstraintType type_;
    ConstraintBounds bounds_;
    std::shared_ptr<ConstraintTpl> ptr_;
    /// @brief Manually set constraint lower bounds
    InputVector lb_;
    /// @brief Manually set constraint upper bounds
    InputVector ub_;
};

template <typename Scalar>
using DenseConstraintTpl = ConstraintTpl<DenseFunctionTraits<Scalar>>;

template <typename Scalar>
using SparseConstraintTpl = ConstraintTpl<SparseFunctionTraits<Scalar>>;

using DenseConstraint = DenseConstraintTpl<Real>;
using SparseConstraint = SparseConstraintTpl<Real>;

template <typename FunctionTraits>
struct ConstraintDataTpl : public EvaluatorDataTpl<FunctionTraits> {
    using Base = EvaluatorDataTpl<FunctionTraits>;

    using DenseVector = typename Base::DenseVector;
    using Vector = typename Base::Vector;
    using Matrix = typename Base::Matrix;

    ConstraintDataTpl(const ConstraintTpl<FunctionTraits> &c)
        : EvaluatorDataTpl<FunctionTraits>(c) {
        const Index p = c.getNumberOfParameters();
        const Index m = c.getOutputDimension();

        lb = DenseVector::Zero(m);
        ub = DenseVector::Zero(m);
        if constexpr (FunctionTraits::type == "Sparse") {
            // Sparse: allocate sparse objects properly
            Jlb_p.resize(m, p);
            Jub_p.resize(m, p);
            Hlb_pp.resize(p, p);
            Hub_pp.resize(p, p);
        } else {
            // Dense
            Jlb_p = Matrix::Zero(m, p);
            Jub_p = Matrix::Zero(m, p);
            Hlb_pp = Matrix::Zero(p, p);
            Hub_pp = Matrix::Zero(p, p);
        }
    }

    /// Lower bound of the constraint
    DenseVector lb;
    /// Upper bound of the constraint
    DenseVector ub;

    /// Jacobian of the lower bound with respect to the parameters
    Matrix Jlb_p;
    /// Jacobian of the upper bound with respect to the parameters
    Matrix Jub_p;

    /// Hessian of the lower bound vector product with respect to the parameters
    Matrix Hlb_pp;
    /// Hessian of the upper bound vector product with respect to the parameters
    Matrix Hub_pp;
};

template <typename Scalar>
struct LinearConstraintDataTpl;

/**
 * @brief Constraint of the form lb ≤ Ax ≤ ub
 *
 */
template <typename FunctionTraits>
class LinearConstraintTpl : public ConstraintTpl<FunctionTraits> {
   public:
    using EvaluatorData = typename ConstraintTpl<FunctionTraits>::EvaluatorData;
    using ConstraintData = typename ConstraintTpl<FunctionTraits>::Data;
    using Data = LinearConstraintDataTpl<FunctionTraits>;

    std::shared_ptr<Data> createData() const {
        return std::shared_ptr<Data>(this->createDataImpl());
    }

    /**
     * @brief Evaluates the vector coeffcient vector bₚ for the constraint lb ≤
     * Ax ≤ ub
     *
     * @param A Coefficient matrix Aₚ
     */
    void evalCoefficients(Data &data) const { evalCoefficientsImpl(data); }

   protected:
    LinearConstraintTpl(const Index &dim_input, const Index &dim_output)
        : ConstraintTpl<FunctionTraits>(dim_input, dim_output) {
        this->setName("linear_constraint");
    }

    LinearConstraintTpl(
        const std::shared_ptr<ConstraintTpl<FunctionTraits>> &constraint)
        : ConstraintTpl<FunctionTraits>(constraint) {
        this->setName("linear_constraint");
    }

    virtual void evalCoefficientsImpl(Data &data) const {}

    virtual Data *createDataImpl() const { return new Data(*this); }

   private:
};

template <typename Scalar>
using DenseLinearConstraintTpl =
    LinearConstraintTpl<DenseFunctionTraits<Scalar>>;

template <typename Scalar>
using SparseLinearConstraintTpl =
    LinearConstraintTpl<SparseFunctionTraits<Scalar>>;

using DenseLinearConstraint = DenseLinearConstraintTpl<Real>;
using SparseLinearConstraint = SparseLinearConstraintTpl<Real>;

template <typename FunctionTraits>
struct LinearConstraintDataTpl : public ConstraintDataTpl<FunctionTraits> {
    using Base = ConstraintDataTpl<FunctionTraits>;
    using Matrix = typename Base::Matrix;

    LinearConstraintDataTpl(const LinearConstraintTpl<FunctionTraits> &c)
        : ConstraintDataTpl<FunctionTraits>(c) {
        if constexpr (FunctionTraits::type == "Sparse") {
            // Sparse: allocate sparse objects properly
            A.resize(c.getOutputDimension(), c.getInputDimension());
        } else {
            A = Matrix::Zero(c.getOutputDimension(), c.getInputDimension());
        }
    }

    Matrix A;
};

/**
 * @brief Constraint of the form lower_bound() <= x <= upper_bound()
 *
 */
template <typename FunctionTraits>
class BoundingBoxConstraintTpl : public ConstraintTpl<FunctionTraits> {
   public:
    using Base = ConstraintTpl<FunctionTraits>;

    using Scalar = typename Base::Scalar;

    using InputVector = typename Base::InputVector;
    using InputVectorConstRef = typename Base::InputVectorConstRef;

    using Vector = typename Base::Vector;
    using Matrix = typename Base::Matrix;

    using VectorInput = typename Base::VectorInput;
    using MatrixInput = typename Base::MatrixInput;

    using VectorConstInput = typename Base::VectorConstInput;
    using MatrixConstInput = typename Base::MatrixConstInput;

    using Data = typename Base::Data;

    BoundingBoxConstraintTpl(const Index &dim_input,
                             const InputVectorConstRef &lower_bound,
                             const InputVectorConstRef &upper_bound)
        : ConstraintTpl<Scalar>(dim_input, dim_input),
          lb_(lower_bound),
          ub_(upper_bound) {}

    BoundingBoxConstraintTpl(const Index &dim_input, const Scalar &lower_bound,
                             const Scalar &upper_bound)
        : ConstraintTpl<Scalar>(dim_input, dim_input),
          lb_(InputVector::Constant(dim_input, lower_bound)),
          ub_(InputVector::Constant(dim_input, upper_bound)) {}

   protected:
    void evalImpl(const InputVectorConstRef &x, Data &data) const override {
        data.y = x;
    }

    void evalJacobiansImpl(const InputVectorConstRef &x, Data &data,
                           bool compute_x, bool compute_p) const override {
        data.Jx.setIdentity();
    }

    void evalBoundsImpl(ConstraintDataTpl<Scalar> &data) const override {
        data.lb = lb_;
        data.ub = ub_;
    }

   private:
    /// Constant bounds that are set at initialisation
    InputVector lb_;
    InputVector ub_;
};

template <typename Scalar>
using DenseBoundingBoxConstraintTpl =
    BoundingBoxConstraintTpl<DenseFunctionTraits<Scalar>>;

template <typename Scalar>
using SparseBoundingBoxConstraintTpl =
    BoundingBoxConstraintTpl<SparseFunctionTraits<Scalar>>;

using DenseBoundingBoxConstraint = DenseBoundingBoxConstraintTpl<Real>;
using SparseBoundingBoxConstraint = SparseBoundingBoxConstraintTpl<Real>;

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
