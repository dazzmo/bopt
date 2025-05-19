#pragma once

#include "bopt/constraints/ConstraintData.hpp"

namespace bopt {

/**
 * @brief Constraint bounds typical of many constraint types
 *
 */
enum class ConstraintBoundType {
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

/**
 * @brief Whether the constraint is an equality of inequality constraint
 *
 */
enum class ConstraintType {
    /// @brief Constraint of the form lower_bound() = c(x) = upper_bound()
    EQUALITY,
    /// @brief Constraint of the form lower_bound() ≤ c(x) ≤ upper_bound()
    INEQUALITY
};

// std::ostream &operator<<(std::ostream &os, const ConstraintBoundType &b) {
//     return os;
// }

template <typename EvaluatorTraits>
class BoundEvaluatorTpl {
   public:
    using Scalar = typename EvaluatorTraits::Scalar;

    using DenseVector = typename EvaluatorTraits::DenseVector;
    using InputVector = typename EvaluatorTraits::InputVector;
    using InputVectorConstRef = typename EvaluatorTraits::InputVectorConstRef;

    using Data = ConstraintDataTpl<EvaluatorTraits>;

    BoundEvaluatorTpl(
        const Size &num_parameters,
        const ConstraintBoundType &bounds = ConstraintBoundType::CUSTOM)
        : type_(bounds), parameters_(DenseVector::Zero(num_parameters)) {}

    /**
     * @brief Set the sparsity of any entries within the provided data
     * structure.
     *
     * @param data
     */
    void setDataSparsity(Data &data) const { setDataSparsityImpl(data); }

    /**
     * @brief Evaluates the expression y = fₚ(x) using variables x and
     * parameters p (set through \ref BoundsEvaluatorTpl::setParameters()).
     *
     * @param data
     */
    void eval(Data &data) const { evalImpl(data); }

    /**
     * @brief Computes the jacobians of the expression f.
     *
     * @param data
     */
    void evalJacobians(Data &data) const { evalJacobiansImpl(data); }

    /**
     * @brief Computes the lower-triangular hessians of the vector-product of
     * the upper and lower bounds with the vector λ.
     *
     * @param lambda
     * @param data
     */
    void evalHessians(const InputVectorConstRef &lambda, Data &data) const {
        evalHessiansImpl(lambda, data);
    }

    const DenseVector &getParameters() const { return parameters_; }
    void setParameters(const InputVectorConstRef &parameters) {
        parameters_ = parameters;
    }

    Size numParameters() const { return parameters_.size(); }

   protected:
    BoundEvaluatorTpl(const Index &n_in, const Index &n_out) {}

    virtual void setDataSparsityImpl(Data &data) const {}

    /**
     * @brief Implementation of the evaluator
     *
     * @param x
     * @param out
     */
    virtual void evalImpl(Data &data) const {
        if (type_ == ConstraintBoundType::ZERO) {
            data.lb.setZero();
            data.ub.setZero();
        } else if (type_ == ConstraintBoundType::POSITIVE) {
            data.lb.setZero();
            data.ub.setConstant(kInf);
        } else if (type_ == ConstraintBoundType::NEGATIVE) {
            data.lb.setConstant(-kInf);
            data.ub.setZero();
        } else if (type_ == ConstraintBoundType::STRICTLY_POSITIVE) {
            data.lb.setConstant(kEpsilon);
            data.ub.setConstant(kInf);
        } else if (type_ == ConstraintBoundType::STRICTLY_NEGATIVE) {
            data.lb.setConstant(-kInf);
            data.ub.setConstant(-kEpsilon);
        }
    }

    virtual void evalJacobiansImpl(Data &data) const {}

    virtual void evalHessiansImpl(const InputVectorConstRef &lambda,
                                  Data &data) const {}

   private:
    ConstraintBoundType type_;
    DenseVector parameters_;
};

}  // namespace bopt
