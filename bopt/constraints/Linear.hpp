#pragma once

#include "bopt/Logging.hpp"
#include "bopt/constraints/Constraint.hpp"

namespace bopt {

/**
 * @brief Linear constraint of the form c(x) = Ax + b
 *
 * @tparam EvaluatorTraits
 * @tparam OutputSize
 */
template <typename LinearEvaluatorType>
class LinearConstraintTpl : public ConstraintTpl<LinearEvaluatorType> {
    using Base = ConstraintTpl<LinearEvaluatorType>;

   public:
    LinearConstraintTpl(const std::shared_ptr<LinearEvaluatorType> &evaluator)
        : Base(evaluator) {}

    void evalCoefficients(typename LinearEvaluatorType::Data &data) const {
        this->getEvaluator().evalCoefficients(data);
    }
};

template <typename Scalar, int OutputSizeAtCompileTime>
using DenseLinearConstraintTpl = LinearConstraintTpl<
    DenseLinearEvaluatorTpl<Scalar, OutputSizeAtCompileTime>>;
template <int OutputSizeAtCompileTime>
using DenseLinearConstraint =
    DenseLinearConstraintTpl<Real, OutputSizeAtCompileTime>;

}  // namespace bopt
