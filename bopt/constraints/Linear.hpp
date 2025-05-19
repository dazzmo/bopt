#pragma once

#include "bopt/Evaluator.hpp"
#include "bopt/Logging.hpp"
#include "bopt/constraints/Constraint.hpp"

namespace bopt {

template <typename EvaluatorTraits>
using LinearConstraintTpl = PolynomialConstraintTpl<
    LinearEvaluatorTpl<EvaluatorTraits, Eigen::Dynamic>>;

template <typename Scalar>
using DenseLinearConstraintTpl =
    LinearConstraintTpl<DenseEvaluatorTraits<Scalar>>;
using DenseLinearConstraint = DenseLinearConstraintTpl<Real>;

template <typename Scalar>
using SparseLinearConstraintTpl =
    LinearConstraintTpl<SparseEvaluatorTraits<Scalar>>;
using SparseLinearConstraint = SparseLinearConstraintTpl<Real>;

}  // namespace bopt
