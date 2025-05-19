#pragma once

#include "bopt/costs/Cost.hpp"

namespace bopt {

// Forward declaration of data type
template <typename EvaluatorTraits>
struct LinearCostDataTpl;

/**
 * @brief Linear cost of the form fₚ(x) = aₚᵀx + bₚ
 *
 */
template <typename EvaluatorTraits>
using LinearCostTpl = PolynomialCostTpl<LinearEvaluatorTpl<EvaluatorTraits, 1>>;

template <typename Scalar>
using DenseLinearCostTpl = LinearCostTpl<DenseEvaluatorTraits<Scalar>>;
using DenseLinearCost = DenseLinearCostTpl<Real>;

template <typename Scalar>
using SparseLinearCostTpl = LinearCostTpl<SparseEvaluatorTraits<Scalar>>;
using SparseLinearCost = SparseLinearCostTpl<Real>;

}  // namespace bopt
