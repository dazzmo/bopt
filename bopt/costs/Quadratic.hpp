#pragma once

#include "bopt/costs/Cost.hpp"

namespace bopt {

// Forward declaration of data type
template <typename EvaluatorTraits>
struct QuadraticCostDataTpl;

/**
 * @brief Quadratic cost of the form fₚ(x) = aₚᵀx + bₚ
 *
 */
template <typename EvaluatorTraits>
using QuadraticCostTpl =
    PolynomialCostTpl<QuadraticEvaluatorTpl<EvaluatorTraits, 1>>;

template <typename Scalar>
using DenseQuadraticCostTpl = QuadraticCostTpl<DenseEvaluatorTraits<Scalar>>;
using DenseQuadraticCost = DenseQuadraticCostTpl<Real>;

template <typename Scalar>
using SparseQuadraticCostTpl = QuadraticCostTpl<SparseEvaluatorTraits<Scalar>>;
using SparseQuadraticCost = SparseQuadraticCostTpl<Real>;

}  // namespace bopt
