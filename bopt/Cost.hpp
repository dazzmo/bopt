#pragma once

#include <memory>

#include "bopt/FunctionTraits.hpp"
#include "bopt/Logging.hpp"

namespace bopt {

/**
 *
 * @brief Cost function y = fₚ(x) ∈ ℝ
 *
 */
template <typename FunctionTraits>
class CostTpl : public EvaluatorTpl<FunctionTraits, 1> {};

template <typename Scalar>
using DenseCostTpl = CostTpl<DenseFunctionTraits<Scalar>>;

template <typename Scalar>
using SparseCostTpl = CostTpl<SparseFunctionTraits<Scalar>>;

using DenseCost = DenseCostTpl<Real>;
using SparseCost = SparseCostTpl<Real>;

}  // namespace bopt
