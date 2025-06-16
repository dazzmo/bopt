#pragma once

#include "bopt/costs/Cost.hpp"

namespace bopt {

/**
 * @brief Linear cost of the form c(x) = a^T x + b
 *
 * @tparam EvaluatorTraits
 * @tparam OutputSize
 */
template <typename ScalarType, SparsityType _Sparsity = SparsityType::DENSE>
class LinearCostTpl
    : public CostTpl<ScalarType, _Sparsity>,
      public PolynomialEvaluator<LinearDataTpl<ScalarType, 1, _Sparsity>> {
   public:
    using CostBase = CostTpl<ScalarType, _Sparsity>;
    using Data = LinearDataTpl<ScalarType, 1, _Sparsity>;

    static constexpr SparsityType Sparsity = CostBase::Sparsity;

    LinearCostTpl(const String &name, const Size &n_in,
                  const String &description = "")
        : CostTpl<ScalarType>(name, n_in, description),
          PolynomialEvaluator<LinearDataTpl<ScalarType, 1, Sparsity>>() {}
};

}  // namespace bopt
