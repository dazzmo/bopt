#pragma once

#include "bopt/costs/Cost.hpp"

namespace bopt {

/**
 * @brief Quadratic cost of the form c(x) = a^T x + b
 *
 * @tparam EvaluatorTraits
 * @tparam OutputSize
 */
template <typename ScalarType, SparsityType Sparsity = SparsityType::DENSE>
class QuadraticCostTpl
    : public CostTpl<ScalarType, Sparsity>,
      public PolynomialEvaluator<QuadraticDataTpl<ScalarType, Sparsity>> {
   public:
    using Data = typename CostTpl<ScalarType, Sparsity>::Data;

    QuadraticCostTpl(const String &name, const Size &n_in,
                     const String &description = "")
        : CostTpl<ScalarType>(name, n_in, description),
          PolynomialEvaluator<QuadraticDataTpl<ScalarType, Sparsity>>() {}
};
}  // namespace bopt
