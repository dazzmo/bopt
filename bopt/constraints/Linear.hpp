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

template <typename ScalarType, SparsityType _Sparsity = SparsityType::DENSE>
class LinearConstraintTpl
    : public ConstraintTpl<ScalarType, _Sparsity>,
      public PolynomialEvaluator<
          LinearDataTpl<ScalarType, Eigen::Dynamic, _Sparsity>> {
   private:
   public:
    using ConstraintBase = ConstraintTpl<ScalarType, _Sparsity>;
    using Data = LinearDataTpl<ScalarType, Eigen::Dynamic, _Sparsity>;

    static constexpr SparsityType Sparsity = _Sparsity;

    LinearConstraintTpl(const String &name, const Size &n_in, const Size &n_out,
                        const ConstraintBoundType &bounds, const Size &n_p = 0)
        : ConstraintBase(name, n_in, n_out, bounds, n_p),
          PolynomialEvaluator<Data>() {}
};

}  // namespace bopt
