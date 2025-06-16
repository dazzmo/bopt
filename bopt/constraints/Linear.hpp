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
    using Base = ConstraintTpl<ScalarType, _Sparsity>;

   public:
    using Data = typename Base::Data;
    using LinearData = LinearDataTpl<ScalarType, Eigen::Dynamic, _Sparsity>;

    static constexpr SparsityType Sparsity = _Sparsity;

    LinearConstraintTpl(const String &name, const Size &n_in, const Size &n_out,
                        const ConstraintBoundType &bounds)
        : Base(name, n_in, n_out, bounds), PolynomialEvaluator<LinearData>() {}
};

}  // namespace bopt
