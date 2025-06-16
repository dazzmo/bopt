#pragma once

#include <memory>

#include "bopt/Evaluator.hpp"
#include "bopt/Logging.hpp"

namespace bopt {

/**
 *
 * @brief Cost function y = fₚ(x) ∈ ℝ
 *
 */
template <typename ScalarType, SparsityType _Sparsity = SparsityType::DENSE>
class CostTpl : public EvaluatorTpl<ScalarType, 1, _Sparsity> {
   public:
    static constexpr SparsityType Sparsity = _Sparsity;

    using Scalar = ScalarType;
    using Data = typename EvaluatorTpl<ScalarType, 1, Sparsity>::Data;

    /**
     * @brief Construct a constraint from an existing evaluator and specifying
     * the bound type
     *
     * @param evaluator
     */
    CostTpl(const String &name, const Size &n_in,
            const String &description = "")
        : name_(name),
          EvaluatorTpl<ScalarType, 1, Sparsity>(n_in, description) {}

    /**
     * @brief Name of the constraint
     *
     * @return const String&
     */
    const String &getName() const { return name_; }

    /**
     * @brief Sets the name of the constraint.
     *
     * @param name
     */
    void setName(const String &name) { name_ = name; }

   protected:
   private:
    /// @brief Name of the constraint
    String name_;
};

}  // namespace bopt