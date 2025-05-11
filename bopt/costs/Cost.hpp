#pragma once

#include <memory>

#include "bopt/EvaluatorTraits.hpp"
#include "bopt/Logging.hpp"

namespace bopt {

/**
 *
 * @brief Cost function y = fₚ(x) ∈ ℝ
 *
 */
template <typename EvaluatorTraits>
class CostTpl {
   public:
    using Evaluator = EvaluatorTpl<EvaluatorTraits, 1>;
    using Scalar = typename Base::Scalar;
    using EvaluatorData = typename Base::Data;

    CostTpl(const std::shared_ptr<Evaluator> &evaluator)
        : Base(cost), name_(""), weighting_(1.0) {}

    /**
     * @brief Name of the cost
     *
     * @return const String&
     */
    const String &getName() const { return name_; }

    /**
     * @brief Sets the name of the cost.
     *
     * @param name
     */
    void setName(const String &name) { name_ = name; }

   protected:
    CostTpl() : name_(""), weighting_(1.0) {}

    CostTpl(const std::shared_ptr<CostTpl<EvaluatorTraits>> &cost)
        : Base(cost), name_(""), weighting_(1.0) {}

   private:
    String name_;
    Scalar weighting_;

    std::shared_ptr<Evaluator> evaluator_;
};

template <typename Scalar>
using DenseCostTpl = CostTpl<DenseEvaluatorTraits<Scalar>>;
using DenseCost = DenseCostTpl<Real>;

template <typename Scalar>
using SparseCostTpl = CostTpl<SparseEvaluatorTraits<Scalar>>;
using SparseCost = SparseCostTpl<Real>;

}  // namespace bopt