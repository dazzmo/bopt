#pragma once

#include "bopt/Evaluator.hpp"
#include "bopt/Logging.hpp"

namespace bopt {

template <typename EvaluatorTraits>
struct CostTpl;

template <typename EvaluatorTraits>
struct CostDataTpl : public EvaluatorDataTpl<EvaluatorTraits, 1> {
    CostDataTpl(const CostTpl<EvaluatorTraits> &c)
        : EvaluatorDataTpl<EvaluatorTraits, 1>(c.getEvaluator()) {}
};

}  // namespace bopt
