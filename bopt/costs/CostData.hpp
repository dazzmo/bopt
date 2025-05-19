#pragma once

#include "bopt/Evaluator.hpp"
#include "bopt/Logging.hpp"

namespace bopt {

template <typename EvaluatorTraits>
struct CostDataTpl : public EvaluatorDataTpl<EvaluatorTraits, 1> {
    CostDataTpl(const CostTpl<EvaluatorTraits> &c)
        : EvaluatorDataTpl<EvaluatorTraits>(c) {}
};

}  // namespace bopt
