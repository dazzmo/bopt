#pragma once

#include "bopt/constraints/ConstraintBase.hpp"

namespace bopt {

/**
 * @brief Constraint of the form lower_bound() <= x <= upper_bound()
 *
 */
template <typename EvaluatorTraits>
class BoundingBoxConstraintTpl : public ConstraintTpl<EvaluatorTraits> {
   public:
    using Base = ConstraintTpl<EvaluatorTraits>;

    using Scalar = typename Base::Scalar;

    using InputVector = typename Base::InputVector;
    using InputVectorConstRef = typename Base::InputVectorConstRef;

    using Vector = typename Base::Vector;
    using Matrix = typename Base::Matrix;

    using Data = typename Base::Data;
    using EvaluatorData = typename Base::EvaluatorData;

   protected:
    void evalImpl(const InputVectorConstRef &x,
                  EvaluatorData &data) const override {
        data.y = x;
    }

    void evalJacobiansImpl(const InputVectorConstRef &x, EvaluatorData &data,
                           bool compute_x, bool compute_p) const override {
        if (compute_x) data.Jx.setIdentity();
    }

   private:
};

}  // namespace bopt
