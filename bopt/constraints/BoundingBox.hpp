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

    BoundingBoxConstraintTpl(const Index &dim_input,
                             const InputVectorConstRef &lower_bound,
                             const InputVectorConstRef &upper_bound)
        : ConstraintTpl<EvaluatorTraits>(dim_input, dim_input),
          lb_(lower_bound),
          ub_(upper_bound) {}

    BoundingBoxConstraintTpl(const Index &dim_input, const Scalar &lower_bound,
                             const Scalar &upper_bound)
        : ConstraintTpl<EvaluatorTraits>(dim_input, dim_input),
          lb_(InputVector::Constant(dim_input, lower_bound)),
          ub_(InputVector::Constant(dim_input, upper_bound)) {}

   protected:
    void evalImpl(const InputVectorConstRef &x,
                  EvaluatorData &data) const override {
        data.y = x;
    }

    void evalJacobiansImpl(const InputVectorConstRef &x, EvaluatorData &data,
                           bool compute_x, bool compute_p) const override {
        if (compute_x) data.Jx.setIdentity();
    }

    void evalBoundsImpl(Data &data) const override {
        data.lb = lb_;
        data.ub = ub_;
    }

   private:
    /// Constant bounds that are set at initialisation
    InputVector lb_;
    InputVector ub_;
};

template <typename Scalar>
using DenseBoundingBoxConstraintTpl =
    BoundingBoxConstraintTpl<DenseEvaluatorTraits<Scalar>>;
using DenseBoundingBoxConstraint = DenseBoundingBoxConstraintTpl<Real>;

template <typename Scalar>
using SparseBoundingBoxConstraintTpl =
    BoundingBoxConstraintTpl<SparseEvaluatorTraits<Scalar>>;
using SparseBoundingBoxConstraint = SparseBoundingBoxConstraintTpl<Real>;

}  // namespace bopt
