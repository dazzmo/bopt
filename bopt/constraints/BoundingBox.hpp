#pragma once

#include "bopt/Logging.hpp"
#include "bopt/constraints/Constraint.hpp"

namespace bopt {

/**
 * @brief Bounding box constraint of the form lb(p) <= x <= ub(p). As a standard
 * constraint, it represented as the c(x, p) =  [lb(p) - x; x - ub(p)] <= 0
 *
 * @tparam EvaluatorTraits
 * @tparam OutputSize
 */
template <typename EvaluatorType>
class BoundingBoxConstraintTpl : public ConstraintTpl<EvaluatorType> {
    using Base = ConstraintTpl<EvaluatorType>;
    using Scalar = typename Base::Scalar;

   public:
    using InputVectorConstRef = typename Base::InputVectorConstRef;
    using OutputType = typename Base::OutputType;

    BoundingBoxConstraintTpl(const std::shared_ptr<EvaluatorType> &evaluator)
        : Base(evaluator, ConstraintBoundType::NEGATIVE) {
        if constexpr (!Base::IsScalar) {
            lb_ = OutputType::Zero(this->numOutputs());
            ub_ = OutputType::Zero(this->numOutputs());
        } else {
            lb_ = Scalar(0);
            ub_ = Scalar(0);
        }
    }

    void evalBoundingBoxBounds(Scalar &lb, Scalar &ub) const {
        static_assert(
            EvaluatorType::IsScalar,
            "You are calling a scalar method on a vector constraint!");
        evalBoundingBoxBoundsImpl(lb, ub);
    }

    void evalBoundingBoxBounds(Eigen::Ref<DenseVector> lb,
                               Eigen::Ref<DenseVector> ub) const {
        static_assert(
            EvaluatorType::IsScalar,
            "You are calling a vector method on a scalar constraint!");
        evalBoundingBoxBoundsImpl(lb, ub);
    }

   protected:
    virtual void evalBoundingBoxBoundsImpl(Scalar &lb, Scalar &ub) const {}

    virtual void evalBoundingBoxBoundsImpl(Eigen::Ref<DenseVector> lb,
                                           Eigen::Ref<DenseVector> ub) const {}

   private:
    OutputType lb_;
    OutputType ub_;

    void evalImpl(const InputVectorConstRef &x, Data &data) const override {
        evalBoundingBoxBounds(lb_, ub_);
        data.c << lb_ - x, x - ub_;
    }

    void evalJacobiansImpl(
        Data &data, const JacobianEvaluationFlags &flags) const override {
        if (flags.compute_x) {
            if constexpr (EvaluatorType::Type == FunctionType::SPARSE) {
            } else {
                data.Jx.topLeftCorner().diagonal().array() = -1.0;
                data.Jx.bottomRightCorner().diagonal().array() = 1.0;
            }
        }
    }
};

template <typename Scalar, int OutputSizeAtCompileTime>
using DenseBoundingBoxConstraintTpl = BoundingBoxConstraintTpl<
    DenseBoundingBoxEvaluatorTpl<Scalar, OutputSizeAtCompileTime>>;
template <int OutputSizeAtCompileTime>
using DenseLinearConstraint =
    DenseLinearConstraintTpl<Real, OutputSizeAtCompileTime>;

}  // namespace bopt
