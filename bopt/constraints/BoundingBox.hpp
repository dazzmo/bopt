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
template <typename ScalarType>
class BoundingBoxConstraintTpl
    : public ConstraintTpl<ScalarType, SparsityType::DENSE> {
    using Base = ConstraintTpl<ScalarType, SparsityType::DENSE>;
    using Scalar = typename Base::Scalar;

   public:
    using DenseVector = typename Base::DenseVector;
    using InputVector = typename Base::InputVector;
    using OutputType = typename Base::OutputType;

    using Data = typename Base::Data;

    BoundingBoxConstraintTpl()
        : Base("bounding_box_constraint", 0, 0, ConstraintBoundType::NEGATIVE) {
        lb_ = OutputType::Zero(this->outputSize());
        ub_ = OutputType::Zero(this->outputSize());
    }

    BoundingBoxConstraintTpl(const Eigen::Ref<const DenseVector> &lb,
                             const Eigen::Ref<const DenseVector> &ub)
        : Base("bounding_box_constraint", lb.size(), 2 * lb.size(),
               ConstraintBoundType::NEGATIVE),
          lb_(lb),
          ub_(ub) {}

    void evalBoundingBoxBounds(DenseVector &lb, DenseVector &ub) const {
        evalBoundingBoxBoundsImpl(lb, ub);
    }

   protected:
    virtual void evalBoundingBoxBoundsImpl(DenseVector &lb,
                                           DenseVector &ub) const {}

   private:
    mutable DenseVector lb_;
    mutable DenseVector ub_;

    void evalImpl(const Eigen::Ref<const InputVector> &x,
                  Data &data) const override {
        this->evalBoundingBoxBounds(lb_, ub_);
        data.y << lb_ - x, x - ub_;
    }

    void evalJacobiansImpl(
        const Eigen::Ref<const InputVector> &x, Data &data,
        const JacobianEvaluationFlags &flags) const override {
        if (flags.compute_x) {
            const Index n = this->inputSize();
            data.Jx.topLeftCorner(n, n).diagonal().array() = -1.0;
            data.Jx.bottomRightCorner(n, n).diagonal().array() = 1.0;
        }
    }
};

}  // namespace bopt
