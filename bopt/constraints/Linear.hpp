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
template <typename EvaluatorTraits,
          int OutputSizeAtCompileTime = Eigen::Dynamic>
class LinearConstraintTpl
    : public ConstraintTpl<EvaluatorTraits, OutputSizeAtCompileTime> {
   public:
    using Base = ConstraintTpl<EvaluatorTraits, OutputSizeAtCompileTime>;
    using LinearEvaluator =
        LinearEvaluatorTpl<EvaluatorTraits, OutputSizeAtCompileTime>;
    using LinearEvaluatorData =
        LinearEvaluatorDataTpl<EvaluatorTraits, OutputSizeAtCompileTime>;

    LinearConstraintTpl(const std::shared_ptr<LinearEvaluator> &evaluator)
        : Base(evaluator), evaluator_(evaluator) {}

    void setDataSparsity(Data &data) const {
        evaluator_->setDataSparsity(data);
    }

    /**
     * @brief Returns the evaluator for the function
     *
     * @return Evaluator&
     */
    LinearEvaluator &getLinearEvaluator() const { return *evaluator_; }

    /**
     * @brief Set an evaluator for the of the constraint.
     *
     * @param evaluator
     */
    void setLinearEvaluator(const std::shared_ptr<LinearEvaluator> &evaluator) {
        Base::setEvaluator(evaluator);
        evaluator_ = evaluator;
    }

    void evalCoefficients(LinearEvaluatorData &data) const {
        evaluator_->evalCoefficients(data);
    }

   protected:
   private:
    std::shared_ptr<LinearEvaluator> evaluator_{nullptr};
};

template <typename Scalar, int OutputSizeAtCompileTime = Eigen::Dynamic>
using DenseLinearConstraintTpl =
    LinearConstraintTpl<DenseEvaluatorTraits<Scalar>, OutputSizeAtCompileTime>;
template <int OutputSizeAtCompileTime = Eigen::Dynamic>
using DenseLinearConstraint =
    DenseLinearConstraintTpl<Real, OutputSizeAtCompileTime>;

template <typename Scalar, int OutputSizeAtCompileTime = Eigen::Dynamic>
using SparseLinearConstraintTpl =
    LinearConstraintTpl<SparseEvaluatorTraits<Scalar>, OutputSizeAtCompileTime>;
template <int OutputSizeAtCompileTime = Eigen::Dynamic>
using SparseLinearConstraint =
    SparseLinearConstraintTpl<Real, OutputSizeAtCompileTime>;

}  // namespace bopt
