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
template <typename EvaluatorTraits>
class CostTpl {
   public:
    using Scalar = typename EvaluatorTraits::Scalar;

    using Evaluator = EvaluatorTpl<EvaluatorTraits, 1>;

    using DenseVector = typename EvaluatorTraits::DenseVector;
    using InputVector = typename EvaluatorTraits::InputVector;
    using InputVectorConstRef = typename EvaluatorTraits::InputVectorConstRef;

    /**
     * @brief Construct a constraint from an existing evaluator and specifying
     * the bound type
     *
     * @param evaluator
     */
    CostTpl(const std::shared_ptr<Evaluator> &evaluator)
        : name_(""), evaluator_(evaluator) {}

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

    /**
     * @brief Returns the evaluator for the function
     *
     * @return Evaluator&
     */
    Evaluator &getEvaluator() const { return *evaluator_; }

    /**
     * @brief Set an evaluator for the of the constraint.
     *
     * @param evaluator
     */
    void setEvaluator(const std::shared_ptr<Evaluator> &evaluator) {
        evaluator_ = evaluator;
    }

    /**
     * @brief Evaluates the constraint c(x) with the input x.
     *
     * @param x
     * @param data
     */
    void eval(const InputVectorConstRef &x, Data &data) const {
        evaluator_->eval(x, data);
    }

    void evalGradients(const InputVectorConstRef &x, Data &data, bool compute_x,
                       bool compute_p) const {
        evaluator_->evalGradients(x, data, compute_x, compute_p);
    }

    void evalHessians(const InputVectorConstRef &x, Data &data, bool compute_xx,
                      bool compute_xp, bool compute_pp) const {
        evaluator_->evalHessians(x, data, compute_xx, compute_xp, compute_pp);
    }

   protected:
   private:
    /// @brief Name of the constraint
    String name_;
    /// @brief Shared pointer to the evaluator the constraint is associated with
    std::shared_ptr<Evaluator> evaluator_{nullptr};
};

template <typename Scalar>
using DenseCostTpl = CostTpl<DenseEvaluatorTraits<Scalar>>;
using DenseCost = DenseCostTpl<Real>;

template <typename Scalar>
using SparseCostTpl = CostTpl<SparseEvaluatorTraits<Scalar>>;
using SparseCost = SparseCostTpl<Real>;

template <typename EvaluatorType>
class PolynomialCostTpl
    : public CostTpl<typename EvaluatorType::EvaluatorTraits> {
   public:
    using Base = CostTpl<EvaluatorTraits>;
    using EvaluatorData = typename Base::EvaluatorData;
    using Data = typename EvaluatorType::Data;

    PolynomialCostTpl(const std::shared_ptr<EvaluatorType> &evaluator)
        : Base(evaluator), evaluator_(evaluator) {}

    void setDataSparsity(Data &data) const {
        evaluator_->setDataSparsity(data);
    }

    /**
     * @brief Returns the evaluator for the function
     *
     * @return Evaluator&
     */
    EvaluatorType &getEvaluator() const { return *evaluator_; }

    /**
     * @brief Set an evaluator for the of the constraint.
     *
     * @param evaluator
     */
    void setEvaluator(const std::shared_ptr<EvaluatorType> &evaluator) {
        Base::setEvaluator(evaluator);
        evaluator_ = evaluator;
    }

    void evalCoefficients(Data &data) const {
        evaluator_->evalCoefficients(data);
    }

   protected:
   private:
    std::shared_ptr<EvaluatorType> evaluator_{nullptr};
};

}  // namespace bopt