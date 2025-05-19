#pragma once

#include "bopt/Evaluator.hpp"
#include "bopt/Logging.hpp"
#include "bopt/constraints/ConstraintBounds.hpp"
#include "bopt/constraints/ConstraintData.hpp"

namespace bopt {

/**
 * @brief Constraint of the form y = fₚ(x) ∈ ℝᵐ
 *
 */
template <typename EvaluatorTraits>
class ConstraintTpl {
   public:
    using Scalar = typename EvaluatorTraits::Scalar;

    using Evaluator = EvaluatorTpl<EvaluatorTraits, Eigen::Dynamic>;
    using BoundEvaluator = BoundEvaluatorTpl<EvaluatorTraits>;

    using DenseVector = typename EvaluatorTraits::DenseVector;
    using InputVector = typename EvaluatorTraits::InputVector;
    using InputVectorConstRef = typename EvaluatorTraits::InputVectorConstRef;

    using Data = ConstraintDataTpl<EvaluatorTraits>;

    /**
     * @brief Construct a constraint from an existing evaluator and specifying
     * the bound type
     *
     * @param evaluator
     */
    ConstraintTpl(const std::shared_ptr<Evaluator> &evaluator,
                  const ConstraintBoundType &bounds)
        : name_(""),
          evaluator_(evaluator),
          bound_evaluator_(std::make_shared<BoundEvaluator>(
              evaluator->numOutputs(), bounds)) {}

    /**
     * @brief Construct a constraint from an existing evaluator and specifying
     * the bound type
     *
     * @param evaluator
     */
    ConstraintTpl(const std::shared_ptr<Evaluator> &evaluator,
                  const std::shared_ptr<BoundEvaluator> &bound_evaluator)
        : name_(""), evaluator_(evaluator), bound_evaluator_(bound_evaluator) {}

    Data createData() const { return Data(*this); }

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
     * @brief Returns the bound evaluator for the function
     *
     * @return BoundEvaluator&
     */
    BoundEvaluator &getBoundEvaluator() const { return *bound_evaluator_; }

    /**
     * @brief Set an evaluator for the bounds of the constraint.
     *
     * @param evaluator
     */
    void setBoundEvaluator(const std::shared_ptr<BoundEvaluator> &evaluator) {
        bound_evaluator_ = evaluator_;
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

    void evalJacobians(const InputVectorConstRef &x, Data &data, bool compute_x,
                       bool compute_p) const {
        evaluator_->evalJacobians(x, data, compute_x, compute_p);
    }

    void evalHessians(const InputVectorConstRef &x, Data &data, bool compute_xx,
                      bool compute_xp, bool compute_pp) const {
        evaluator_->evalHessians(x, data, compute_xx, compute_xp, compute_pp);
    }

    /**
     * @brief Evaluate the bounds of the constraint.
     *
     * @param data
     */
    void evalBounds(Data &data) const { bound_evaluator_->eval(data); }

   protected:
   private:
    /// @brief Name of the constraint
    String name_;
    /// @brief Shared pointer to the evaluator the constraint is associated with
    std::shared_ptr<Evaluator> evaluator_{nullptr};
    /// @brief Shared pointer to the evaluator the constraint bounds are
    /// associated with
    std::shared_ptr<BoundEvaluator> bound_evaluator_{nullptr};
};

template <typename Scalar>
using DenseConstraintTpl = ConstraintTpl<DenseEvaluatorTraits<Scalar>>;
using DenseConstraint = DenseConstraintTpl<Real>;

template <typename Scalar>
using SparseConstraintTpl = ConstraintTpl<SparseEvaluatorTraits<Scalar>>;
using SparseConstraint = SparseConstraintTpl<Real>;

template <typename EvaluatorType>
class PolynomialConstraintTpl
    : public ConstraintTpl<typename EvaluatorType::EvaluatorTraits> {
   public:
    using Base = ConstraintTpl<typename EvaluatorType::EvaluatorTraits>;
    using EvaluatorData = typename Base::EvaluatorData;
    using Data = typename EvaluatorType::Data;

    PolynomialConstraintTpl(const std::shared_ptr<EvaluatorType> &evaluator)
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
