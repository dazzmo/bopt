#pragma once

#include "bopt/Evaluator.hpp"
#include "bopt/Logging.hpp"
#include "bopt/constraints/Bounds.hpp"
#include "bopt/constraints/Types.hpp"

namespace bopt {

template <typename Scalar>
struct ConstraintDataTpl;

/**
 * @brief Constraint of the form y = fₚ(x) ∈ ℝᵐ
 *
 */
template <typename EvaluatorTraits>
class ConstraintTpl : public EvaluatorTpl<EvaluatorTraits, Eigen::Dynamic> {
    using Base = EvaluatorTpl<EvaluatorTraits, Eigen::Dynamic>;

   public:
    using Scalar = typename Base::Scalar;

    using DenseVector = typename Base::DenseVector;
    using InputVector = typename Base::InputVector;
    using InputVectorConstRef = typename Base::InputVectorConstRef;

    using Vector = typename Base::Vector;
    using Matrix = typename Base::Matrix;

    using EvaluatorData = typename Base::Data;
    using Data = ConstraintDataTpl<EvaluatorTraits>;

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

    const ConstraintType &type() const { return type_; }

    std::shared_ptr<Data> createData() const {
        auto ptr = std::shared_ptr<Data>(this->createDataImpl());
        return ptr;
    }

    /**
     * @brief Set the constraint to a particular type
     *
     * @return const Type&
     */
    void setType(const ConstraintType &type) { type_ = type; }

    void setBounds(const ConstraintBounds &bounds) { bounds_ = bounds; }

    void setBounds(const Scalar &lb, const Scalar &ub) {
        bounds_ = ConstraintBounds::CUSTOM;
        lb_.setConstant(lb);
        ub_.setConstant(ub);
    }

    void setBounds(const InputVectorConstRef &lb,
                   const InputVectorConstRef &ub) {
        bounds_ = ConstraintBounds::CUSTOM;
        lb_ = lb;
        ub_ = ub;
    }

    // Derivatives with respect to parameters
    void evalBounds(Data &data) const { evalBoundsImpl(data); }

    void evalBoundJacobians(Data &data) const { evalBoundJacobiansImpl(data); }

    void evalBoundHessians(const InputVectorConstRef &lambda,
                           Data &data) const {
        evalBoundHessiansImpl(lambda, data);
    }

    /**
     * @brief Whether the constraints of the system are satisfied to a given
     * tolerance.
     *
     * @param value The current value of the constraint
     * @param epsilon Tolerance
     * @return true
     * @return false
     */
    bool isSatisfied(Data &data, const double &epsilon = kEpsilon) const {
        for (int i = 0; i < this->numOutputs(); ++i) {
            if (data.lb[i] - data.y[i] > epsilon ||
                data.ub[i] - data.y[i] < -epsilon)
                return false;
        }
        return true;
    }

   protected:
    ConstraintTpl(const Index &dim_input, const Index &dim_output,
                  const ConstraintBounds &bounds = ConstraintBounds::ZERO)
        : EvaluatorTpl<EvaluatorTraits>(dim_input, dim_output),
          name_(""),
          type_(ConstraintType::EQUALITY),
          bounds_(dim_output, bounds),
          ptr_(nullptr) {}

    /**
     * @brief Construct a constraint from an existing evaluator and specifying
     * the bound type
     *
     * @param evaluator
     */
    ConstraintTpl(const std::shared_ptr<Base> &evaluator)
        : Base(evaluator),
          name_(""),
          type_(ConstraintType::EQUALITY),
          bounds_(ConstraintBounds::ZERO),
          lb_(DenseVector::Zero(evaluator->numOutputs())),
          ub_(DenseVector::Zero(evaluator->numOutputs())),
          ptr_(nullptr) {}

    virtual Data *createDataImpl() const {
        Data *data = new Data(*this);
        return data;
    }

    virtual void evalBoundsImpl(Data &data) const {
        if (ptr_) {
            ptr_->evalBounds(data);
        } else {
            // Based off given type
            if (bounds_ != ConstraintBounds::CUSTOM) {
                setBoundsFromType(data.lb, data.ub, bounds_);
            } else {
                data.lb = lb_;
                data.ub = ub_;
            }
        }
    }

    virtual void evalBoundJacobiansImpl(Data &data) const {
        if (ptr_) ptr_->evalBoundJacobians(data);
    }

    virtual void evalBoundHessiansImpl(const InputVectorConstRef &lambda,
                                       Data &data) const {
        if (ptr_) ptr_->evalBoundHessians(lambda, data);
    }

   private:
    String name_;
    Bounds bounds_;

    std::shared_ptr<ConstraintTpl> ptr_;

    /// @brief Manually set constraint lower bounds
    InputVector lb_;
    /// @brief Manually set constraint upper bounds
    InputVector ub_;
};

template <typename Scalar>
using DenseConstraintTpl = ConstraintTpl<DenseEvaluatorTraits<Scalar>>;
using DenseConstraint = DenseConstraintTpl<Real>;

template <typename Scalar>
using SparseConstraintTpl = ConstraintTpl<SparseEvaluatorTraits<Scalar>>;
using SparseConstraint = SparseConstraintTpl<Real>;

}  // namespace bopt
