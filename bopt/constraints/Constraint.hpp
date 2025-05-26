#pragma once

#include "bopt/Evaluator.hpp"
#include "bopt/Logging.hpp"
#include "bopt/constraints/ConstraintBounds.hpp"
#include "bopt/constraints/ConstraintData.hpp"

namespace bopt {

/**
 * @brief Constraint bounds typical of many constraint types
 *
 */
enum class ConstraintBoundType {
    /// @brief Constraint of the form 0 = c(x) = 0
    ZERO,
    /// @brief Constraint of the form 0 <= c(x) = inf
    POSITIVE,
    /// @brief Constraint of the form -inf <= c(x) <= 0
    NEGATIVE,
    /// @brief Constraint of the form 0 < c(x) < inf
    STRICTLY_POSITIVE,
    /// @brief Constraint of the form -inf < c(x) < 0
    STRICTLY_NEGATIVE,
    /// @brief Custom bounds of the form c_l < c(x) < c_u
    CUSTOM
};

/**
 * @brief Constraint of the form y = fₚ(x) ∈ ℝᵐ
 *
 */
template <typename EvaluatorType>
class ConstraintTpl
    : public internal::EvaluatorWrapper<
          EvaluatorTpl<EvaluatorTraits, OutputSizeAtCompileTime>> {
    static constexpr bool is_scalar = OutputSizeAtCompileTime == 1;

   public:
    using Scalar = typename Base::Scalar;

    using DenseVector = typename Base::DenseVector;
    using InputVector = typename Base::InputVector;
    using InputVectorConstRef = typename Base::InputVectorConstRef;

    using Evaluator = typename Base::Evaluator;
    using Data = typename Base::Data;

    /**
     * @brief Construct a constraint from an existing evaluator and specifying
     * the bound type
     *
     * @param evaluator
     */
    ConstraintTpl(const String &name = "",
                  const std::shared_ptr<Evaluator> &evaluator,
                  const ConstraintBoundType &bounds)
        : internal::EvaluatorWrapper<
              EvaluatorTpl<EvaluatorTraits, OutputSizeAtCompileTime>>(
              evaluator),
          name_(name),
          type_(bounds) {}

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

    void setBoundsType(const ConstraintBoundType &type) { type_ = type; }
    const ConstraintBoundType &getBoundsType() const { return type_; }

    /**
     * @brief Sets the provided bound vectors to the type specified by the
     * constraint's type (accessed through getBoundsType())
     *
     * @param lb
     * @param ub
     */
    virtual void evalBounds(auto &&lb, auto &&ub) const {
        if constexpr (is_scalar) {
            switch (getBoundsType()) {
                case ConstraintBoundType::ZERO:
                    lb = 0.0;
                    ub = 0.0;
                    break;
                case ConstraintBoundType::POSITIVE:
                    lb = 0.0;
                    ub = kInf;
                    break;
                case ConstraintBoundType::NEGATIVE:
                    lb = -kInf;
                    ub = 0.0;
                    break;
                case ConstraintBoundType::STRICTLY_POSITIVE:
                    lb = kEpsilon;
                    ub = kInf;
                    break;
                case ConstraintBoundType::STRICTLY_NEGATIVE:
                    lb = -kInf;
                    ub = -kEpsilon;
                    break;
            }
        } else {
            assert(lb.size() == outputSize() && ub.size() == outputSize());
            switch (getBoundsType()) {
                case ConstraintBoundType::ZERO:
                    lb.setZero();
                    ub.setZero();
                    break;
                case ConstraintBoundType::POSITIVE:
                    lb.setZero();
                    ub.setConstant(kInf);
                    break;
                case ConstraintBoundType::NEGATIVE:
                    lb.setConstant(-kInf);
                    ub.setZero();
                    break;
                case ConstraintBoundType::STRICTLY_POSITIVE:
                    lb.setConstant(kEpsilon);
                    ub.setConstant(kInf);
                    break;
                case ConstraintBoundType::STRICTLY_NEGATIVE:
                    lb.setConstant(-kInf);
                    ub.setConstant(-kEpsilon);
                    break;
            }
        }
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
    bool isSatisfied(const Data &data, const Real epsilon = kEpsilon) const {
        if constexpr (is_scalar) {
            Scalar lb, ub;
            evalBounds(lb, ub);
            lb -= epsilon;
            ub += epsilon;
            return (data.y >= lb && data.y <= ub);
        } else {
            const Size m = data.y.size();
            DenseVector lb(m), ub(m);
            evalBounds(lb, ub);
            lb.array() -= epsilon;
            ub.array() += epsilon;
            for (Size i = 0; i < m; ++i) {
                if (data.y[i] > ub[i] || data.y[i] < lb[i]) return false;
            }
            return true;
        }
    }

   private:
    /// @brief Name of the constraint
    String name_;
    /// @brief Constraint bounds type
    ConstraintBoundType type_;
};

template <typename Scalar, int OutputSize = Eigen::Dynamic>
using DenseConstraintTpl =
    ConstraintTpl<DenseEvaluatorTraits<Scalar>, OutputSize>;
template <int OutputSize = Eigen::Dynamic>
using DenseConstraint = DenseConstraintTpl<Real, OutputSize>;

template <typename Scalar, int OutputSize = Eigen::Dynamic>
using SparseConstraintTpl =
    ConstraintTpl<SparseEvaluatorTraits<Scalar>, OutputSize>;
template <int OutputSize = Eigen::Dynamic>
using SparseConstraint = SparseConstraintTpl<Real, OutputSize>;

}  // namespace bopt
