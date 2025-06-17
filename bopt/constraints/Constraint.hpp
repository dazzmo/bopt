#pragma once

#include "bopt/Evaluator.hpp"
#include "bopt/Logging.hpp"

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
    STRICTLY_NEGATIVE
};

template <typename ScalarType, SparsityType _Sparsity = SparsityType::DENSE>
class ConstraintTpl
    : public EvaluatorTpl<ScalarType, Eigen::Dynamic, _Sparsity> {
    using Base = EvaluatorTpl<ScalarType, Eigen::Dynamic, _Sparsity>;

   public:
    using Scalar = typename Base::Scalar;
    using DenseVector = typename Base::DenseVector;
    using Data = typename Base::Data;

    static constexpr SparsityType Sparsity = _Sparsity;

    ConstraintTpl(const String &name, const Size &n_in, const Size &n_out,
                  const ConstraintBoundType &type, const Size &n_p = 0,
                  const String &description = "")
        : name_(name), type_(type), Base(n_in, n_out, n_p, description) {}

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

    void evalBounds(Eigen::Ref<DenseVector> lb,
                    Eigen::Ref<DenseVector> ub) const {
        // Ensure that the correct method is called depending on the output of
        // the constraint evaluator
        assert(lb.size() == this->outputSize() &&
               ub.size() == this->outputSize());
        Scalar lb_val, ub_val;
        this->evalBoundsImpl(lb_val, ub_val);
        lb.setConstant(lb_val);
        ub.setConstant(ub_val);
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
        DenseVector lb(this->getOutputSize()), ub(this->getOutputSize());
        evalBounds(lb, ub);
        lb.array() -= epsilon;
        ub.array() += epsilon;
        return (data.y.array() >= lb.array()).all() &&
               (data.y.array() <= ub.array()).all();
    }

    /**
     * @brief Whether the constraint is of the form 0 == c(x, p) == 0
     *
     * @return true
     * @return false
     */
    bool isEquality() const { return type_ == ConstraintBoundType::ZERO; }

    /**
     * @brief Whether the constraint is of the form cl <= c(x, p) <= cu
     *
     * @return true
     * @return false
     */
    bool isInequality() const { return type_ != ConstraintBoundType::ZERO; }

   private:
    /// @brief Name of the constraint
    String name_;
    /// @brief Constraint bounds type
    ConstraintBoundType type_;

    void evalBoundsImpl(Scalar &lb, Scalar &ub) const {
        switch (getBoundsType()) {
            case ConstraintBoundType::ZERO:
                lb = Scalar(0);
                ub = Scalar(0);
                break;
            case ConstraintBoundType::POSITIVE:
                lb = Scalar(0);
                ub = kInf;
                break;
            case ConstraintBoundType::NEGATIVE:
                lb = -kInf;
                ub = Scalar(0);
                break;
            case ConstraintBoundType::STRICTLY_POSITIVE:
                lb = kEpsilon;
                ub = kInf;
                break;
            case ConstraintBoundType::STRICTLY_NEGATIVE:
                lb = -kInf;
                ub = -kEpsilon;
                break;
            default:
                lb = Scalar(0);
                ub = Scalar(0);
        }
    }
};

}  // namespace bopt
