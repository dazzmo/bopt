#pragma once

#include "bopt/Evaluator.hpp"
#include "bopt/Logging.hpp"

namespace bopt {

/**
 * @brief Constraint bounds typical of many constraint types
 *
 */
enum class ConstraintBounds {
    /// Constraint of the form 0 = c(x) = 0
    ZERO,
    /// Constraint of the form 0 <= c(x) = inf
    POSITIVE,
    /// Constraint of the form -inf <= c(x) <= 0
    NEGATIVE,
    /// Constraint of the form 0 < c(x) < inf
    STRICTLY_POSITIVE,
    /// Constraint of the form -inf < c(x) < 0
    STRICTLY_NEGATIVE,
    /// Constraint of the form lb <= c(x) <= ub
    CUSTOM
};

std::ostream &operator<<(std::ostream &os, const ConstraintBounds &b) {
    return os;
}

template <typename Scalar>
class Bounds {
   public:
    using Type = ConstraintBounds;
    using Vector = typename MathTypes<Scalar>::VectorX;

    Bounds() : type_(), lb_(Vector::Zero(0)), ub_(Vector::Zero(0)) {}

    Bounds(const Size &size, const Type &type = Type::ZERO)
        : type_(type), lb_(Vector::Zero(size)), ub_(Vector::Zero(size)) {
        setBounds(type);
    }

    const Type &getType() const { return type_; }
    const Vector &getLowerBound() const { return lb_; }
    const Vector &getUpperBound() const { return ub_; }

    void setBounds(const Scalar &lb, const Scalar &ub);
    void setBounds(const ConstraintBounds &type);
    void setBounds(const Eigen::Ref<const Vector> &lb,
                   const const Eigen::Ref<const Vector> &ub);

   private:
    Type type_;
    Vector lb_;
    Vector ub_;
};

template <typename Scalar>
void setBoundsFromType(Eigen::Ref<typename MathTypes<Scalar>::VectorX> &lb,
                       Eigen::Ref<typename MathTypes<Scalar>::VectorX> &ub,
                       const ConstraintBounds &bounds) {
    // Based off given type
    switch (bounds) {
        case ConstraintBounds::ZERO:
            data.lb.setZero();
            data.ub.setZero();
            break;
        case ConstraintBounds::POSITIVE:
            data.lb.setZero();
            data.ub.setConstant(kInf);
            break;
        case ConstraintBounds::NEGATIVE:
            data.lb.setConstant(-kInf);
            data.ub.setZero();
            break;
        case ConstraintBounds::STRICTLY_POSITIVE:
            data.lb.setConstant(kEpsilon);
            data.ub.setConstant(kInf);
            break;
        case ConstraintBounds::STRICTLY_NEGATIVE:
            data.lb.setConstant(-kInf);
            data.ub.setConstant(-kEpsilon);
            break;
        default:
            break;
    }
};

}  // namespace bopt