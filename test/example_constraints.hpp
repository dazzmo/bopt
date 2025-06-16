#include "bopt/Constraints.hpp"

class SumOfSquaresConstraint : public bopt::ConstraintTpl<double> {
   public:
    SumOfSquaresConstraint(const bopt::Size &n)
        : bopt::ConstraintTpl<double>("SumOfSquares", n, 1,
                                      bopt::ConstraintBoundType::ZERO) {}

    using Base = bopt::ConstraintTpl<double>;
    using Data = typename Base::Data;

   protected:
    void evalImpl(const Eigen::Ref<const Eigen::VectorXd> &x,
                  Data &data) const override {
        data.y << x.squaredNorm();
    }

    void evalJacobiansImpl(
        const Eigen::Ref<const Eigen::VectorXd> &x, Data &data,
        const bopt::JacobianEvaluationFlags &flags) const override {
        data.Jx << 2.0 * x;
    }
};
