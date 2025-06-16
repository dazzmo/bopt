#include "bopt/Costs.hpp"

class SumOfSquares : public bopt::CostTpl<double> {
   public:
    SumOfSquares(const bopt::Size &n) : bopt::CostTpl<double>("SumOfSquares", n) {}

    using Base = bopt::EvaluatorTpl<double, 1>;
    using Data = typename Base::Data;

   protected:
    void evalImpl(const Eigen::Ref<const Eigen::VectorXd> &x,
                  Data &data) const override {
        data.y = x.squaredNorm();
    }

    void evalGradientsImpl(
        const Eigen::Ref<const Eigen::VectorXd> &x, Data &data,
        const bopt::GradientEvaluationFlags &flags) const override {
        data.gx << 2.0 * x;
        data.gp << 0.0;
    }
};

class LinearCost : public bopt::LinearCostTpl<double> {
   public:
    LinearCost(const bopt::Size &n) : bopt::LinearCostTpl<double>("LinearCost", n) {}

    using Base = bopt::LinearCostTpl<double>;
    using Data = typename Base::Data;

   protected:

    void evalCoefficientsImpl(Data &data) const override {
        data.a.setOnes();
        data.b = 0.0;
    }
};
