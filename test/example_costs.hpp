#include "bopt/Costs.hpp"

class CostEvaluator : public bopt::DenseEvaluator<1> {
   public:
    CostEvaluator() : bopt::DenseEvaluator<1>(1) {}

    using Base = bopt::DenseEvaluator<1>;
    using InputVectorConstRef = typename Base::InputVectorConstRef;
    using Data = typename Base::Data;

   protected:
    void evalImpl(const InputVectorConstRef &x, Data &data) const override {
        data.y = 1.0;
    }

    void evalGradientsImpl(const InputVectorConstRef &x, Data &data,
                           bool compute_x, bool compute_p) const override {
        data.gx << 1.0;
        data.gp << 0.0;
    }
};

class LinearCostEvaluator
    : public bopt::LinearEvaluatorTpl<bopt::DenseEvaluatorTraits<double>, 1> {
   public:
    using Base =
        bopt::LinearEvaluatorTpl<bopt::DenseEvaluatorTraits<double>, 1>;
    using Data = typename Base::Data;
    using EvaluatorData = typename Base::EvaluatorData;

    LinearCostEvaluator() : Base(2, "linear cost") {}

   protected:
    void evalImpl(const InputVectorConstRef &x,
                  EvaluatorData &data) const override {
        data.y = 1.0 * x[0] + 2 * x[1] + 1.0;
    }

    void evalGradientsImpl(const InputVectorConstRef &x, EvaluatorData &data,
                           bool compute_x, bool compute_p) const override {
        data.gx << 1.0, 2.0;
    }

    void evalCoefficientsImpl(Data &data) const override {
        data.a << 1.0, 2.0;
        data.b = 1.0;
    }
};