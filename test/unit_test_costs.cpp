#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/Costs.hpp"
#include "bopt/Logging.hpp"
#include "bopt/Profiler.hpp"

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
    void evalImpl(const InputVectorConstRef &x, EvaluatorData &data) const override {
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


TEST(Cost, Construction) {
    auto e = std::make_shared<CostEvaluator>();
    auto c = bopt::DenseCost(e);
}

int main(int argc, char **argv) {
    FLAGS_logtostderr = true;
    FLAGS_v = 10;

    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    testing::InitGoogleTest(&argc, argv);
    int status = RUN_ALL_TESTS();
    bopt::Profiler summary;
    return status;
}