#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/Evaluator.hpp"
#include "bopt/Logging.hpp"
#include "bopt/Profiler.hpp"

class DenseEvaluatorTest : public bopt::EvaluatorTpl<double, Eigen::Dynamic> {
    using Base = bopt::EvaluatorTpl<double, Eigen::Dynamic>;

   public:
    using Data = typename Base::Data;

    DenseEvaluatorTest() : Base(2, 5, "Dense evaluator") {}

    void evalImpl(const Eigen::Ref<const typename Base::InputVector> &x,
                  Data &data) const {
        data.y << x[0] * x[1], x[0], 0, 0, 0;
    }

    void evalJacobiansImpl(
        const Eigen::Ref<const typename Base::InputVector> &x, Data &data,
        const bopt::JacobianEvaluationFlags &flags) const override {
        // data.Jx << x[1], x[0], 1.0, 0.0;
    }
};

class DenseLinearEvaluatorTest
    : public bopt::LinearEvaluatorTpl<double, Eigen::Dynamic> {
    using Base = bopt::LinearEvaluatorTpl<double, Eigen::Dynamic>;

   public:
    using EvaluatorData = typename Base::EvaluatorData;
    using Data = typename Base::Data;

    DenseLinearEvaluatorTest() : Base(2, 5, "Dense evaluator") {}

    void evalImpl(const Eigen::Ref<const typename Base::InputVector> &x,
                  EvaluatorData &data) const {
        data.y << x[0] * x[1], x[0], 0, 0, 0;
    }

    void evalJacobiansImpl(
        const Eigen::Ref<const typename Base::InputVector> &x,
        EvaluatorData &data,
        const bopt::JacobianEvaluationFlags &flags) const override {
        // data.Jx << x[1], x[0], 1.0, 0.0;
    }
};

// class ScalarEvaluator
//     : public bopt::EvaluatorTpl<bopt::DenseEvaluatorTraits<double>, 1> {
//     using Base = bopt::EvaluatorTpl<bopt::DenseEvaluatorTraits<double>, 1>;

//    public:
//     using Data = typename Base::Data;

//     ScalarEvaluator() : Base(2, "Dense evaluator") {}

//     void setDataSparsityImpl(Data &data) const override { data.gx.resize(2);
//     }

//     void evalImpl(const typename Base::InputVectorConstRef &x,
//                   Data &data) const {
//         data.y = x[0] * x[1];
//     }

//     void evalGradientsImpl(
//         const typename Base::InputVectorConstRef &x, Data &data,
//         const bopt::GradientEvaluationFlags &flags) const override {
//         data.gx << x[1], x[0];
//     }
// };

TEST(DenseEvaluator, Constructor) { DenseEvaluatorTest e; }

TEST(Wrapper, Conversion) {
    auto e = std::make_shared<DenseEvaluatorTest>();
    auto l = std::make_shared<DenseLinearEvaluatorTest>();

    std::shared_ptr<bopt::EvaluatorTpl<double, Eigen::Dynamic>> c = l;
}

// TEST(EvaluatorWrapper, Wrapper) {
//     auto e = std::make_shared<ScalarEvaluator>();
//     auto wrapper = bopt::internal::EvaluatorWrapper<
//         bopt::EvaluatorTpl<bopt::DenseEvaluatorTraits<double>, 1>>(e);
//     auto d = wrapper.createData();
//     Eigen::Vector2d x;
//     wrapper.evalGradients(x, *d);
// }

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    testing::InitGoogleTest(&argc, argv);
    int status = RUN_ALL_TESTS();
    bopt::Profiler summary;
    return status;
}