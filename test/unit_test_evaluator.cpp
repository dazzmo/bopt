#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/Evaluator.hpp"
#include "bopt/Logging.hpp"
#include "bopt/Profiler.hpp"

class DenseEvaluatorTest : public bopt::DenseEvaluatorTpl<double, 2> {
    using Base = bopt::DenseEvaluatorTpl<double, 2>;

   public:
    using Data = typename Base::Data;

    DenseEvaluatorTest() : Base(2, 2, "Dense evaluator") {}

    void setDataSparsityImpl(Data &data) const override {
        data.Jx.resize(2, 2);
    }

    void evalImpl(const typename Base::InputVectorConstRef &x, Data &data) const {
        data.y << x[0] * x[1], x[0];
    }

    void evalJacobiansImpl(const typename Base::InputVectorConstRef &x,
                           Data &data, bool compute_x,
                           bool compute_p) const override {
        data.Jx << x[1], x[0], 1.0, 0.0;
    }
};

class DenseScalarEvaluatorTest : public bopt::DenseEvaluatorTpl<double, 1> {
    using Base = bopt::DenseEvaluatorTpl<double, 1>;

   public:
    using Data = typename Base::Data;
    DenseScalarEvaluatorTest() : Base(2, "Dense evaluator") {}

    void setDataSparsityImpl(Data &data) const override {
        data.gx.resize(this->dimInputTangentSpace());
    }

    void evalImpl(const typename Base::InputVectorConstRef &x,
                  Data &data) const override {
        data.y = x[0] * x[1];
    }

    void evalGradientsImpl(const typename Base::InputVectorConstRef &x,
                           Data &data, bool compute_x,
                           bool compute_p) const override {
        data.gx << x[1], x[0];
    }
};

TEST(DenseEvaluator, Constructor) {
    DenseEvaluatorTest e;
    DenseScalarEvaluatorTest e_scalar;
    EXPECT_EQ(e.numInputs(), 2);

    std::cout << e << std::endl;

    DenseEvaluatorTest::Data data(e);
    DenseScalarEvaluatorTest::Data scalar_data(e_scalar);

    Eigen::VectorXd x(2);
    x.setRandom();
    {
        for (int i = 0; i < 100; ++i) {
            bopt::Profiler profiler("DenseEvaluatorTest");
            e.eval(x, data);
            e.evalJacobians(x, data);
        }
    }

    {
        for (int i = 0; i < 100; ++i) {
            bopt::Profiler profiler("DenseEvaluatorScalarTest");
            e_scalar.eval(x, scalar_data);
            e_scalar.evalGradients(x, scalar_data);
        }
    }
}

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    testing::InitGoogleTest(&argc, argv);
    int status = RUN_ALL_TESTS();
    bopt::Profiler summary;
    return status;
}