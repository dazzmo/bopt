#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/Profiler.hpp"
#include "bopt/ad/Casadi.hpp"
#include "casadi_expressions.hpp"

using SX = ::casadi::SX;
using dm = ::casadi::DM;

TEST(Casadi, ScalarEvaluator) {
    for (const auto& test : getScalarTestExpressions()) {
        bopt::casadi::Evaluator<double, 1> e(test.expr, test.x, test.p);

        EXPECT_EQ(e.inputSize(), test.x.rows()) << test.name;
        EXPECT_EQ(e.dimInputSpace(), test.x.rows()) << test.name;
        EXPECT_EQ(e.dimInputTangentSpace(), test.x.rows()) << test.name;
        EXPECT_EQ(e.outputSize(), 1) << test.name;
        EXPECT_EQ(e.dimOutputSpace(), 1) << test.name;

        EXPECT_EQ(e.numParameters(), test.p.rows()) << test.name;

        // Create data
        bopt::EvaluatorDataTpl<double, 1> data(e);

        EXPECT_EQ(data.gx.rows(), test.x.rows()) << test.name;
        EXPECT_EQ(data.gp.rows(), test.p.rows()) << test.name;
        EXPECT_EQ(data.Hxx.rows(), test.x.rows()) << test.name;
        EXPECT_EQ(data.Hxx.cols(), test.x.rows()) << test.name;
        EXPECT_EQ(data.Hxp.rows(), test.x.rows()) << test.name;
        EXPECT_EQ(data.Hxp.cols(), test.p.rows()) << test.name;
        EXPECT_EQ(data.Hpp.rows(), test.p.rows()) << test.name;
        EXPECT_EQ(data.Hpp.cols(), test.p.rows()) << test.name;
    }
}

TEST(Casadi, ScalarLinearEvaluator) {
    for (const auto& test : getScalarTestExpressions()) {
        if (test.name != "linear") {
            EXPECT_THROW(
                {
                    bopt::casadi::LinearCost<double> e(test.expr, test.x,
                                                       test.p);
                },
                std::runtime_error);
        } else {
            bopt::casadi::LinearCost<double> e(test.expr, test.x, test.p);

            EXPECT_EQ(e.inputSize(), test.x.rows()) << test.name;
            EXPECT_EQ(e.dimInputSpace(), test.x.rows()) << test.name;
            EXPECT_EQ(e.dimInputTangentSpace(), test.x.rows()) << test.name;
            EXPECT_EQ(e.outputSize(), 1) << test.name;
            EXPECT_EQ(e.dimOutputSpace(), 1) << test.name;

            EXPECT_EQ(e.numParameters(), test.p.rows()) << test.name;

            // Create data
            bopt::LinearDataTpl<double, 1> data(e);

            EXPECT_EQ(data.gx.rows(), test.x.rows()) << test.name;
            EXPECT_EQ(data.gp.rows(), test.p.rows()) << test.name;
            EXPECT_EQ(data.Hxx.rows(), test.x.rows()) << test.name;
            EXPECT_EQ(data.Hxx.cols(), test.x.rows()) << test.name;
            EXPECT_EQ(data.Hxp.rows(), test.x.rows()) << test.name;
            EXPECT_EQ(data.Hxp.cols(), test.p.rows()) << test.name;
            EXPECT_EQ(data.Hpp.rows(), test.p.rows()) << test.name;
            EXPECT_EQ(data.Hpp.cols(), test.p.rows()) << test.name;

            EXPECT_EQ(data.a.rows(), test.x.rows()) << test.name;

            // Try set of random values
            Eigen::VectorXd x(4), p(4);
            for (int i = 0; i < 10; ++i) {
                x.setRandom();
                p.setRandom();
                e.setParameters(p);
                e.eval(x, data);
                e.evalGradients(x, data,
                                bopt::GradientEvaluationFlags(true, true));
                e.evalCoefficients(data);

                EXPECT_DOUBLE_EQ(data.y, x.dot(p));
                EXPECT_TRUE(data.gx.isApprox(p));
                EXPECT_TRUE(data.gp.isApprox(x));
                EXPECT_TRUE(data.a.isApprox(p));
            }
        }
    }
}

TEST(Casadi, QuadraticEvaluator) {
    for (const auto& test : getScalarTestExpressions()) {
        if (test.name != "quadratic" && test.name != "linear") {
            EXPECT_THROW(
                {
                    bopt::casadi::QuadraticCost<double> e(test.expr, test.x,
                                                          test.p);
                },
                std::runtime_error);
        } else {
            bopt::casadi::QuadraticCost<double> e(test.expr, test.x, test.p);

            EXPECT_EQ(e.inputSize(), test.x.rows()) << test.name;
            EXPECT_EQ(e.dimInputSpace(), test.x.rows()) << test.name;
            EXPECT_EQ(e.dimInputTangentSpace(), test.x.rows()) << test.name;
            EXPECT_EQ(e.outputSize(), 1) << test.name;
            EXPECT_EQ(e.dimOutputSpace(), 1) << test.name;

            EXPECT_EQ(e.numParameters(), test.p.rows()) << test.name;

            // Create data
            bopt::QuadraticDataTpl<double> data(e);

            EXPECT_EQ(data.gx.rows(), test.x.rows()) << test.name;
            EXPECT_EQ(data.gp.rows(), test.p.rows()) << test.name;
            EXPECT_EQ(data.Hxx.rows(), test.x.rows()) << test.name;
            EXPECT_EQ(data.Hxx.cols(), test.x.rows()) << test.name;
            EXPECT_EQ(data.Hxp.rows(), test.x.rows()) << test.name;
            EXPECT_EQ(data.Hxp.cols(), test.p.rows()) << test.name;
            EXPECT_EQ(data.Hpp.rows(), test.p.rows()) << test.name;
            EXPECT_EQ(data.Hpp.cols(), test.p.rows()) << test.name;

            EXPECT_EQ(data.A.rows(), test.x.rows()) << test.name;
            EXPECT_EQ(data.A.cols(), test.x.rows()) << test.name;
            EXPECT_EQ(data.b.rows(), test.x.rows()) << test.name;

            // Try set of random values
            Eigen::VectorXd x(4), p(4);
            for (int i = 0; i < 10; ++i) {
                x.setRandom();
                p.setRandom();
                e.setParameters(p);
                e.eval(x, data);
                e.evalGradients(x, data,
                                bopt::GradientEvaluationFlags(true, true));
                // e.evalCoefficients(data);

                Eigen::MatrixXd P = p.asDiagonal();

                // EXPECT_DOUBLE_EQ(data.y, x.dot(p));
                EXPECT_TRUE(data.gx.isApprox(2 * P * x));
                // EXPECT_TRUE(data.gp.isApprox(x));

                // Eigen::MatrixXd P = p.asDiagonal();
                // EXPECT_TRUE(data.A.isApprox(P));
            }
        }
    }
}

int main(int argc, char** argv) {
    // FLAGS_logtostderr = true;
    // FLAGS_v = 10;

    // google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    testing::InitGoogleTest(&argc, argv);
    int status = RUN_ALL_TESTS();
    bopt::Profiler summary;
    return status;
}
