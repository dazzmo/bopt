#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/Costs.hpp"
#include "bopt/Logging.hpp"
#include "bopt/Profiler.hpp"
#include "example_costs.hpp"

TEST(SumOfSquares, Construction) {
    auto c = SumOfSquares(10);
    auto d = SumOfSquares::Data(c);
}

TEST(SumOfSquares, Evaluation) {
    auto c = SumOfSquares(10);
    auto d = SumOfSquares::Data(c);

    Eigen::VectorXd x(10);
    x.setRandom();

    c.eval(x, d);
}

TEST(LinearCost, Construction) {
    auto c = LinearCost(10);
    auto d = LinearCost::Data(c);
}

TEST(LinearCost, Conversion) {
    auto c = std::make_shared<LinearCost>(10);

    std::shared_ptr<bopt::CostTpl<double>> c_ptr = c; 
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