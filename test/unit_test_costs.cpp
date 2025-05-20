#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/Costs.hpp"
#include "bopt/Logging.hpp"
#include "bopt/Profiler.hpp"




TEST(Cost, Construction) {
    auto e = std::make_shared<CostEvaluator>();
    auto c = bopt::DenseCost(e);
    auto d = c.createData();
}

TEST(LinearCost, Construction) {
    auto e = std::make_shared<LinearCostEvaluator>();
    auto c = bopt::DenseLinearCost(e);
    auto d = c.createData();
    c.evalCoefficients(d);
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