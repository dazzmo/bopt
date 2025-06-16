#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/Binding.hpp"
#include "bopt/Costs.hpp"
#include "bopt/Logging.hpp"
#include "bopt/Profiler.hpp"
#include "example_costs.hpp"

TEST(CostEvaluator, Binding) {
    std::vector<Eigen::Index> indices = {0};
    auto e = std::make_shared<CostEvaluator>();
    auto cost = std::make_shared<bopt::CostTpl<CostEvaluator>>(e);
    auto data = std::make_shared<bopt::EvaluatorDataTpl<double, 1>>(*e);

    bopt::Binding<bopt::CostTpl<CostEvaluator>> b(cost, data, indices);
}

TEST(LinearCostEvaluator, Binding) {
    std::vector<Eigen::Index> indices = {0};
    auto e = std::make_shared<LinearCostEvaluator>();
    auto cost = std::make_shared<bopt::CostTpl<LinearCostEvaluator>>(e);
    auto data = std::make_shared<bopt::LinearEvaluatorDataTpl<double, 1>>(*e);

    bopt::Binding<bopt::CostTpl<LinearCostEvaluator>> b(cost, data, indices);
}

TEST(LinearCostEvaluator, Convert) {
    std::vector<Eigen::Index> indices = {0};
    auto e = std::make_shared<LinearCostEvaluator>();
    auto cost = std::make_shared<bopt::CostTpl<LinearCostEvaluator>>(e);
    auto data = std::make_shared<bopt::LinearEvaluatorDataTpl<double, 1>>(*e);

    bopt::Binding<bopt::CostTpl<LinearCostEvaluator>> b1(cost, data, indices);
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