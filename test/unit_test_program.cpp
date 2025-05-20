#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/Logging.hpp"
#include "bopt/Profiler.hpp"
#include "bopt/Program.hpp"
#include "example_costs.hpp"

TEST(Program, AddCosts) {
    bopt::MathematicalProgram p("program");
    bopt::VariableVector x(1);
    x << p.addVariable("x");

    auto e0 = std::make_shared<CostEvaluator>();
    auto c0 = std::make_shared<bopt::DenseCost>(e0);
    auto d0 = c0->createData();
    p.addCost(c0, d0, x);

    // auto c1 = std::make_shared<BasicSparseCost>();
    // auto d1 = c1->createData();
    // p.addCost(c1, d1, x);

    // auto c2 = std::make_shared<LinearCost>();
    // auto d2 = c2->createData();
    // p.addLinearCost(c2, d2, x);

    // auto c = p.getCosts<bopt::SparseCostTpl<double>>();
}

int main(int argc, char **argv) {
    FLAGS_logtostderr = true;
    FLAGS_v = 10;

    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    testing::InitGoogleTest(&argc, argv);
    int status = RUN_ALL_TESTS();
    // bopt::profiler summary;
    return status;
}