#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/Logging.hpp"
#include "bopt/Profiler.hpp"
#include "bopt/Program.hpp"
#include "example_constraints.hpp"
#include "example_costs.hpp"

TEST(Program, AddCosts) {
    bopt::MathematicalProgram p("program");
    bopt::VariableVector x(2);
    x << p.addVariables("x", 2);

    auto f0 = std::make_shared<SumOfSquares>(2);
    auto fl0 = std::make_shared<LinearCost>(2);
    p.addCost(f0, x);
    p.addLinearCost(fl0, x);

    auto c0 = std::make_shared<SumOfSquaresConstraint>(2);
    p.addConstraint(c0, x);

    const auto lbx = Eigen::Vector2d(-2.0, -2.0);
    const auto ubx = Eigen::Vector2d(14.0, 12.0);

    p.addBoundingBoxConstraint(x, lbx, ubx);
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