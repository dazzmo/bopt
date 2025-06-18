#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/Bopt.hpp"
#include "bopt/ad/Casadi.hpp"
#include "bopt/solvers/Clp.hpp"

TEST(Program, SimpleProgram) {
    // Create variables

    bopt::MathematicalProgram p("program");

    auto x = p.addVariables("x", 10);

    for (const auto &xi : x) {
        p.setVariableBounds(xi, -1.0, 1.0);
    }

    using SX = ::casadi::SX;

    SX xs = SX::sym("x", 10);

    auto c0 = std::make_shared<
        bopt::casadi::LinearConstraint<double, bopt::SparsityType::SPARSE>>(
        xs(0) + 5 * xs(7) - 0.1 * xs(5) - 1e-6, xs, SX(),
        bopt::ConstraintBoundType::STRICTLY_POSITIVE);
    p.addLinearConstraint(c0, x);

    auto f = std::make_shared<
        bopt::casadi::LinearCost<double, bopt::SparsityType::DENSE>>(
        SX::sum1(xs) + 100, xs, SX());
    p.addLinearCost(f, x);

    auto lp = bopt::solvers::ClpSolver(p);
    lp.init();
    lp.solve();

    bopt::Logger::info() << lp.getResults().objective;
    bopt::Logger::info() << lp.getResults().primal;
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