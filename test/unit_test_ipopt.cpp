#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/Bopt.hpp"
#include "bopt/ad/Casadi.hpp"
#include "bopt/solvers/Ipopt.hpp"

TEST(Program, SimpleProgram) {
    // Create variables

    bopt::MathematicalProgram p("program");
    auto x = p.addVariable("x", 0.0, 0.0, 1.0);
    auto y = p.addVariable("y", -5, -10, 10);
    auto z = p.addVariable("z", -5, -10, 10);

    // Add variables

    bopt::VariableVector v(3);
    v << x, y, z;

    using SX = ::casadi::SX;

    SX xs = SX::sym("x");
    SX ys = SX::sym("y");
    SX zs = SX::sym("z");

    auto c0 = std::make_shared<
        bopt::casadi::Constraint<double, bopt::SparsityType::DENSE>>(
        xs + ys - zs, SX::vertcat({xs, ys, zs}), SX(),
        bopt::ConstraintBoundType::ZERO);
    p.addConstraint(c0, v);

    auto c1 = std::make_shared<bopt::casadi::Constraint<double>>(
        ys * zs, SX::vertcat({xs, ys, zs}), SX(),
        bopt::ConstraintBoundType::POSITIVE);
    p.addConstraint(c1, v);

    // fixme - solution changes with sparsity
    auto f =
        std::make_shared<bopt::casadi::Cost<double, bopt::SparsityType::DENSE>>(
            xs * ys + zs, SX::vertcat({xs, ys, zs}), SX());
    p.addCost(f, v);

    auto nlp = bopt::solvers::IpoptSolver(p);
    nlp.init();
    nlp.options()->SetStringValue("hessian_approximation", "exact");

    nlp.solve();
}

int main(int argc, char **argv) {
    FLAGS_logtostderr = true;
    // FLAGS_v = 10;

    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    testing::InitGoogleTest(&argc, argv);
    int status = RUN_ALL_TESTS();
    bopt::Profiler summary;
    return status;
}