#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/ad/casadi.hpp"
#include "bopt/logging.hpp"
#include "bopt/profiler.hpp"
#include "bopt/program.hpp"
#include "bopt/solvers/ipopt.hpp"

TEST(Program, SimpleProgram) {
    using sym = ::casadi::SX;
    using sym_vec = ::casadi::SXVector;

    // Create variables

    bopt::MathematicalProgram p("program");
    auto x = p.addVariable("x", 0.0);
    auto y = p.addVariable("y", 0.0);
    auto z = p.addVariable("z", 0.0);

    bopt::VariableVector v(3);
    v << x, y, z;

    sym xs = sym::sym("x");
    sym ys = sym::sym("y");
    sym zs = sym::sym("z");

    sym ex = xs + ys - zs;

    auto c0 = std::make_shared<bopt::DenseConstraint>(
        std::make_shared<bopt::casadi::DenseEvaluator>(
            ex, sym::vertcat({xs, ys, zs}), sym(), false),
        1.0, 1.0);
    auto d0 = c0->createData();
    p.addConstraint(c0, d0, v);

    ex = ys * zs;
    auto c1 = std::make_shared<bopt::SparseConstraint>(
        std::make_shared<bopt::casadi::SparseEvaluator>(
            ex, sym::vertcat({xs, ys, zs}), sym(), false),
        bopt::ConstraintBounds::POSITIVE);
    auto d1 = c1->createData();
    p.addConstraint(c1, d1, v);

    ex = xs * ys * zs;
    auto f = std::make_shared<bopt::DenseCost>(
        std::make_shared<bopt::casadi::DenseCost>(
            ex, sym::vertcat({xs, ys, zs}), sym(), false));
    auto df = f->createData();
    p.addCost(f, df, v);

    auto nlp = bopt::solvers::ipopt_solver(p);
    nlp.options()->SetStringValue("hessian_approximation", "exact");

    for (int i = 0; i < 1; ++i) {
        nlp.solve();
    }
    std::cout << nlp.getPrimalSolution();
}

int main(int argc, char **argv) {
    FLAGS_logtostderr = true;
    FLAGS_v = 10;

    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    testing::InitGoogleTest(&argc, argv);
    int status = RUN_ALL_TESTS();
    bopt::profiler summary;
    return status;
}