#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/Logging.hpp"
#include "bopt/ad/casadi.hpp"
#include "bopt/profiler.hpp"
#include "bopt/program.hpp"
#include "bopt/solvers/Clp.hpp"

TEST(Program, SimpleProgram) {
    using sym = ::casadi::SX;
    using sym_vec = ::casadi::SXVector;

    // Create variables

    bopt::MathematicalProgram p("program");
    auto x = p.addVariable("x", 0.0, 0.0, 1.0);
    auto y = p.addVariable("y", 0.0);
    auto z = p.addVariable("z", 0.0);

    // Add variables

    bopt::VariableVector v(3);
    v << x, y, z;

    sym xs = sym::sym("x");
    sym ys = sym::sym("y");
    sym zs = sym::sym("z");

    auto c0 = std::make_shared<bopt::casadi::DenseConstraint>(
        xs + ys - zs, sym::vertcat({xs, ys, zs}), sym(), 1.0, 1.0, false);
    auto d0 = c0->createData();
    p.addConstraint<bopt::DenseConstraint>(c0, d0, v);

    auto c1 = std::make_shared<bopt::casadi::DenseConstraint>(
        ys + zs, sym::vertcat({xs, ys, zs}), sym(),
        bopt::ConstraintBounds::POSITIVE, false);
    bopt::DenseConstraint::Data d(c);
    c1->evalBounds(*d1);

    p.addConstraint<bopt::DenseConstraint>(c1, d1, v);

    // todo - solution changes with sparsity
    auto f = std::make_shared<bopt::DenseLinearCost>(
        std::make_shared<bopt::casadi::DenseLinearCost>(
            xs + ys + zs, sym::vertcat({xs, ys, zs}), sym(), false));
    auto df = f->createData();
    p.addLinearCost(f, df, v);

    p.addBoundingBoxConstraint(v, Eigen::Vector3d(0.0, 0.0, 0.0),
                               Eigen::Vector3d(1.0, 1.0, 1.0));

    auto nlp = bopt::solvers::ClpSolver(p);
    for (int i = 0; i < 1; ++i) {
        nlp.solve(p);
    }
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