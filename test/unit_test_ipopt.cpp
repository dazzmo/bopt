#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/Bopt.hpp"
#include "bopt/ad/Casadi.hpp"
#include "bopt/solvers/Ipopt.hpp"

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
        ys * zs, sym::vertcat({xs, ys, zs}), sym(),
        bopt::ConstraintBounds::POSITIVE, false);
    auto d1 = c1->createData();
    c1->evalBounds(*d1);
    std::cout << d1->lb << std::endl;
    std::cout << d1->ub << std::endl;

    Eigen::Vector3d xx;
    xx.setOnes();
    c1->eval(xx, *d1);
    c1->evalJacobians(xx, *d1);
    std::cout << d1->y << std::endl;
    std::cout << d1->Jx << std::endl;

    p.addConstraint<bopt::DenseConstraint>(c1, d1, v);

    // todo - solution changes with sparsity
    auto f = std::make_shared<bopt::DenseCost>(
        std::make_shared<bopt::casadi::DenseCost>(
            xs * ys + zs, sym::vertcat({xs, ys, zs}), sym(), false));
    auto df = f->createData();
    p.addCost(f, df, v);

    p.addBoundingBoxConstraint(v, Eigen::Vector3d(0.0, 0.0, 0.0),
                               Eigen::Vector3d(1.0, 1.0, 1.0));

    auto nlp = bopt::solvers::ipopt_solver(p);
    nlp.options()->SetStringValue("hessian_approximation", "exact");

    for (int i = 0; i < 1; ++i) {
        nlp.solve();
    }
    std::cout << nlp.getPrimalSolution();
}

int main(int argc, char **argv) {
    FLAGS_logtostderr = true;
    // FLAGS_v = 10;

    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    testing::InitGoogleTest(&argc, argv);
    int status = RUN_ALL_TESTS();
    bopt::profiler summary;
    return status;
}