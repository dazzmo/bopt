#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/ad/casadi.hpp"

using sym = ::casadi::SX;
using dm = ::casadi::DM;

TEST(Casadi, Evaluator) {
    std::size_t n = 10;
    sym x = sym::sym("x", n);
    sym p = sym::sym("p", n);
    // Create symbolic constraint
    sym ex = x;
    for (int i = 0; i < n; ++i) {
        ex(i) = sin(p(i)) * x(i);
    }

    auto e = bopt::casadi::DenseEvaluatorTpl<double>(ex, x, p, true);
    auto d = e.createData();
}

TEST(Casadi, Cost) {
    std::size_t n = 2;
    sym x = sym::sym("x", n);
    sym p = sym::sym("p", n);
    // Create symbolic constraint
    sym ex = 0.0;
    for (int i = 0; i < n; ++i) {
        ex += sin(p(i)) * x(i);
    }

    std::cout << ex << std::endl;

    auto e = bopt::casadi::DenseCost(ex, x, p, true);
    auto d = e.createData();

    EXPECT_EQ(d->Hxx.rows(), n);
    EXPECT_EQ(d->Hxx.cols(), n);
}

TEST(Casadi, Constraint) {
    std::size_t n = 10;
    sym x = sym::sym("x", n);
    sym p = sym::sym("p", n);
    // Create symbolic constraint
    sym ex = x;
    for (int i = 0; i < n; ++i) {
        ex(i) = sin(p(i)) * x(i);
    }

    sym lb = -10 * sym::ones(n);
    sym ub = 10 * sym::ones(n);

    auto c = bopt::SparseConstraint(
        std::make_shared<bopt::casadi::SparseEvaluator>(ex, x, p, false),
        0.0, 1.0);
    auto d = c.createData();

    auto c1 = bopt::casadi::SparseConstraint(ex, x, p, lb, ub, false);
    auto d1 = c1.createData();

    c.evalBounds(*d);

    std::cout << d->lb << std::endl;
    std::cout << d->ub << std::endl;

    c.setBounds(bopt::ConstraintBounds::NEGATIVE);
    c.evalBounds(*d);

    std::cout << d->lb << std::endl;
    std::cout << d->ub << std::endl;

    c.setBounds(-1.0, 0.0);
    c.evalBounds(*d);

    std::cout << d->lb << std::endl;
    std::cout << d->ub << std::endl;
}

// TEST(Casadi, Cost) {
//     // Create variable vector and parameters
//     std::size_t n = 3;
//     sym xs = sym::sym("x", n);
//     sym ps = sym::sym("p", n);

//     // Create some costs
//     sym f0 = sym::dot(xs, xs);
//     sym f1 = sym::dot(xs, ps);

//     std::shared_ptr<bopt::Cost> c0 =
//         std::make_shared<bopt::casadi::Cost>(f0, xs, ps, true);
//     std::shared_ptr<bopt::Cost> c1 =
//         std::make_shared<bopt::casadi::Cost>(f1, xs, ps, true);

//     bopt::VectorXd x(n), p(n);
//     x.setRandom();
//     p.setRandom();

//     bopt::CostData data0(*c0), data1(*c1);
//     c0->setParameters(p);
//     c1->setParameters(p);

//     c0->eval(x, data0);
//     c1->eval(x, data1);

//     EXPECT_DOUBLE_EQ(data0.f, x.dot(x));
//     EXPECT_DOUBLE_EQ(data1.f, x.dot(p));

//     // Create linear cost
//     auto l1 = std::make_shared<bopt::casadi::LinearCost>(f1, xs, ps, false);
//     auto dl1 = bopt::LinearCostData(*l1);

//     std::shared_ptr<bopt::Cost> cl1 = l1;
//     auto dcl1 = bopt::CostData(*cl1);

//     l1->setParameters(p);
//     l1->eval(x, dl1);
//     cl1->eval(x, dcl1);

//     EXPECT_DOUBLE_EQ(dcl1.f, dl1.f);
// }

// TEST(Casadi, MathematicalProgram) {
//     // Create variable vector and parameters
//     std::size_t n = 10;
//     sym x = sym::sym("x", n);
//     sym p = sym::sym("p", n);

//     // Create some costs
//     sym f0 = sym::dot(x, x);  // + 2.0 * sym::sum1(x);
//     sym f1 = sym::dot(x, p) + 10;
//     // Create linear constraints
//     sym c0 = sym::zeros(2);
//     c0(0) = 1.0 * x(2) - 5.0 * x(7);
//     c0(1) = 2.0 * x(1) - 12.0 * x(3);

//     std::shared_ptr<bopt::QuadraticCost> cost0 =
//         std::make_shared<bopt::casadi::QuadraticCost>(f0, x, p, false);
//     std::shared_ptr<bopt::LinearCost> cost1 =
//         std::make_shared<bopt::casadi::LinearCost>(f1, x, p, false);

//     std::shared_ptr<bopt::LinearConstraint> constraint0 =
//         std::make_shared<bopt::casadi::LinearConstraint>(c0, x, p,
//         sym::ones(2),
//                                                          sym::ones(2),
//                                                          false);

//     auto bb = bopt::BoundingBoxConstraint::create(2, 0.0, 10.0);

//     cost0->setParameters(Eigen::VectorXd::Ones(n));
//     cost1->setParameters(Eigen::VectorXd::Ones(n));
//     constraint0->setParameters(Eigen::VectorXd::Ones(n));

//     auto program = bopt::MathematicalProgram("program");
//     bopt::VariableVector v(n);
//     for (int i = 0; i < n; ++i) {
//         v[i] = program.addVariable("x");
//     }

//     program.addQuadraticCost(cost0, v);
//     program.addLinearCost(cost1, v);
//     program.addLinearConstraint(constraint0, v);
//     program.addBoundingBoxConstraint(bb, v({0, 1}));

//     auto qp = bopt::solvers::qpoases_solver(program);
//     qp.options().printLevel = qpOASES::PrintLevel::PL_LOW;
//     qp.options().nWSR = 100;
//     qp.options().perform_hotstart = false;

//     LOG(INFO) << program;

//     for (int i = 0; i < 2; ++i) {
//         qp.solve(program);
//     }

//     // Try IPOPT

//     // auto nlp = bopt::solvers::ipopt_solver(program);

//     // for (int i = 0; i < 1; ++i) {
//     //     nlp.solve();
//     // }
// }

// #endif  // BOPT_WITH_CASADI

int main(int argc, char **argv) {
    // FLAGS_logtostderr = true;
    // FLAGS_v = 10;

    // google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    testing::InitGoogleTest(&argc, argv);
    int status = RUN_ALL_TESTS();
    // bopt::profiler summary;
    return status;
}
