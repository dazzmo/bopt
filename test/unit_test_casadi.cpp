#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "bopt/logging.hpp"
#include "bopt/profiler.hpp"

#ifdef BOPT_WITH_CASADI

#include <Eigen/Core>

#include "bopt/ad/casadi.hpp"
#include "bopt/constraints.hpp"

using sym = ::casadi::SX;
using dm = ::casadi::DM;

// TEST(Casadi, Codegen) {
//     std::size_t n = 5;
//     sym x = sym::sym("x", n);
//     // Create symbolic constraint
//     sym ex = sym::dot(x, x);
//     ex += sin(dot(x, x));

//     auto fun = ::casadi::Function("f", {x}, {ex});

//     auto fun_cg = bopt::casadi::codegen(fun);

//     Eigen::VectorXd y(5);
//     y.setRandom();

//     double ret;

//     std::vector<const double *> in = {y.data()};
//     std::vector<double *> out = {&ret};

//     fun_cg(in, out);

//     EXPECT_DOUBLE_EQ(ret, y.squaredNorm() + sin(y.squaredNorm()));
// }

TEST(Casadi, ScalarEvaluator) {
    std::size_t n = 10;
    sym x = sym::sym("x", n);
    sym p = sym::sym("p", n);
    // Create symbolic constraint
    sym ex = sym::dot(x, p) + sin(dot(x, x));

    auto expr =
        std::make_shared<bopt::casadi::scalar_evaluator>(ex, x, p, true);

    EXPECT_EQ(expr->sz_in(), n);
    EXPECT_EQ(expr->sz_out().first, 1);
    EXPECT_EQ(expr->sz_out().second, 1);

    EXPECT_EQ(expr->parameters().size(), n);

    Eigen::VectorXd xv(10), pv(10);
    double out;
    xv.setRandom();
    pv.setRandom();

    double val = xv.dot(pv) + sin(xv.dot(xv));

    expr->parameters() << pv;

    for (int i = 0; i < 1000; ++i) {
        bopt::profiler("casadi_evaluator_no_cg");
        expr->eval(xv, out);
    }

    auto cpy = bopt::scalar_evaluator(expr);

    EXPECT_EQ(cpy.sz_in(), n);
    EXPECT_EQ(cpy.sz_out().first, 1);
    EXPECT_EQ(cpy.sz_out().second, 1);

    EXPECT_EQ(cpy.parameters().size(), n);

    for (int i = 0; i < 1000; ++i) {
        bopt::profiler("casadi_evaluator_cg");
        cpy.eval(xv, out);
    }

    EXPECT_DOUBLE_EQ(out, val);
}

TEST(Casadi, VectorEvaluator) {
    std::size_t n = 10;
    sym x = sym::sym("x", n);
    sym p = sym::sym("p", n);
    // Create symbolic constraint
    sym ex = x;
    for (int i = 0; i < n; ++i) {
        ex(i) = sin(p(i)) * x(i);
    }

    auto expr =
        std::make_shared<bopt::casadi::vector_evaluator>(ex, x, p, false);

    EXPECT_EQ(expr->sz_in(), n);
    EXPECT_EQ(expr->sz_out().first, n);
    EXPECT_EQ(expr->sz_out().second, 1);

    EXPECT_EQ(expr->parameters().size(), n);

    Eigen::VectorXd xv(10), pv(10);
    double out;
    xv.setRandom();
    pv.setRandom();

    auto cpy = bopt::vector_evaluator(expr);

    EXPECT_EQ(cpy.sz_in(), n);
    EXPECT_EQ(cpy.sz_out().first, n);
    EXPECT_EQ(cpy.sz_out().second, 1);

    EXPECT_EQ(cpy.parameters().size(), n);
}

TEST(Casadi, DifferentiableScalarEvaluator) {
    sym x = sym::sym("x", 2);
    sym p = sym::sym("p", 1);
    // Create symbolic expression
    sym ex = p * sym::dot(x, x);

    double out;

    Eigen::Vector2d xv, grd;
    Eigen::VectorXd lv(1);
    Eigen::Matrix2d hes;

    xv.setOnes();
    lv << 1.0;

    // Map to bopt
    auto cpy = bopt::differentiable_scalar_evaluator(
        std::make_shared<bopt::casadi::differentiable_scalar_evaluator>(
            ex, x, p, true, false));

    cpy.parameters().setConstant(1.0);
    cpy.eval(xv, out);
    cpy.eval_gradient(xv, grd);
    cpy.eval_hessian(xv, lv, hes);

    VLOG(10) << out;
    VLOG(10) << grd.transpose();
    VLOG(10) << hes;

    cpy.parameters().setConstant(5.0);
    for (int i = 0; i < 1000; ++i) {
        {
            bopt::profiler("test eval");
            cpy.eval(xv, out);
        }
        {
            bopt::profiler("test grd");
            cpy.eval_gradient(xv, grd);
        }
        {
            bopt::profiler("test hes");
            cpy.eval_hessian(xv, lv, hes);
        }
    }

    VLOG(10) << out;
    VLOG(10) << grd.transpose();
    VLOG(10) << hes;

    EXPECT_EQ(cpy.sz_gradient().first, 1);
    EXPECT_EQ(cpy.sz_gradient().second, 2);

    EXPECT_EQ(cpy.sz_hessian().first, 2);
    EXPECT_EQ(cpy.sz_hessian().second, 2);
}

TEST(Casadi, DifferentiableVectorEvaluator) {
    sym x = sym::sym("x", 10);
    sym p = sym::sym("p", 2);
    // Create symbolic expression
    sym ex = x;
    ex(2) = p(0) * x(2);
    ex(7) = p(1) * x(7);

    // Map to bopt
    auto cpy = bopt::differentiable_vector_evaluator(
        std::make_shared<bopt::casadi::differentiable_vector_evaluator>(
            ex, x, p, true, false));

    Eigen::VectorXd out(cpy.sz_out().first);
    Eigen::VectorXd xv(cpy.sz_in());
    Eigen::VectorXd lv(cpy.sz_out().first);
    Eigen::MatrixXd jac(cpy.sz_jacobian().first, cpy.sz_jacobian().second);
    Eigen::MatrixXd hes(cpy.sz_hessian().first, cpy.sz_hessian().second);

    xv.setRandom();
    lv.setRandom();
    cpy.parameters().setRandom();

    cpy.eval(xv, out);
    cpy.eval_jacobian(xv, jac);
    cpy.eval_hessian(xv, lv, hes);

    VLOG(10) << out;
    VLOG(10) << jac.transpose();
    VLOG(10) << hes;

    // cpy.parameters().setConstant(5.0);
    // for (int i = 0; i < 1000; ++i) {
    //     {
    //         bopt::profiler("test eval");
    //         cpy.eval(xv, out);
    //     }
    //     {
    //         bopt::profiler("test grd");
    //         cpy.eval_gradient(xv, grd);
    //     }
    //     {
    //         bopt::profiler("test hes");
    //         cpy.eval_hessian(xv, lv, hes);
    //     }
    // }

    // VLOG(10) << out;
    // VLOG(10) << grd.transpose();
    // VLOG(10) << hes;

    // EXPECT_EQ(cpy.sz_gradient().first, 1);
    // EXPECT_EQ(cpy.sz_gradient().second, 2);

    // EXPECT_EQ(cpy.sz_hessian().first, 2);
    // EXPECT_EQ(cpy.sz_hessian().second, 2);
}

TEST(Casadi, LinearVectorEvaluator) {
    sym x = sym::sym("x", 10);
    sym p = sym::sym("p", 2);
    // Create symbolic expression
    sym ex = x;
    ex(2) = p(0) * x(2);
    ex(7) = p(1) * x(7);
    ex(9) = 10.0;

    // Map to bopt
    auto cpy = bopt::linear_vector_evaluator(
        std::make_shared<bopt::casadi::linear_vector_evaluator>(ex, x, p, true,
                                                                false));

    Eigen::VectorXd out(cpy.sz_out().first);
    Eigen::VectorXd xv(cpy.sz_in());
    Eigen::MatrixXd A(cpy.sz_A().first, cpy.sz_A().second);
    Eigen::VectorXd b(cpy.sz_b().first);

    xv.setRandom();
    cpy.parameters().setRandom();

    cpy.eval(xv, out);
    cpy.eval_A(A);
    cpy.eval_b(b);

    VLOG(10) << out;
    VLOG(10) << A;
    VLOG(10) << b;

    // cpy.parameters().setConstant(5.0);
    // for (int i = 0; i < 1000; ++i) {
    //     {
    //         bopt::profiler("test eval");
    //         cpy.eval(xv, out);
    //     }
    //     {
    //         bopt::profiler("test grd");
    //         cpy.eval_gradient(xv, grd);
    //     }
    //     {
    //         bopt::profiler("test hes");
    //         cpy.eval_hessian(xv, lv, hes);
    //     }
    // }

    // VLOG(10) << out;
    // VLOG(10) << grd.transpose();
    // VLOG(10) << hes;

    // EXPECT_EQ(cpy.sz_gradient().first, 1);
    // EXPECT_EQ(cpy.sz_gradient().second, 2);

    // EXPECT_EQ(cpy.sz_hessian().first, 2);
    // EXPECT_EQ(cpy.sz_hessian().second, 2);
}

// TEST(Casadi, QuadraticExpression) {
//     sym x = sym::sym("x", 5);
//     sym p = sym::sym("p", 1);
//     // Create symbolic constraint
//     sym ex = p * sym::dot(x, x);

//     auto expr = bopt::casadi::quadratic_scalar_evaluator(ex, x, p, true,
//     false);

//     Eigen::MatrixXd A(expr.rows_A(), expr.cols_A());
//     Eigen::VectorXd pv(1);
//     pv << 1.0;
//     expr.set_parameters(pv);
//     expr.eval_A(A);

//     LOG(INFO) << "A: " << A;

//     expr = bopt::casadi::quadratic_scalar_evaluator(ex, x, p, false, false);

//     Eigen::SparseMatrix<double> As(expr.rows_A(), expr.cols_A());
//     expr.set_parameters(pv);
//     expr.sparsity_A(As);
//     expr.eval_A(As);

//     LOG(INFO) << "A: " << As;
// }

// TEST(Casadi, ExpressionWithParameter) {
//     std::size_t n = 10;
//     sym x = sym::sym("x", n);
//     sym p = sym::sym("p");

//     // Create symbolic expression
//     sym ex = p * sym::dot(x, x);

//     auto expr = bopt::casadi::vector_evaluator(ex, x, p, false);

//     Eigen::VectorXd in(10), out(1);
//     in.setRandom();

//     Eigen::VectorXd pv(1);
//     pv << 2.0;

//     expr.set_parameters(pv);
//     for (int i = 0; i < 1000; ++i) {
//         bopt::profiler("casadi_p_evaluator_no_cg");
//         expr.eval(in, out);
//     }

//     EXPECT_DOUBLE_EQ(out[0], pv[0] * in.squaredNorm());

//     expr = bopt::casadi::vector_evaluator(ex, x, p, true);
//     expr.set_parameters(pv);

//     for (int i = 0; i < 1000; ++i) {
//         bopt::profiler("casadi_p_evaluator_cg");
//         expr.eval(in, out);
//     }

//     EXPECT_DOUBLE_EQ(out[0], pv[0] * in.squaredNorm());
// }

// TEST(Casadi, Constraint) {
//     std::size_t n = 10;
//     sym x = sym::sym("x", n);
//     sym p = sym::sym("p");

//     // Create symbolic expression
//     sym ex = x(0) + p * x(3);

//     auto c = bopt::constraint(
//         std::make_shared<bopt::casadi::differentiable_vector_evaluator>(
//             ex, x, p, true, false),
//         bopt::bounds::type::Negative);

//     Eigen::VectorXd xv(10), pv(1), out(1);
//     Eigen::MatrixXd jacobian(1, 10);
//     xv.setRandom();
//     pv << 1.0;

//     c.set_parameters(pv);
//     VLOG(10) << "p: " << c.parameters().transpose();
//     VLOG(10) << "Status: " << (int)c.eval(xv, out);
//     c.eval_jacobian(xv, jacobian);

//     VLOG(10) << c;
//     VLOG(10) << jacobian;

//     EXPECT_DOUBLE_EQ(out[0], xv[0] + pv[0] * xv[3]);
// }

#endif  // BOPT_WITH_CASADI

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
