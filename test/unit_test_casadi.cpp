#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "bopt/logging.hpp"
#include "bopt/profiler.hpp"

#ifdef BOPT_WITH_CASADI

#include <Eigen/Core>

#include "bopt/ad/casadi.hpp"
#include "bopt/ad/casadi/expressions/linear.hpp"

using sym = ::casadi::SX;
using dm = ::casadi::DM;

TEST(Casadi, Codegen) {
    std::size_t n = 5;
    sym x = sym::sym("x", n);
    // Create symbolic constraint
    sym ex = sym::dot(x, x);
    ex += sin(dot(x, x));

    auto fun = ::casadi::Function("f", {x}, {ex});

    auto fun_cg = bopt::casadi::codegen(fun);

    Eigen::VectorXd y(5);
    y.setRandom();

    double ret;

    std::vector<const double *> in = {y.data()};
    std::vector<double *> out = {&ret};

    fun_cg(in, out);

    EXPECT_DOUBLE_EQ(ret, y.squaredNorm() + sin(y.squaredNorm()));
}

TEST(Casadi, Expression) {
    std::size_t n = 10;
    sym x = sym::sym("x", n);
    // Create symbolic constraint
    sym ex = sym::dot(x, x) + sin(dot(x, x));

    auto expr = bopt::casadi::vector_expression(ex, x, sym(), false);

    Eigen::VectorXd in(10), out(1);
    in.setRandom();

    for (int i = 0; i < 1000; ++i) {
        bopt::profiler("casadi_expression_no_cg");
        expr.eval(in, out);
    }

    expr = bopt::casadi::vector_expression(ex, x, sym(), true);
    for (int i = 0; i < 1000; ++i) {
        bopt::profiler("casadi_expression_cg");
        expr.eval(in, out);
    }

    EXPECT_DOUBLE_EQ(out[0], in.squaredNorm() + sin(in.squaredNorm()));
}

TEST(Casadi, LinearExpression) {
    sym x = sym::sym("x", 1);
    // Create symbolic constraint
    sym ex = 2.0 * x + 1.0;

    auto expr = bopt::casadi::linear_scalar_expression(ex, x, sym(), false);

    Eigen::VectorXd in(1), out(1);
    in.setRandom();

    for (int i = 0; i < 1000; ++i) {
        bopt::profiler("casadi_linear_expression_no_cg");
        expr.eval_a(out);
        expr.eval_b(out[0]);
    }

    expr = bopt::casadi::linear_scalar_expression(ex, x, sym(), true);
    for (int i = 0; i < 1000; ++i) {
        bopt::profiler("casadi_linear_expression_cg");
        expr.eval_a(out);
        expr.eval_b(out[0]);
    }
}

TEST(Casadi, QuadraticExpression) {
    sym x = sym::sym("x", 5);
    sym p = sym::sym("p", 1);
    // Create symbolic constraint
    sym ex = p * sym::dot(x, x);

    auto expr =
        bopt::casadi::quadratic_scalar_expression(ex, x, p, true, false);

    Eigen::MatrixXd A(expr.rows_A(), expr.cols_A());
    Eigen::VectorXd pv(1);
    pv << 1.0;
    expr.set_parameters(pv);
    expr.eval_A(A);

    LOG(INFO) << "A: " << A;

    expr = bopt::casadi::quadratic_scalar_expression(ex, x, p, false, false);

    Eigen::SparseMatrix<double> As(expr.rows_A(), expr.cols_A());
    expr.set_parameters(pv);
    expr.sparsity_A(As);
    expr.eval_A(As);

    LOG(INFO) << "A: " << As;
}

TEST(Casadi, ExpressionWithParameter) {
    std::size_t n = 10;
    sym x = sym::sym("x", n);
    sym p = sym::sym("p");

    // Create symbolic expression
    sym ex = p * sym::dot(x, x);

    auto expr = bopt::casadi::vector_expression(ex, x, p, false);

    Eigen::VectorXd in(10), out(1);
    in.setRandom();

    Eigen::VectorXd pv(1);
    pv << 2.0;

    expr.set_parameters(pv);
    for (int i = 0; i < 1000; ++i) {
        bopt::profiler("casadi_p_expression_no_cg");
        expr.eval(in, out);
    }

    EXPECT_DOUBLE_EQ(out[0], pv[0] * in.squaredNorm());

    expr = bopt::casadi::vector_expression(ex, x, p, true);
    expr.set_parameters(pv);

    for (int i = 0; i < 1000; ++i) {
        bopt::profiler("casadi_p_expression_cg");
        expr.eval(in, out);
    }

    EXPECT_DOUBLE_EQ(out[0], pv[0] * in.squaredNorm());
}

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
