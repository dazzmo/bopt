#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "bopt/constraints.hpp"
#include "bopt/logging.hpp"
#include "bopt/profiler.hpp"

#ifdef BOPT_WITH_CASADI

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

    auto expr = std::make_shared<bopt::casadi::Evaluator>(ex, x, p, true);

    EXPECT_EQ(expr->dim_input(), n);
    EXPECT_EQ(expr->dim_output(), n);

    EXPECT_EQ(expr->parameters().size(), n);

    auto data = bopt::EvaluatorData(*expr);
    Eigen::VectorXd v(n), z(n);
    v.setRandom();
    z.setRandom();
    expr->setParameters(z);
    {
        bopt::profiler profiler("eval");
        expr->eval(v, data);
    }
    {
        bopt::profiler profiler("evalJacobians");
        expr->evalJacobians(v, data, true, true);
    }
    {
        bopt::profiler profiler("evalSparseJacobians");
        expr->evalSparseJacobians(v, data, true, true);
    }

    VLOG(10) << data.y;
    VLOG(10) << data.Jx;
    VLOG(10) << data.Jp;

    VLOG(10) << data.Jx_s;
    VLOG(10) << data.Jp_s;
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
