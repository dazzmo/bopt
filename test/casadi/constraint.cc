
#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/ad/casadi/casadi.hpp"
#include "bopt/dlib_handler.hpp"
#include "bopt/logging.hpp"
#include "bopt/profiler.hpp"

TEST(CasadiConstraint, Constraint) {
    typedef casadi::SX sym;

    std::size_t n = 5;
    sym x = sym::sym("x", n);
    // Create symbolic constraint
    sym ex = -3.0 * x(1) + 5.0 * x(2) + 2.0 * x(4);

    // Compute Jacobian
    sym jac = sym::jacobian(ex, x);

    std::shared_ptr<bopt::linear_constraint<double>> c =
        std::make_shared<bopt::casadi::linear_constraint<double>>(ex, x, sym());

    bopt::linear_constraint<double>::out_info_t info;
    c->jac_info(info);

    EXPECT_EQ(info.m, jac.size1());
    EXPECT_EQ(info.n, jac.size2());
    EXPECT_EQ(info.nnz, jac.nnz());
}

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    testing::InitGoogleTest(&argc, argv);
    FLAGS_logtostderr = 1;
    int status = RUN_ALL_TESTS();
    bopt::profiler summary;
    return status;
}