#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/Constraints.hpp"
#include "bopt/Logging.hpp"
#include "bopt/Profiler.hpp"

TEST(BoundingBox, DefaultConstructor) {
    auto con = bopt::BoundingBoxConstraintTpl<double>();
}

TEST(BoundingBox, BoundsConstructor) {
    Eigen::VectorXd lb(2), ub(2);
    lb.setConstant(-10.0);
    ub.setConstant(10.0);
    auto con = bopt::BoundingBoxConstraintTpl<double>(lb, ub);
    auto data = bopt::EvaluatorDataTpl<double>(con);
    Eigen::VectorXd x(2);
    x.setRandom();
    bopt::Logger::info() << "x " << x.transpose();
    con.eval(x, data);
    bopt::Logger::info() << "c " << data.y.transpose();
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