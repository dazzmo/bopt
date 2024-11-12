#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <list>

#include "boost/numeric/ublas/io.hpp"
#include "bopt/constraints.hpp"
#include "bopt/logging.hpp"
#include "bopt/profiler.hpp"

TEST(Constraint, BoundingBox) {
    std::vector<double> ub = {1.0, 1.0}, lb = {-1.0, -1.0};

    // auto bb = bopt::BoundingBoxConstraint<double>();
    auto bb2 = bopt::BoundingBoxConstraint<double>(2, lb, ub);
    bb2.bounds({}, lb.data(), ub.data());
}

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    testing::InitGoogleTest(&argc, argv);
    FLAGS_logtostderr = 1;
    int status = RUN_ALL_TESTS();
    bopt::Profiler summary;
    return status;
}