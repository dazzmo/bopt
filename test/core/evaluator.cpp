#define BOPT_USE_PROFILING
#include "bopt/evaluator.hpp"

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <list>

#include "bopt/logging.hpp"
#include "bopt/profiler.hpp"

class TestEvaluator : public bopt::evaluator<double, std::size_t, std::vector> {
   public:
    typedef typename bopt::evaluator<double, std::size_t, std::vector> Base;

    index_type operator()(const value_type **arg, value_type *res) override {
        return index_type(0);
    }

    index_type info(bopt::evaluator_out_info<TestEvaluator> &info) {
        info.m = 1;
        info.n = 1;
        info.nnz = 1;

        info.values = buffer.data();
        return index_type(0);
    }
};

TEST(Evaluator, Initialise) {
    bopt::profiler test("Initialise");
    TestEvaluator eval;
    bopt::evaluator_out_info<TestEvaluator> info;
    eval.info(info);

    EXPECT_EQ(info.m, 1);
    EXPECT_EQ(info.n, 1);
    EXPECT_EQ(info.nnz, 1);
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