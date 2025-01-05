#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/expression.hpp"
#include "bopt/logging.hpp"
#include "bopt/profiler.hpp"

class ScalarExpression1 : public bopt::expression_scalar_tpl<double> {
   public:
   protected:
    bopt::evaluator::return_status eval_impl(const dense_vector_t &x,
                                             double &out) override {
        out = x.squaredNorm();
        return bopt::evaluator::return_status::Success;
    }
};

TEST(Expression, ScalarExpression) {}

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    testing::InitGoogleTest(&argc, argv);
    int status = RUN_ALL_TESTS();
    bopt::profiler summary;
    return status;
}