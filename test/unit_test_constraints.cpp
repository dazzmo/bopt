#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/constraints.hpp"
#include "bopt/logging.hpp"
#include "bopt/profiler.hpp"

class GenericConstraint : public bopt::constraint_tpl<double> {
   public:
    GenericConstraint() : bopt::constraint_tpl<double>(2, 2) {}

   protected:
    bopt::evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_vector_t> out) override {
        out[0] = x[0];
        out[0] = x[0] - x[1];
        return bopt::evaluator::return_status::Success;
    }

    bopt::evaluator::return_status eval_jacobian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_matrix_t> out) override {
        out(0, 0) = 1.0;
        out(1, 0) = 1.0;
        out(1, 1) = -1.0;
        return bopt::evaluator::return_status::Success;
    }
};

TEST(Constraint, ScalarConstraint) {
    std::shared_ptr<bopt::constraint_tpl<double>> c =
        std::make_shared<GenericConstraint>();
        c->set_name("constraint!");

    LOG(INFO) << *c;
}

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