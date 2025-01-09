#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/costs.hpp"
#include "bopt/logging.hpp"
#include "bopt/profiler.hpp"

class BasicCost : public bopt::cost_tpl<double> {
   public:
    BasicCost() : bopt::cost_tpl<double>(2) {}

   protected:
    bopt::evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x, double &out) override {
        out = x.sum();
        return bopt::evaluator::return_status::Success;
    }

    bopt::evaluator::return_status eval_gradient_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_vector_t> out) override {
        out.setOnes();
        return bopt::evaluator::return_status::Success;
    }
};

class LinearCost : public bopt::linear_cost_tpl<double> {
   public:
    LinearCost() : bopt::linear_cost_tpl<double>(3) {}

   protected:
    bopt::evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x, double &out) override {
        out = x.sum();
        return bopt::evaluator::return_status::Success;
    }

    bopt::evaluator::return_status eval_gradient_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_vector_t> out) override {
        out.setOnes();
        return bopt::evaluator::return_status::Success;
    }
};

TEST(Expression, ScalarExpression) {
    LOG(INFO) << "BasicCost";
    std::shared_ptr<bopt::cost_tpl<double>> c = std::make_shared<BasicCost>();
    LOG(INFO) << c->buffer_gradient().dense.transpose();
    LOG(INFO) << c->buffer_hessian().dense;
    
    LOG(INFO) << "LinearCost";
    std::shared_ptr<bopt::cost_tpl<double>> i = std::make_shared<LinearCost>();
    LOG(INFO) << i->buffer_gradient().dense.transpose();
    LOG(INFO) << i->buffer_hessian().dense;
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