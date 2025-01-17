#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/expression.hpp"
#include "bopt/logging.hpp"
#include "bopt/profiler.hpp"

class ScalarExpression1 : public bopt::expression_scalar_tpl<double> {
   public:
    bopt::bopt_index cols_gradient() const override { return 2; }
    bopt::bopt_index rows_hessian() const override { return 2; }
    bopt::bopt_index cols_hessian() const override { return 2; }

   protected:
    bopt::evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x, double &out) override {
        out = x.squaredNorm();
        return bopt::evaluator::return_status::Success;
    }

    bopt::evaluator::return_status eval_gradient_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_vector_t> out) override {
        out = 2.0 * x;
        return bopt::evaluator::return_status::Success;
    }
};

TEST(Expression, ScalarExpression) {
    std::shared_ptr<bopt::expression_scalar_tpl<double>> c =
        std::make_shared<ScalarExpression1>();

    LOG(INFO) << "p: " << c->parameters().transpose();

    Eigen::VectorXd x(2);
    x.setRandom();

    double out;
    Eigen::VectorXd grd(2);
    Eigen::SparseVector<double> grd_sparse;

    c->eval(x, out);
    c->eval_gradient(x, grd);
    LOG(INFO) << "x: " << x.transpose() << " out: " << out;
    LOG(INFO) << "x: " << x.transpose() << " grd: " << grd;
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