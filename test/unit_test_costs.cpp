#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/costs.hpp"
#include "bopt/logging.hpp"
#include "bopt/profiler.hpp"

class IdentityCost : public bopt::linear_cost_tpl<double> {
   public:
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

class NormSquaredCost : public bopt::cost_tpl<double> {
   public:
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
    std::shared_ptr<bopt::cost_tpl<double>> c =
        std::make_shared<NormSquaredCost>();

    std::shared_ptr<bopt::linear_cost_tpl<double>> ls =
        std::make_shared<IdentityCost>();

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

TEST(Expression, LeastSquares) {
    std::shared_ptr<bopt::linear_cost_tpl<double>> c =
        std::make_shared<IdentityCost>();

    std::shared_ptr<bopt::least_squares_cost_tpl<double>> ls =
        std::make_shared<bopt::least_squares_cost_tpl<double>>(c);

    bopt::bopt_index n = 10;

    Eigen::VectorXd x(n);
    x.setRandom();

    double out;
    Eigen::VectorXd grd(n);

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