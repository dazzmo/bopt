#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/ad/casadi.hpp"
#include "bopt/logging.hpp"
#include "bopt/profiler.hpp"
#include "bopt/program.hpp"
#include "bopt/solvers/ipopt.hpp"

class GenericQuadraticCost : public bopt::QuadraticCost {
    using MatrixXd = bopt::MatrixXd;
    using VectorXd = bopt::VectorXd;

   public:
    GenericQuadraticCost() : bopt::QuadraticCost(2) {
    }

   protected:
    void evalImpl(const Eigen::Ref<const VectorXd> &x,
                  Eigen::Ref<VectorXd> y) override {
        y << x.squaredNorm();
    }
};

class GenericLinearConstraint : public bopt::LinearConstraint {
    using MatrixXd = bopt::MatrixXd;
    using VectorXd = bopt::VectorXd;

   public:
    GenericLinearConstraint() : bopt::LinearConstraint(2, 2) {
        this->setName("linear_constraint");
        this->setType(bopt::Constraint::Type::Equality);

        setANonZeroOnly(true);
        SparsityPattern pattern = {};
        pattern.push_back({0, 0});
        pattern.push_back({1, 0});
        pattern.push_back({1, 0});
        setASparsityPattern(pattern);

        setLowerBound(Eigen::Vector2d(1.0, 2.0));
        setUpperBound(Eigen::Vector2d(1.0, 2.0));
    }

   protected:
    void evalImpl(const Eigen::Ref<const VectorXd> &x,
                  Eigen::Ref<VectorXd> out) override {
        out[0] = x[1];
        out[1] = x[0] + x[1];
    }

    void evalAImpl(Eigen::Ref<MatrixXd> out) override { out << 1.0, 1.0, 1.0; }
};

int main(int argc, char **argv) {
    FLAGS_logtostderr = true;
    // FLAGS_v = 10;

    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    testing::InitGoogleTest(&argc, argv);
    int status = RUN_ALL_TESTS();
    bopt::profiler summary;
    return status;
}