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
    GenericQuadraticCost() : bopt::QuadraticCost(2) {}

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

        SparsityPattern pattern = {};
        pattern.push_back({0, 0});
        pattern.push_back({1, 0});
        pattern.push_back({1, 1});
        setANonZeroOnly(true);
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

TEST(Program, SimpleProgram) {
    auto c = std::make_shared<GenericQuadraticCost>();
    auto g0 = std::make_shared<GenericLinearConstraint>();

    bopt::MathematicalProgram p("program");
    auto x = p.addVariable("x", 0.0);
    auto y = p.addVariable("y", 0.0);
    auto z = p.addVariable("z", 0.0);

    bopt::variable_vector v(3);
    v << x, y, z;

    p.add_quadratic_cost(c, v({0, 2}));
    p.add_linear_constraint(g0, v({0, 2}));

    auto nlp = bopt::solvers::ipopt_solver(p);
    nlp.options()->SetStringValue("hessian_approximation", "limited-memory");

    for (int i = 0; i < 200; ++i) {
        nlp.solve();
    }
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