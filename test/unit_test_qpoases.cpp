#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/logging.hpp"
#include "bopt/profiler.hpp"
#include "bopt/program.hpp"
#include "bopt/solvers/qpoases.hpp"

class GenericQuadraticCost : public bopt::QuadraticCost {
   public:
    GenericQuadraticCost(const int n) : bopt::QuadraticCost(n) {
        this->setName("quadratic cost");

        bopt::EvaluatorBase::SparsityPattern A_pattern;
        for (int i = 0; i < n; ++i) {
            A_pattern.push_back({i, i});
        }
        setASparsityPattern(A_pattern);
        setANonZeroOnly(true);
    }

   protected:
    void evalImpl(const Eigen::Ref<const bopt::VectorXd> &x,
                  Eigen::Ref<bopt::VectorXd> out) override {
        out[0] = x.squaredNorm();
    }

    void evalAImpl(Eigen::Ref<bopt::MatrixXd> out) override {
        VLOG(10) << "evalAImpl";
        out.setOnes();
    }

    void evalbImpl(Eigen::Ref<bopt::VectorXd> out) override { out.setZero(); }
};

class GenericLinearConstraint : public bopt::LinearConstraint {
   public:
    GenericLinearConstraint() : bopt::LinearConstraint(2, 2) {
        this->setName("linear_constraint");
        setLowerBound(Eigen::Vector2d(-1.0, 2.0));
        setUpperBound(Eigen::Vector2d(3.0, 2.0));
        this->setType(bopt::Constraint::Type::Inequality);
    }

   protected:
    void evalImpl(const Eigen::Ref<const bopt::VectorXd> &x,
                  Eigen::Ref<bopt::VectorXd> out) override {
        out[0] = 2 * x[1];
        out[1] = x[0] - x[1];
    }

    void evalAImpl(Eigen::Ref<bopt::MatrixXd> out) override {
        out(0, 0) = 0.0;
        out(0, 1) = 2.0;
        out(1, 0) = 1.0;
        out(1, 1) = -1.0;
    }
};

TEST(Program, SimpleProgram) {
    auto c = std::make_shared<GenericQuadraticCost>(2);
    auto g0 = std::make_shared<GenericLinearConstraint>();

    bopt::MathematicalProgram p("program");
    auto x = p.addVariable("x", 0.0);
    auto y = p.addVariable("y", 0.0);
    auto z = p.addVariable("z", 0.0);

    bopt::variable_vector v(3);
    v << x, y, z;

    p.add_quadratic_cost(c, v({0, 2}));
    p.add_linear_constraint(g0, v({0, 2}));

    auto qp = bopt::solvers::qpoases_solver(p);
    qp.options().printLevel = qpOASES::PrintLevel::PL_LOW;
    qp.options().nWSR = 100;
    qp.options().perform_hotstart = false;

    for (int i = 0; i < 200; ++i) {
        qp.solve(p);
    }
}

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