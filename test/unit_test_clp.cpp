#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/logging.hpp"
#include "bopt/program.hpp"
#include "bopt/solvers/Clp.hpp"

class LinearCost : public bopt::DenseLinearCostTpl<double> {
   public:
    LinearCost() : bopt::DenseLinearCostTpl<double>(5) {}
    using Base = bopt::DenseLinearCostTpl<double>;
    using EvaluatorData = typename Base::EvaluatorData;
    using Data = typename Base::Data;

   protected:
    void evalImpl(const InputVectorConstRef &x,
                  EvaluatorData &data) const override {
        data.y = x[0] + 5.0 * x[3] - x[2];
    }

    void evalCoefficientsImpl(Data &data) const override {
        data.a << 1.0, 0.0, -1.0, 5.0, 0.0;
    }
};

class LinearConstraint : public bopt::DenseLinearConstraintTpl<double> {
   public:
    LinearConstraint() : bopt::DenseLinearConstraintTpl<double>(5, 1) {}
    using Base = bopt::DenseLinearConstraintTpl<double>;
    using EvaluatorData = typename Base::EvaluatorData;
    using ConstraintData = typename Base::ConstraintData;
    using Data = typename Base::Data;

   protected:
    void evalImpl(const InputVectorConstRef &x,
                  EvaluatorData &data) const override {
        data.y << x[0] + x[1] + x[2] + x[3] + x[4];
    }

    void evalCoefficientsImpl(Data &data) const override { data.A.setOnes(); }

    void evalBoundsImpl(ConstraintData &data) const override {
        data.lb.setConstant(5.0);
        data.ub.setConstant(5.0);
    }
};

TEST(Program, SimpleProgram) {
    bopt::MathematicalProgram p("program");
    bopt::VariableVector x = p.addVariables("x", 5);
    
    auto c0 = std::make_shared<LinearCost>();
    auto d0 = c0->createData();
    p.addLinearCost(c0, d0, x);

    auto c1 = std::make_shared<LinearConstraint>();
    auto d1 = c1->createData();
    p.addLinearConstraint(c1, d1, x);

    auto lp = bopt::solvers::ClpSolver(p);
    lp.solve(p);
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