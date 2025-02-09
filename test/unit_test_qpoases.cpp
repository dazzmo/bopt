#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/logging.hpp"
#include "bopt/profiler.hpp"
#include "bopt/program.hpp"
#include "bopt/solvers/qpoases.hpp"

class GenericQuadraticCost : public bopt::QuadraticCost {
    using MatrixXd = bopt::MatrixXd;
    using VectorXd = bopt::VectorXd;
    using SparseMatrix = bopt::SparseMatrix<double>;
    using SparseVector = bopt::SparseVector<double>;

   public:
    GenericQuadraticCost() : bopt::QuadraticCost(2) {}

   protected:
    void evalImpl(const Eigen::Ref<const VectorXd> &x,
                  bopt::CostData &data) const override {
        data.f = x.squaredNorm();
    }

    void setGradientSparsityPatterns(CostData &data) const override {
        data.gx_s.insert(0);
        data.gx_s.insert(1);
    }

    void evalSparseGradientsImpl(const Eigen::Ref<const VectorXd> &x,
                                 CostData &data, bool compute_x,
                                 bool compute_p) const override {
        if (compute_x) {
            data.gx_s.valuePtr()[0] = 2.0;
            data.gx_s.valuePtr()[1] = 2.0;
        }
    }

    void setHessianSparsityPatterns(CostData &data) const override {
        data.Hxx_s.insert(0, 0);
        data.Hxx_s.insert(1, 1);
        data.Hxx_s.makeCompressed();
    }

    void evalSparseHessiansImpl(const Eigen::Ref<const VectorXd> &x,
                                CostData &data, bool compute_xx,
                                bool compute_xp,
                                bool compute_pp) const override {
        if (compute_xx) {
            data.Hxx_s.valuePtr()[0] = 2.0;
            data.Hxx_s.valuePtr()[1] = 2.0;
        }
    }
};

class GenericLinearConstraint : public bopt::LinearConstraint {
    using MatrixXd = bopt::MatrixXd;
    using VectorXd = bopt::VectorXd;
    using SparseMatrix = bopt::SparseMatrix<double>;

   public:
    GenericLinearConstraint() : bopt::LinearConstraint(2, 2) {
        this->setName("linear_constraint");
        this->setType(bopt::ConstraintType::Inequality);
    }

   protected:
    void evalImpl(const Eigen::Ref<const VectorXd> &x,
                  EvaluatorData &data) const override {
        data.y[0] = x[1];
        data.y[1] = x[0] + x[1];
    }

    void setCoefficientSparsityPatternsImpl(
        bopt::LinearConstraintData &data) const override {
        std::vector<Eigen::Triplet<double>> triplets;
        triplets.push_back(Eigen::Triplet<double>(0, 0));
        triplets.push_back(Eigen::Triplet<double>(1, 0));
        triplets.push_back(Eigen::Triplet<double>(1, 1));
        data.A_s.setFromTriplets(triplets.begin(), triplets.end());
    }

    void evalSparseCoefficientsImpl(
        bopt::LinearConstraintData &data) const override {
        VLOG(10) << "evalSparseCoefficientMatrixImpl";
        data.A_s.valuePtr()[0] = 1;
        data.A_s.valuePtr()[1] = 1;
        data.A_s.valuePtr()[2] = 1;
    }

    void evalBoundsImpl(bopt::ConstraintData &data) const override {
        data.lb << -5.0, 2.0;
        data.ub << 5.0, 10.0;
    }
};

TEST(Program, SimpleProgram) {
    auto c = std::make_shared<GenericQuadraticCost>();
    auto g0 = std::make_shared<GenericLinearConstraint>();

    bopt::MathematicalProgram p("program");
    auto x = p.addVariable("x", 0.0);
    auto y = p.addVariable("y", 0.0);
    auto z = p.addVariable("z", 0.0);

    bopt::VariableVector v(3);
    v << x, y, z;

    p.addQuadraticCost(c, v({0, 2}));
    p.addLinearConstraint(g0, v({0, 2}));

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
    FLAGS_v = 10;

    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    testing::InitGoogleTest(&argc, argv);
    int status = RUN_ALL_TESTS();
    bopt::profiler summary;
    return status;
}