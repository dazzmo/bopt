#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/logging.hpp"
#include "bopt/profiler.hpp"
#include "bopt/program.hpp"
#include "bopt/solvers/ipopt.hpp"

class GenericQuadraticCost : public bopt::quadratic_cost {
   public:
    GenericQuadraticCost() : bopt::quadratic_cost(2) {
        this->set_name("quadratic cost");
    }

   protected:
    bopt::evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x, double &out) override {
        out = x.squaredNorm();
        return bopt::evaluator::return_status::Success;
    }

    bopt::evaluator::return_status eval_A_impl(
        Eigen::Ref<dense_matrix_t> out) override {
        out.setIdentity();
        return bopt::evaluator::return_status::Success;
    }

    bopt::evaluator::return_status eval_b_impl(
        Eigen::Ref<dense_vector_t> out) override {
        out.setZero();
        return bopt::evaluator::return_status::Success;
    }
};

class GenericLinearConstraint : public bopt::linear_constraint {
   public:
    GenericLinearConstraint()
        : bopt::linear_constraint(2, 2, 0, bopt::bounds::type::Equality) {
        this->set_name("linear_constraint");
    }

    void sparsity_jacobian(sparse_matrix_t &jac) const override {
        jac.resize(rows_jacobian(), cols_jacobian());
        jac.coeffRef(0, 1) = 0.0;
        jac.coeffRef(1, 0) = 0.0;
        jac.coeffRef(1, 1) = 0.0;
    }

   protected:
    bopt::evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_vector_t> out) override {
        out[0] = x[1];
        out[0] = x[0] - x[1];
        return bopt::evaluator::return_status::Success;
    }

    bopt::evaluator::return_status eval_A_impl(
        Eigen::Ref<dense_matrix_t> out) override {
        out(0, 1) = 1.0;
        out(1, 0) = 1.0;
        out(1, 1) = -1.0;
        return bopt::evaluator::return_status::Success;
    }

    bopt::evaluator::return_status eval_b_impl(
        Eigen::Ref<dense_vector_t> out) override {
        out << 1.0, 2.0;
        return bopt::evaluator::return_status::Success;
    }
};

TEST(Program, SimpleProgram) {
    auto c = std::make_shared<GenericQuadraticCost>();
    auto g0 = std::make_shared<GenericLinearConstraint>();

    auto x = bopt::create_variable_vector("x", 2);

    bopt::mathematical_program<double> p("program");
    p.add_variables(x);

    p.add_cost(c, x);
    p.add_linear_constraint(g0, x);

    auto nlp = bopt::solvers::ipopt_solver(p);
    nlp.options()->SetNumericValue("tol", 1e-3);
    nlp.options()->SetStringValue("mu_strategy", "adaptive");
    nlp.solve();
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