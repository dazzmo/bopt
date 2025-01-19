#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/logging.hpp"
#include "bopt/profiler.hpp"
#include "bopt/program.hpp"

class GenericConstraint : public bopt::constraint {
   public:
    GenericConstraint() : bopt::constraint(2, 2, 0) {
        this->set_name("constraint");
    }

    void sparsity_jacobian(sparse_matrix_t &jac) const override {
        jac.resize(rows_jacobian(), cols_jacobian());
        jac.coeffRef(0, 1) = 0.0;
        jac.coeffRef(1, 0) = 0.0;
        jac.coeffRef(1, 1) = 0.0;
    }

    void sparsity_hessian(sparse_matrix_t &hes) const override {
        hes.resize(rows_hessian(), cols_hessian());
        hes.coeffRef(0, 0) = 0.0;
        hes.coeffRef(1, 0) = 0.0;
        hes.coeffRef(1, 1) = 0.0;
    }

   protected:
    bopt::evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_vector_t> out) override {
        out[0] = x[0] * x[0];
        out[1] = x[1] * x[1];
        return bopt::evaluator::return_status::Success;
    }

    bopt::evaluator::return_status eval_jacobian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_matrix_t> out) override {
        out(0, 0) = 1.0;
        out(1, 0) = 1.0;
        out(1, 1) = 1.0;
        return bopt::evaluator::return_status::Success;
    }

    bopt::evaluator::return_status eval_hessian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        const Eigen::Ref<const dense_vector_t> &lambda,
        sparse_matrix_t &out) override {
        out.coeffRef(0, 0) = lambda[0];
        out.coeffRef(1, 0) = lambda[1];
        out.coeffRef(1, 1) = lambda[1];
        return bopt::evaluator::return_status::Success;
    }
};

class GenericLinearConstraint : public bopt::linear_constraint {
   public:
    GenericLinearConstraint() : bopt::linear_constraint(2, 2, 0) {
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

    bopt::evaluator::return_status eval_jacobian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_matrix_t> out) override {
        out(0, 1) = 1.0;
        out(1, 0) = 1.0;
        out(1, 1) = -1.0;
        return bopt::evaluator::return_status::Success;
    }

    bopt::evaluator::return_status eval_jacobian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        sparse_matrix_t &out) override {
        out.coeffRef(0, 1) = 1.0;
        out.coeffRef(1, 0) = 1.0;
        out.coeffRef(1, 1) = -1.0;
        return bopt::evaluator::return_status::Success;
    }
};

TEST(Program, ConstraintJacobian) {
    auto c0 = std::make_shared<GenericConstraint>();
    auto c1 = std::make_shared<GenericLinearConstraint>();

    auto x = bopt::create_variable_vector("x", 5);
    auto b0 =
        bopt::binding<bopt::constraint>(c0, std::vector<Eigen::Index>({0, 1}));
    auto b1 =
        bopt::binding<bopt::constraint>(c0, std::vector<Eigen::Index>({1, 2}));
    auto b2 =
        bopt::binding<bopt::constraint>(c1, std::vector<Eigen::Index>({3, 4}));

    Eigen::SparseMatrix<double> jacobian;
    std::vector<bopt::binding<bopt::constraint>> bindings = {b0, b1, b2};
    bopt::get_constraint_jacobian(jacobian, x.size(), bindings);

    VLOG(10) << jacobian;
    Eigen::VectorXd values(5);
    values.setRandom();
    bopt::eval_constraint_jacobian(values, jacobian, bindings);

    VLOG(10) << jacobian;
}

TEST(Program, LagrangianHessian) {
    auto c0 = std::make_shared<GenericConstraint>();

    auto x = bopt::create_variable_vector("x", 5);
    auto b0 =
        bopt::binding<bopt::constraint>(c0, std::vector<Eigen::Index>({0, 1}));
    auto b1 =
        bopt::binding<bopt::constraint>(c0, std::vector<Eigen::Index>({1, 2}));
    auto b2 =
        bopt::binding<bopt::constraint>(c0, std::vector<Eigen::Index>({2, 4}));

    Eigen::SparseMatrix<double> hessian;
    std::vector<bopt::binding<bopt::constraint>> bindings = {b0, b1, b2};
    bopt::get_lagrangian_hessian(hessian, x.size(), {}, bindings);

    VLOG(10) << hessian;
    Eigen::VectorXd values(5);
    Eigen::VectorXd lambda(6);
    values.setRandom();
    lambda.setRandom();
    bopt::eval_lagrangian_hessian(values, lambda, hessian, {}, bindings);

    VLOG(10) << hessian;
}

TEST(Program, AddVariable) {
    bopt::mathematical_program<double> p("program");
    bopt::variable x("x"), y("y");
    p.add_variable(x);
    p.add_variable(x);

    EXPECT_EQ(p.variable_index(x), 0);
    // EXPECT_DEATH({p.variable_index(y);}, "");
}

TEST(Program, AddVariables) {
    bopt::mathematical_program<double> p("program");
    bopt::variable_vector x = bopt::create_variable_vector("x", 10);
    p.add_variables(x);

    EXPECT_EQ(p.variable_index(x[3]), 3);
    EXPECT_EQ(p.variable_index(x[9]), 9);

    VLOG(10) << p.variables_initial_value().transpose();
    VLOG(10) << p.variables_upper_bound().transpose();
    VLOG(10) << p.variables_lower_bound().transpose();
}

TEST(Program, AddConstraints) {
    bopt::mathematical_program<double> p("program");
    bopt::variable_vector x = bopt::create_variable_vector("x", 10);
    p.add_variables(x);

    auto c0 = std::make_shared<GenericConstraint>();
    auto c1 = std::make_shared<GenericLinearConstraint>();
    auto c2 = std::make_shared<bopt::bounding_box_constraint_tpl<double>>(
        2, 0, Eigen::Vector2d(-1.0, -2.0), Eigen::Vector2d(2.0, 1.0));

    p.add_constraint(c0, x.topRows(2));
    p.add_constraint(c0, x.middleRows(1, 2));
    p.add_constraint(c1, x.middleRows(5, 2));
    p.add_bounding_box_constraint(c2, x.middleRows(3, 2));

    EXPECT_EQ(p.n_constraints(), 6);
}

TEST(Program, AddCosts) {
    bopt::mathematical_program<double> p("program");
    bopt::variable_vector x = bopt::create_variable_vector("x", 10);
    p.add_variables(x);

    auto c0 = std::make_shared<GenericConstraint>();
    auto c1 = std::make_shared<GenericLinearConstraint>();
    auto c2 = std::make_shared<bopt::bounding_box_constraint_tpl<double>>(
        2, 0, Eigen::Vector2d(-1.0, -2.0), Eigen::Vector2d(2.0, 1.0));

    p.add_constraint(c0, x.topRows(2));
    p.add_constraint(c0, x.middleRows(1, 2));
    p.add_constraint(c1, x.middleRows(5, 2));
    p.add_bounding_box_constraint(c2, x.middleRows(3, 2));

    EXPECT_EQ(p.n_constraints(), 6);
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