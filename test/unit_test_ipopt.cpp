#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/ad/casadi.hpp"
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

    bopt::evaluator::return_status eval_A_impl(sparse_matrix_t &out) override {
        out.coeffRef(0, 0) = 1.0;
        out.coeffRef(1, 1) = 1.0;
        return bopt::evaluator::return_status::Success;
    }

    void get_A_sparsity_impl(sparse_matrix_t &out) const override {
        out.resize(2, 2);
        out.coeffRef(0, 0) = 0.0;
        out.coeffRef(1, 1) = 0.0;
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

   protected:
    bopt::evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_vector_t> out) override {
        out[0] = x[1];
        out[1] = x[0] + x[1];
        return bopt::evaluator::return_status::Success;
    }

    void get_jacobian_sparsity_impl(sparse_matrix_t &jac) const override {
        jac.resize(2, 2);
        jac.coeffRef(0, 1) = 0.0;
        jac.coeffRef(1, 0) = 0.0;
        jac.coeffRef(1, 1) = 0.0;
    }

    bopt::evaluator::return_status eval_A_impl(
        Eigen::Ref<dense_matrix_t> out) override {
        out(0, 1) = 1.0;
        out(1, 0) = 1.0;
        out(1, 1) = 1.0;
        return bopt::evaluator::return_status::Success;
    }

    bopt::evaluator::return_status eval_b_impl(
        Eigen::Ref<dense_vector_t> out) override {
        out << 1.0, 2.0;
        return bopt::evaluator::return_status::Success;
    }
};

// TEST(Program, SimpleProgram) {
//     auto c = std::make_shared<GenericQuadraticCost>();
//     auto g0 = std::make_shared<GenericLinearConstraint>();
//     g0->set_lower_bound(Eigen::Vector2d(1.0, 2.0));
//     g0->set_upper_bound(Eigen::Vector2d(10.0, 5.0));

//     auto x = bopt::create_variable_vector("x", 2);

//     bopt::mathematical_program<double> p("program");
//     p.add_variables(x);

//     p.add_cost(c, x);
//     p.add_linear_constraint(g0, x);

//     auto nlp = bopt::solvers::ipopt_solver(p);
//     nlp.options()->SetNumericValue("tol", 1e-3);
//     nlp.options()->SetStringValue("mu_strategy", "adaptive");
//     nlp.solve();
// }

class TestEvaluator : public bopt::EvaluatorBase {
   public:
    TestEvaluator()
        : bopt::EvaluatorBase(2, 2, "TestEvaluator: Basic Implementation") {
        setJacobianFlag(true);
        setHessianFlag(false);

        setNumberOutputs(2);
        setNumberOutputs(2);
    }

   private:
    void evalImpl(const Eigen::Ref<const Eigen::VectorXd> &x,
                  Eigen::Ref<Eigen::VectorXd> y) {
        y = x;
    }

    void jacobianImpl(const Eigen::Ref<const Eigen::VectorXd> &x,
                      Eigen::Ref<Eigen::MatrixXd> jac) {
        jac.setIdentity();
    }
};

class TestAutoDiff : public bopt::TestAutodiffModule {
   public:
    TestAutoDiff() { z_ = VectorADD(20); }

    void eval(const Eigen::Ref<const VectorADD> &x,
              Eigen::Ref<VectorADD> y) override {
        evalImpl(x, y);
    }

    void eval(const Eigen::Ref<const VectorAD> &x, Eigen::Ref<VectorAD> y) {
        evalImpl(x, y);
    }

    void eval(const Eigen::Ref<const VectorADD> &x,
              const Eigen::Ref<const Eigen::VectorXd> &lambda, ADD &y) {
        evalImpl<ADD>(x, z_);
        y = lambda.cast<ADD>().dot(z_);
    }

    template <typename T>
    void evalImpl(const Eigen::Ref<const Eigen::VectorX<T>> &x,
                  Eigen::Ref<Eigen::VectorX<T>> y) {
        y = x;
        for (int i = 1; i < x.size(); ++i) {
            y[i] *= x[i - 1];
            y[i] *= y[i];
        }
    }

   private:
    VectorADD z_;
};

TEST(Program, Rosenbrock) {
    bopt::profiler profile("total");
    int N = 50;
    using bopt::casadi::sym_t;
    sym_t x = sym_t::sym("x", N);
    sym_t p = sym_t::sym("p", 1);
    sym_t f = 0.0;

    for (int i = 0; i < N - 1; ++i) {
        f += 100.0 * pow(x(i + 1) - pow(x(i), 2), 2) + pow(1.0 - x(i), 2);
    }

    auto opt = bopt::casadi::evaluator::differentiable::scalar::options();
    opt.dense_gradient = true;
    opt.dense_hessian = false;

    auto c = std::make_shared<bopt::cost_tpl<double>>(
        std::make_shared<bopt::casadi::evaluator::differentiable::scalar>(
            f, x, p, opt));

    bopt::mathematical_program<double> pg("rosenbrock");
    bopt::variable_vector v = bopt::create_variable_vector("x", N);
    for (int i = 0; i < N; ++i) {
        pg.add_variable(v[i], 0.5, -10, 10.0);
    }

    pg.add_cost(c, v);

    auto nlp = bopt::solvers::ipopt_solver(pg);
    nlp.options()->SetNumericValue("tol", 1e-3);
    nlp.options()->SetStringValue("mu_strategy", "adaptive");
    // nlp.options()->SetStringValue("hessian_approximation", "limited-memory");
    try {
        {
            bopt::profiler profile("ipopt");
            nlp.solve();
        }
    } catch (std::exception &e) {
        LOG(ERROR) << e.what();
    }

    TestEvaluator e;
    e.setDescription("This is an evaluator base object, it is pretty cool!");
    VLOG(10) << e;

    TestAutoDiff ad;
    int n = 20;
    TestAutoDiff::VectorADD xad(n);
    TestAutoDiff::VectorADD yad(n);

    TestAutoDiff::VectorAD wad(n);
    TestAutoDiff::VectorAD zad(n);

    // Create autodiff for jacobian evaluation
    // Create autodiff for hessian evaluation

    {
        bopt::profiler profile("create");
        for (int i = 0; i < n; ++i) {
            xad(i).value() = 1.0;
            wad(i).value() = 1.0;
            xad(i).derivatives() = Eigen::VectorXd::Unit(n, i);
            xad(i).value().derivatives() = Eigen::VectorXd::Unit(n, i);
            wad(i).derivatives() = Eigen::VectorXd::Unit(n, i);
        }

        // Hessian
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                xad(i).derivatives()(j).derivatives() =
                    Eigen::VectorXd::Zero(n);
            }
        }
    }

    // Remove auto diff row to avoid computation?
    TestAutoDiff::ADD ly;
    Eigen::VectorXd lambda(n);
    lambda.setConstant(5.0);
    for (int k = 0; k < 10000; ++k) {
        {
            bopt::profiler profile("ad eval");
            ad.eval(wad, zad);
        }
        {
            bopt::profiler profile("ad hessian");
            ad.eval(xad, lambda, ly);
        }
    }

    LOG(INFO) << yad;
    LOG(INFO) << "Jacobian";
    for (int i = 0; i < n; ++i) {
        LOG(INFO) << zad(i).derivatives().transpose();
    }
    LOG(INFO) << "Hessian";
    for (int i = 0; i < n; ++i) {
        LOG(INFO) << ly.derivatives()(i).derivatives().transpose();
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