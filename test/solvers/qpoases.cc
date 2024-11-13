
#define BOPT_USE_PROFILING
#include "bopt/solvers/qpoases.h"

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "bopt/logging.hpp"
#include "bopt/variable.hpp"

using sym = ::casadi::SX;
using dm = ::casadi::DM;

namespace dcas = bopt::casadi;

typedef double value_type;

class CostFunction : public bopt::linear_cost<value_type> {
   public:
   private:
};

class ConstraintFunction : public bopt::linear_constraint<value_type> {
   public:
    typedef std::shared_ptr<ConstraintFunction> shared_ptr;
    typedef bopt::linear_constraint<double> linear_constraint;

    ConstraintFunction() : linear_constraint(1) {
        this->out_n = 1;
    }

    integer_type operator()(const value_type **arg, value_type *res) {
        return integer_type(0);
    }

    integer_type A_info(bopt::evaluator_out_info<linear_constraint> &info) override {
        info.m = 1;
        info.n = 1;
        info.nnz = 1;
        info.type = bopt::evaluator_matrix_type::Dense;
        return integer_type(0);
    }

    integer_type A(const value_type **arg, value_type *res) {
        res[0] = arg[0][0];
        return integer_type(0);
    }

    integer_type b_info(bopt::evaluator_out_info<linear_constraint> &info) {
        info.m = 1;
        info.n = 1;
        info.nnz = 1;
        info.type = bopt::evaluator_matrix_type::Dense;
        return integer_type(0);
    }

    integer_type b(const value_type **arg, value_type *res) {
        res[0] = arg[0][1];
        return integer_type(0);
    }

   private:
};

TEST(qpoases, LinearProgram) {
    bopt::mathematical_program<double> program("basic program");

    auto x = bopt::variable("x");
    auto p1 = bopt::variable("p1");
    auto p2 = bopt::variable("p2");

    // Eventually, we'll use casadi and simply go
    /**
     * sym x;
     * sym c = x * x;
     *
     * constraint b = make_constraint(c, x);
     * program.add_constraint(b, x);
     *
     */

    program.add_variable(x, 0.0, -5.0, 5.0);
    program.add_parameter(p1);
    program.add_parameter(p2);

    program.add_constraint<bopt::linear_constraint<double>>(
        std::make_shared<ConstraintFunction>(), {{x}}, {{p1, p2}});

    program.set_parameter(p1, 1.0);
    program.set_parameter(p2, -1.0);

    for (const auto &pi : program.p()) {
        LOG(INFO) << pi;
    }

    for (const auto &x : program.variable_bounds().m_values) {
        LOG(INFO) << x.lower << " " << x.upper;
    }

    // Solve the program
    bopt::solvers::qpoases_solver_instance qp(program);
    qp.solve();
}

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    testing::InitGoogleTest(&argc, argv);

    FLAGS_logtostderr = 1;
    FLAGS_colorlogtostderr = 1;
    FLAGS_log_prefix = 1;
    // FLAGS_v = 10;

    int status = RUN_ALL_TESTS();

    bopt::profiler summary;
    return status;
}