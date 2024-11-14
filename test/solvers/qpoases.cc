
#define BOPT_USE_PROFILING
#include "bopt/solvers/qpoases.hpp"

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "bopt/ad/casadi/casadi.hpp"
#include "bopt/logging.hpp"
#include "bopt/variable.hpp"

using sym_t = ::casadi::SX;

typedef double value_type;

TEST(qpoases, LinearProgram) {
    bopt::mathematical_program<double> program("basic program");

    std::vector<bopt::variable> x;
    for (int i = 0; i < 10; ++i) {
        x.push_back(bopt::variable("x" + std::to_string(i)));
        program.add_variable(x.back(), 0.0, -50.0, 50.0);
    }

    // Create constraint with casadi
    sym_t xs = sym_t::sym("x", 10);
    sym_t expr = sym_t(2, 1);

    expr(0) = 1.0 * xs(1) + 2.0 * xs(2) + 1;
    expr(1) = 3.0 * xs(3) + 4.0 * xs(4) - 1;

    program.add_linear_constraint(
        bopt::casadi::linear_constraint<double>::create(
            expr, xs, sym_t(), bopt::bound_type::Equality),
        {{x}}, {{}});

    sym_t cost = sym_t::dot(xs, xs);
    program.add_quadratic_cost(
        bopt::casadi::quadratic_cost<double>::create(cost, xs, sym_t()), {{x}},
        {{}});

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