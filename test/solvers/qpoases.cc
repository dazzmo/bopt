
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
    std::vector<bopt::variable> p1, p2;
    for (int i = 0; i < 10; ++i) {
        x.push_back(bopt::variable("x" + std::to_string(i)));
        program.add_variable(x.back(), 0.0, -50.0, 50.0);
    }

    for (int i = 0; i < 10; ++i) {
        p1.push_back(bopt::variable("p1" + std::to_string(i)));
        p2.push_back(bopt::variable("p2" + std::to_string(i)));
        program.add_parameter(p1.back(), 0.0);
        program.add_parameter(p2.back(), 0.0);
    }

    // Create constraint with casadi
    sym_t xs = sym_t::sym("x", 10);
    sym_t p1s = sym_t::sym("p1", 10);
    sym_t p2s = sym_t::sym("p2", 10);
    sym_t expr = sym_t(2, 1);

    expr(0) = p1s(2) * xs(1) - 1;
    expr(1) = p2s(8) * xs(3) - 1;

    program.add_linear_constraint(
        bopt::casadi::linear_constraint<double>::create(
            expr, xs, casadi::SXVector({p1s, p2s}), bopt::bound_type::Equality),
        {x}, {p1, p2});

    sym_t cost = sym_t::dot(xs, xs);
    program.add_quadratic_cost(bopt::casadi::quadratic_cost<double>::create(
                                   cost, xs, casadi::SXVector()),
                               {x}, {p1, p2});

    program.set_parameter(p1[2], 1.0);
    program.set_parameter(p2[8], 1.0);

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