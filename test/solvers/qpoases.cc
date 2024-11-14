
#define BOPT_USE_PROFILING
#include "bopt/solvers/qpoases.hpp"

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "bopt/ad/casadi.hpp"
#include "bopt/logging.hpp"
#include "bopt/variable.hpp"

using sym = ::casadi::SX;
using dm = ::casadi::DM;

namespace dcas = bopt::casadi;

typedef double value_type;

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

    // Create constraint with casadi

    program.add_constraint(ptr, {{x}}, {{p1, p2}});

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