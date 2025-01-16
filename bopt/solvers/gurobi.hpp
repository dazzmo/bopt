#pragma once

#define BOPT_WITH_GUROBI
#ifdef BOPT_WITH_GUROBI

#include <gurobi_c++.h>

#include "bopt/program.hpp"
#include "bopt/solvers/base.hpp"

namespace bopt {
namespace solvers {

struct gurobi_info : public solver_information<double> {};

struct gurobi_options : public solver_options<double> {
    bool perform_hotstart = false;
};

class gurobi_solver_instance : public solver<double> {
   public:
    gurobi_solver_instance() = default;
    gurobi_solver_instance(mathematical_program<double>& program)
        : solver<double>(program) {
        // Create gurobi environment
        GRBEnv env = GRBEnv(true);
        env.set("LogFile", "mip1.log");
        env.start();
    }

    ~gurobi_solver_instance() = default;

    // void reset();
    // void solve(mathematical_program<double>& program);

   private:
    bool first_solve_ = true;
    int n_solves_ = 0;

    gurobi_options options_;
    gurobi_info info_;
};

}  // namespace solvers
}  // namespace bopt
#endif