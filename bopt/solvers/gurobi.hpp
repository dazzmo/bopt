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
    gurobi_solver_instance(mathematical_program<double> &program)
        : solver<double>(program) {
        // Create gurobi environment

        try {
            GRBEnv env = GRBEnv(true);
            env.set("LogFile", program.name() + ".log");
            env.start();
        } catch (GRBException e) {
            LOG(ERROR) << "Error code = " << e.getErrorCode();
            LOG(ERROR) << e.getMessage();
        }

        // Create an empty model
        // model_ = std::make_unique<GRBModel>(env);

        // Create variables
        // for (const auto &x : program.variables()) {
        //     // model.addVar()
        // }

        // GRBLinExpr lin_costs;
        // GRBQuadExpr qdr_costs;

        // // Linear costs
        // for (auto &binding : program.linearCosts()) {
        //     auto &a = binding.get()->buffer_a().dense;
        //     binding.get()->eval_a(a);
        //     for (int i = 0; i < a.rows(); ++i) {
        //         if (a[i] != 0) lin_costs += a[i];
        //     }
        // }

        // VLOG(10) << lin_costs;

        // qdr_costs += lin_costs;

        // // Quadratic costs
        // model_->setObjective(qdr_costs);
        // Other costs

        // Matrix constraint

        //   GRBVar* vars = model.addVars(lb, ub, NULL, vtype, NULL, cols);
    }

    void solve(mathematical_program<double> &program) { model_->optimize(); }

    ~gurobi_solver_instance() = default;

    // void reset();
    // void solve(mathematical_program<double>& program);

   private:
    bool first_solve_ = true;
    int n_solves_ = 0;

    gurobi_options options_;
    gurobi_info info_;

    std::unique_ptr<GRBModel> model_;
};

}  // namespace solvers
}  // namespace bopt
#endif