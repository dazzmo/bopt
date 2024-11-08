
#include "bopt/binding.h"

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <casadi/casadi.hpp>

#include "bopt/ad/casadi/casadi.hpp"
#include "bopt/costs.hpp"
#include "bopt/logging.hpp"
#include "bopt/profiler.hpp"
#include "bopt/variable.hpp"

using sym = ::casadi::SX;
using dm = ::casadi::DM;

namespace dcas = bopt::casadi;

// Can do a generic casadi nonlinaer constraint

class CustomLinearCost : public bopt::LinearCost {
   public:
    CustomLinearCost() : bopt::LinearCost() {
        // Compute the coefficients
        sym x = sym::sym("x", 1);
        dm c = 10.0;
        dm b = -1.23;

        // Create symbolic constraint
        sym f = sym(c) * x + sym(b);

        // Codegen the expressions
        dcas::codegen(::casadi::Function("lincost_c", {}, {sym(c)}));
        dcas::codegen(::casadi::Function("lincost_b", {}, {sym(b)}));

        // Create a linear cost evaluator
        std::shared_ptr<bopt::DynamicLibraryHandler> handle;
        handle = std::make_shared<bopt::DynamicLibraryHandler>(
            "./cg_lincost_c_18121696174184105721.so");
        c_ = std::make_shared<dcas::Evaluator>(handle, "lincost_c");

        handle = std::make_shared<bopt::DynamicLibraryHandler>(
            "./cg_lincost_c_18121696174184105721.so");
        b_ = std::make_shared<dcas::Evaluator>(handle, "lincost_c");
    }

    // Evaluate A and b
    bopt_int A(const double **arg, double *res) override {
        c_->eval(arg);
        *res = c_->buffer[0];
        return 0;
    }

    bopt_int b(const double **arg, double *res) override {
        c_->eval(arg);
        *res = c_->buffer[0];
        return 0;
    }

   private:
    std::shared_ptr<bopt::EvaluatorBase> c_;
    std::shared_ptr<bopt::EvaluatorBase> b_;
};

TEST(Optimisation, BindScalarCost) {
    std::shared_ptr<bopt::LinearCost> ptr =
        std::make_shared<CustomLinearCost>();

  // Create binding for the constraint
  std::vector<bopt::Variable> x = {};
  x.emplace_back(bopt::Variable("x"));

  bopt::Binding<bopt::LinearCost> binding(ptr, {x});

}

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    testing::InitGoogleTest(&argc, argv);
    int status = RUN_ALL_TESTS();
    bopt::Profiler summary;
    return status;
}