
#include "bopt/solvers/qpoases.h"

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "bopt/logging.hpp"
#include "bopt/variable.hpp"

using sym = ::casadi::SX;
using dm = ::casadi::DM;

namespace dcas = bopt::casadi;
namespace dsym = bopt::symbolic;
namespace dopt = bopt::optimisation;

typedef double value_type;

class CostFunction : public bopt::LinearCost<value_type> {
   public:
   private:
};

class ConstraintFunction : public bopt::LinearConstraint<value_type> {
   public:
    ConstraintFunction() {
        this->out_n = 1;
        this->bounds.resize(this->out_n);
        this->bounds[0].set(bopt::bound_type::StrictlyPositive);
    }

    index_type A_info(bopt::evaluator_out_info<ConstraintFunction> &info) {
        info.m = 1;
        info.n = 1;
        info.nnz = 1;
        return index_type(0);
    }

    index_type A(const value_type **arg, value_type *res) {
        res[0] = arg[1][0];
    }

    index_type b_info(bopt::evaluator_out_info<ConstraintFunction> &info) {
        info.m = 1;
        info.n = 1;
        info.nnz = 1;
        return index_type(0);
    }

    index_type b(const value_type **arg, value_type *res) {
        res[0] = arg[1][1];
    }

   private:
};

TEST(qpoases, LinearProgram) {
    bopt::MathematicalProgram program;

    auto x = bopt::Variable("x");
    auto p1 = bopt::Variable("p1");
    auto p2 = bopt::Variable("p2");

    // Eventually, we'll use casadi and simply go
    /**
     * sym x;
     * sym c = x * x;
     *
     * constraint b = make_constraint(c, x);
     * program.add_constraint(b, x);
     *
     */

    program.addDecisionVariable(x);
    program.addParameter(p1);
    program.addParameter(p2);

    program.appendLinearConstraint(std::make_shared<ConstraintFunction>(),
                                   {{x}, {p1, p2}});

    program.setParameter(p1, 1.0);
    program.setParameter(p2, -1.0);
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

    bopt::Profiler summary;
    return status;
}