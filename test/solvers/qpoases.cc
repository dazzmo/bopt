
#include "bopt/solvers/qpoases.h"

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "bopt/ad/casadi/function.hpp"
#include "bopt/logging.hpp"
#include "bopt/profiler.hpp"
#include "bopt/program.h"
#include "bopt/symbolic/variable.hpp"

using sym = ::casadi::SX;
using dm = ::casadi::DM;

namespace dcas = bopt::casadi;
namespace dsym = bopt::symbolic;
namespace dopt = bopt::optimisation;

class BasicObjective {
 public:
  BasicObjective() : program() {
    std::size_t n = 2;
    // Symbolic cost creation
    sym s = sym::sym("s", n);

    dopt::QuadraticCost::SharedPtr obj =
        std::make_shared<dcas::QuadraticCost>("qc", sym::dot(s, s), s);

    dopt::LinearConstraint::SharedPtr con =
        std::make_shared<dcas::LinearConstraint>("lc", s(0) + s(1), s);
    con->setBoundsFromType(dopt::BoundType::POSITIVE);

    dopt::BoundingBoxConstraint::SharedPtr bb =
        std::make_shared<dopt::BoundingBoxConstraint>("bb", n);
    bb->setLowerBound(-1.0);
    bb->setUpperBound(1.0);

    // Create variables
    dsym::Vector x = dsym::createVector("x", n);

    program.x().add(x);
    // Create objective
    program.f().add(obj, x, {});
    // Add constraints
    program.g().add(con, x, {});
    // TODO - Add bounding box constraint functionality
    program.g().add(bb, x, {});

    LOG(INFO) << program.g();
  }

  dopt::MathematicalProgram program;

 private:
};

TEST(qpoases, BasicObjective) {
  BasicObjective problem;
  dopt::solvers::QPOASESSolverInstance qp(problem.program);

  // Attempt to solve the program
  qp.solve();

  std::cout << qp.getPrimalSolution() << '\n';
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