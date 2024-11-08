#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "bopt/logging.hpp"
#include "bopt/profiler.hpp"
#include "bopt/planning/optimisation/collocation/base.h"

using SX = ::casadi::SX;
using DM = ::casadi::DM;

class DoubleIntegratorDynamics {
 public:
  ::casadi::Function dynamics() {
    // Create state and control
    SX q = SX::sym("q", 1);
    SX v = SX::sym("v", 1);

    SX u = SX::sym("u", 1);


    // Create parameter for mass
    SX m = SX::sym("m", 1);

    // Create system dynamics
    SX x = SX::vertcat({q, v});

    SX a = u / m;

    SX dx = SX::vertcat({v, a});

    SX p = SX::vertcat({m});

    // Return function of the form dx = f(x, u, p)
    return ::casadi::Function("dynamics",{x, u, p}, {dx});
  }
 private:
};

TEST(Collocation, Trapezoidal) { 
  
  DoubleIntegratorDynamics dynamics;

  // Create collocation constraint
  bopt::planning::optimisation::DynamicsFunctionInformation dyn_info;
  bopt::planning::optimisation::CollocationInformation col_info;

  dyn_info.state_info.nx = 2;
  dyn_info.state_info.nt = 1;
  dyn_info.control_info.nu = 1;
  dyn_info.np = 1;

  col_info.variable_time = true;

  auto con = bopt::planning::optimisation::createCollocationConstraint(dyn_info, dynamics.dynamics(), col_info);
 
}

TEST(Collocation, SimpsonHermite) { 
  
  DoubleIntegratorDynamics dynamics;

  // Create collocation constraint
  bopt::planning::optimisation::DynamicsFunctionInformation dyn_info;
  bopt::planning::optimisation::CollocationInformation col_info;

  dyn_info.state_info.nx = 2;
  dyn_info.state_info.nt = 1;
  dyn_info.control_info.nu = 1;
  dyn_info.np = 1;

  col_info.variable_time = true;
  col_info.type = bopt::planning::optimisation::CollocationType::SIMPSON_HERMITE;

  auto con = bopt::planning::optimisation::createCollocationConstraint(dyn_info, dynamics.dynamics(), col_info);
 
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  testing::InitGoogleTest(&argc, argv);
  
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;
  FLAGS_log_prefix = 1;
  FLAGS_v = 10;
  
  int status = RUN_ALL_TESTS();
  bopt::Profiler summary;
  return status;
}
