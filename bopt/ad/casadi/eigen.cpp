#include "bopt/ad/casadi/eigen.hpp"

namespace bopt {
namespace casadi {

::casadi::Function toCasadiFunction(
    const std::string &name,
    const std::vector<Eigen::VectorX<::casadi::SX>> &in,
    const std::vector<Eigen::VectorX<::casadi::SX>> &out) {
  // Casadi input and output vectors
  ::casadi::SXVector in_cs = {}, out_cs = {};
  // Inputs
  for (const auto &xi : in) {
    ::casadi::SX xc;
    toCasadi(xi, xc);
    in_cs.push_back(xc);
  }

  // Outputs
  for (const auto &oi : out) {
    ::casadi::SX oc;
    toCasadi(oi, oc);
    out_cs.push_back(::casadi::SX::densify(oc));
  }

  // Create function
  return ::casadi::Function(name, in_cs, out_cs);
}

}  // namespace casadi

}  // namespace bopt
