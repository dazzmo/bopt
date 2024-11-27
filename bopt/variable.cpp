#include "bopt/variable.hpp"

namespace bopt {

std::vector<bopt::variable> create_variable_vector(const std::string &name,
                                                   const std::size_t &sz) {
  std::vector<bopt::variable> res(sz);
  for (std::size_t i = 0; i < sz; ++i) {
    res[i] = bopt::variable(name + "_" + std::to_string(i));
  }
  return res;
}

// Operator overloading

std::ostream &operator<<(std::ostream &os, bopt::variable var) {
    return os << var.name();
}

}  // namespace bopt
