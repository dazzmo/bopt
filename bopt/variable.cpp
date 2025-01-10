#include "bopt/variable.hpp"

namespace bopt {

variable_vector create_variable_vector(const std::string &name,
                                       const Eigen::Index &sz) {
    DBGASSERT(sz >= 0);
    variable_vector res(sz);
    for (Eigen::Index i = 0; i < sz; ++i) {
        res[i] = bopt::variable(name + "_" + std::to_string(i));
    }
    return res;
}

// Operator overloading

std::ostream &operator<<(std::ostream &os, const bopt::variable &var) {
    return os << var.name();
}

}  // namespace bopt
