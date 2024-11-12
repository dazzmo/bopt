#include "bopt/variable.hpp"

namespace bopt {

// Operator overloading

std::ostream &operator<<(std::ostream &os, bopt::variable var) {
    return os << var.name();
}

}  // namespace bopt
