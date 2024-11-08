#include "bopt/variable.hpp"

namespace bopt {

// Operator overloading

std::ostream &operator<<(std::ostream &os, bopt::Variable var) {
    return os << var.name();
}

}  // namespace bopt
