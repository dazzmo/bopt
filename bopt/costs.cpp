#include "bopt/costs.hpp"

namespace bopt {
std::ostream& operator<<(std::ostream& os, const Cost& c) {
    os << "cost:\n";
    os << "name: " << c.name() << '\n';
    os << "description: " << c.description();
    return os;
}
}  // namespace bopt