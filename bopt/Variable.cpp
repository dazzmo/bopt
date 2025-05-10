#include "bopt/Variable.hpp"

namespace bopt {

VariableVector createVariableVector(const std::string &name,
                                       const Eigen::Index &sz) {
    // DBGASSERT(sz >= 0);
    VariableVector res(sz);
    for (Eigen::Index i = 0; i < sz; ++i) {
        res[i] = bopt::Variable(name + "_" + std::to_string(i));
    }
    return res;
}

// Operator overloading

std::ostream &operator<<(std::ostream &os, const bopt::Variable &var) {
    return os << var.name();
}

std::ostream &operator<<(std::ostream &os, const bopt::variable_indices &vi) {
    os << "indices: {";
    for (const auto &idx : vi.indices()) {
        os << idx << ' ';
    }
    os << "}\n";
    os << "is_block: " << (vi.is_block() ? "true" : "false");
    return os;
}

}  // namespace bopt
