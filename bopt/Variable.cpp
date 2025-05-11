#include "bopt/Variable.hpp"

namespace bopt {

VariableVector createVariableVector(const std::string &name, const Size &sz) {
    VariableVector res(sz);
    for (Size i = 0; i < sz; ++i) {
        res[i] = bopt::Variable(name + "_" + std::to_string(i));
    }
    return res;
}

// Operator overloading

std::ostream &operator<<(std::ostream &os, const bopt::Variable &var) {
    return os << var.name();
}

std::ostream &operator<<(std::ostream &os,
                         const bopt::VariableIndexManager &m) {
    os << "Indices: {";
    for (const auto &idx : m.getIndices()) {
        os << idx << ' ';
    }
    os << "}\n";
    os << "Seqeuntial: " << (m.isSequential() ? "true" : "false");
    return os;
}

}  // namespace bopt
