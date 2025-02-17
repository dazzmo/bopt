#include "bopt/program.hpp"

namespace bopt {

std::ostream &operator<<(std::ostream &os, const MathematicalProgram &program) {
    os << "Program:\n";
    os << "name: " << program.name() << '\n';
    os << "number of variables: " << program.n_variables() << '\n';
    os << "number of constraints: " << program.n_constraints() << '\n';
    // Variables
    os << "variables:\n";
    const auto &v = program.getAllVariables();
    for (const auto &vi : v) {
        os << vi.name() << '\n';
    }
    os << "constraints: \n";
    return os;
}

}  // namespace bopt
