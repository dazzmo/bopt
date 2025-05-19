#include "bopt/Program.hpp"

namespace bopt {

std::ostream &operator<<(std::ostream &os, const MathematicalProgram &program) {
    os << "Program:\n";
    os << "name: " << program.name() << '\n';
    os << "number of variables: " << program.numVariables() << '\n';
    os << "number of constraints: " << program.numConstraints() << '\n';
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
