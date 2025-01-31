#include "bopt/evaluator.hpp"

namespace bopt {

std::ostream& operator<<(std::ostream& os, const EvaluatorBase& e) {
    os << "EvaluatorBase\n";
    os << "description: " << e.description() << '\n';
    os << "input dim: " << e.dim_input() << '\n';
    os << "output dim: " << e.dim_output();
    return os;
}

}  // namespace bopt