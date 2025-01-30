#include "bopt/evaluator.hpp"

namespace bopt {

std::ostream& operator<<(std::ostream& os, const EvaluatorBase& e) {
    os << "EvaluatorBase\n";
    os << "description: " << e.description() << '\n';
    os << "n_inputs: " << e.n_inputs() << '\n';
    os << "n_outputs: " << e.n_outputs();
    return os;
}

}  // namespace bopt