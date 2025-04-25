#ifndef SOLVERS_BASE_H
#define SOLVERS_BASE_H

#include <unordered_map>

#include "bopt/program.hpp"

namespace bopt {
namespace solvers {

template <class ValueType>
struct solver_information {
    // Whether the solver was successful
    bool success;
    // Number of iterations performed
    bopt_index iterations;
    // Execution time
    ValueType execution_time;
};

template <class ValueType>
struct solver_options {
    // Number of iterations performed
    bopt_index max_iterations;
    // Execution time (s)
    ValueType max_execution_time;
};

/**
 * @brief Solver base class
 *
 * Any solver should have the following information
 *
 * A solution x
 * An optimal return value f(x)
 * An optimal constraint set g(x)
 *
 * Any other information could be
 *
 */
template <class ValueType>
class solver {
   public:
    // todo - make this const?
    solver(const MathematicalProgram& program) {
        primal_solution_ = Eigen::VectorXd::Zero(program.numVariables());
    }
    ~solver() {}

    void solve() {}

    const Eigen::VectorXd& getPrimalSolution() const { return primal_solution_; }

    // void evaluate_constraint()
   protected:
    Eigen::VectorXd& primal_solution() { return primal_solution_; }

   private:
    Eigen::VectorXd primal_solution_;
};

}  // namespace solvers
}  // namespace bopt

#endif /* SOLVERS_BASE_H */
