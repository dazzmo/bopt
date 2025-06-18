#pragma once

#include <cassert>

#include "bopt/Program.hpp"

namespace bopt {
namespace solvers {

struct SolverInfoBase {
    SolverInfoBase() : success(false), num_iterations(0), wall_time(0) {}

    bool success;
    Index num_iterations;
    Real wall_time;
};

struct SolverResultsBase {
    using VectorX = typename MathTypes<Real>::VectorX;
    Real objective;
    VectorX primal;
};

template <typename SolverInfoType>
class SolverBase {
   public:
    using VectorX = typename MathTypes<Real>::VectorX;
    using SolverInfo = SolverInfoType;

    virtual const SolverInfo& getInfo() const = 0;

    virtual MathematicalProgram& getProgram() { return program_; };

    /**
     * @brief Initialise the solver for solving the given mathematical program.
     *
     */
    void init() {
        initImpl();
        initialised_ = true;
    }

    /**
     * @brief
     *
     * @note Ensure that init() has been called before calling this function.
     */
    void solve() {
        if (!initialised_) {
            std::stringstream ss;
            ss << "Solver: \' " << name_
               << "\' not initialised, call init() before solving!";
            throw std::runtime_error(ss.str());
        }
        solveImpl();
    }

    virtual const SolverResultsBase& getResults() const = 0;

   protected:
    SolverBase(MathematicalProgram& program, const std::string& name)
        : program_(program), name_(name) {}

    const MathematicalProgram& getProgram() const { return program_; };

    virtual void initImpl() = 0;
    virtual void solveImpl() = 0;

   private:
    bool initialised_{false};
    /// @brief Name of the solver
    String name_;
    /// @brief Reference to the program that the solver is to solve
    MathematicalProgram& program_;
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
   protected:
   public:
    // todo - make this const?
    solver(const MathematicalProgram& program) {
        primal_solution_ = Eigen::VectorXd::Zero(program.numVariables());
    }
    ~solver() {}

    void solve() {}

    const Eigen::VectorXd& getPrimalSolution() const {
        return primal_solution_;
    }

    // void evaluate_constraint()
   protected:
    Eigen::VectorXd& primal_solution() { return primal_solution_; }

   private:
    Eigen::VectorXd primal_solution_;
};

}  // namespace solvers
}  // namespace bopt
