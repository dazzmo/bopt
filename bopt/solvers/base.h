#ifndef SOLVERS_BASE_H
#define SOLVERS_BASE_H

#include <unordered_map>

#include "bopt/program.h"

namespace bopt {
namespace solvers {

class OptionsBase {
   public:
    template <typename T>
    using OptionsMap = std::unordered_map<std::string, T>;
    
    void setNumericalOption(const std::string& name, const double& val) {
        if (opt_num_.find(name) != opt_num_.end()) {
            opt_num_.at(name) = val;
        } else {
            // todo - throw error that option does not exist
        }
    }

    void setStringOption(const std::string& name, const std::string& val) {
        if (opt_str_.find(name) != opt_str_.end()) {
            opt_str_.at(name) = val;
        } else {
            // todo - throw error that option does not exist
        }
    }

    void setIntegerOption(const std::string& name, const bopt_int& val) {
        if (opt_int_.find(name) != opt_int_.end()) {
            opt_int_.at(name) = val;
        } else {
            // todo - throw error that option does not exist
        }
    }

    void setBooleanOption(const std::string& name, const bool& val) {
        if (opt_bln_.find(name) != opt_bln_.end()) {
            opt_bln_.at(name) = val;
        } else {
            // todo - throw error that option does not exist
        }
    }

    double getNumericalOption(const std::string& name) {
        if (opt_num_.find(name) != opt_num_.end()) {
            return opt_num_.at(name);
        } else {
            // todo - throw error that option does not exist
        }
    }

    std::string getStringOption(const std::string& name) {
        if (opt_str_.find(name) != opt_str_.end()) {
            return opt_str_.at(name);
        } else {
            // todo - throw error that option does not exist
        }
    }

    bopt_int getIntegerOption(const std::string& name) {
        if (opt_int_.find(name) != opt_int_.end()) {
            return opt_int_.at(name);
        } else {
            // todo - throw error that option does not exist
        }
    }

    bool getBooleanOption(const std::string& name) {
        if (opt_bln_.find(name) != opt_bln_.end()) {
            return opt_bln_.at(name);
        } else {
            // todo - throw error that option does not exist
        }
    }

   protected:
    void createNumericalOption(const std::string& name,
                               const double& default_value = 0.0) {
        opt_num_.insert({name, default_value});
    }
    void createStringOption(const std::string& name,
                            const std::string& default_value = "") {
        opt_str_.insert({name, default_value});
    }
    void createIntegerOption(const std::string& name,
                             const bopt_int& default_value = 0) {
        opt_int_.insert({name, default_value});
    }
    void createBooleanOption(const std::string& name,
                             const bool& default_value = false) {
        opt_bln_.insert({name, default_value});
    }

   private:
    OptionsMap<double> opt_num_;
    OptionsMap<bopt_int> opt_int_;
    OptionsMap<std::string> opt_str_;
    OptionsMap<bool> opt_bln_;
};

struct ProgramData {};

struct ProgramResults {
    double* x;
    double* lam;
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
class SolverBase {
   public:
    struct ResultsData {
        // Optimal value for x
        VectorX x;

        // Objective value at optimal solution
        Scalar objective_value;

        // Constraint vector
        VectorX constraint_vector;
    };

    struct SolverInformation {
        // Whether the solver was successful
        bool success;
        // Number of iterations performed
        Index iterations;
        Scalar execution_time;
    };

    struct SolverOptions {
        Index max_iterations;
    };

    ResultsData getResults() const { return ResultsData(); }

    SolverInformation getSolverInformation() const {
        return SolverInformation();
    }

    SolverOptions getSolverOptions() const { return SolverOptions(); }
    void setSolverOptions(const SolverOptions& options) {}

    SolverBase(MathematicalProgram& program) : program_(program) {}

    ~SolverBase() {}

    void solve(const MathematicalProgram& program) {}

    // solver.solve();
    // solver.getResults();

    MathematicalProgram& program() { return program_; }

   private:
    bool is_solved_ = false;
    // Reference to current program in solver
    MathematicalProgram& program_;
};

enum class Operation { SET = 0, ADD };

/**
 * @brief Update the sparse matrix with a block, where the rows and entries of
 * the block are indexed in the sparse matrix using the provided index vectors.
 *
 * @param M
 * @param block
 * @param row_indices
 * @param col_indices
 */
void updateSparseMatrix(Eigen::SparseMatrix<double>& M,
                        const Eigen::MatrixXd& block,
                        const std::vector<Eigen::Index>& row_indices,
                        const std::vector<Eigen::Index>& col_indices,
                        const Operation& operation = Operation::SET);

}  // namespace solvers
}  // namespace bopt

#endif /* SOLVERS_BASE_H */
