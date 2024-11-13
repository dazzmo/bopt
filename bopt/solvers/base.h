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

template <class ValueType, class IndexType>
struct solver_information {
    typedef ValueType value_type;
    typedef IndexType index_type;

    // Whether the solver was successful
    bool success;
    // Number of iterations performed
    index_type iterations;
    // Execution time
    value_type execution_time;
};

template <class ValueType, class IndexType>
struct solver_options {
    typedef ValueType value_type;
    typedef IndexType index_type;

    // Number of iterations performed
    index_type max_iterations;
    // Execution time (s)
    value_type max_execution_time;
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
template <class ValueType, class IntegerType = int,
          class IndexType = std::size_t>
class solver {
   public:
    typedef ValueType value_type;
    typedef IntegerType integer_type;
    typedef IndexType index_type;

    // todo - make this const?
    solver(mathematical_program<value_type>& program) : m_program(program) {}
    ~solver() {}

    void solve() {}

    mathematical_program<value_type>& program() { return m_program; }

   private:
    // Reference to current program in solver
    mathematical_program<value_type>& m_program;
};

}  // namespace solvers
}  // namespace bopt

#endif /* SOLVERS_BASE_H */
