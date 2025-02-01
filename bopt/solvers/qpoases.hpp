#ifndef SOLVERS_QPOASES_H
#define SOLVERS_QPOASES_H

// #ifdef WITH_QPOASES

#include <boost/numeric/ublas/matrix.hpp>
#include <qpOASES.hpp>

#include "bopt/logging.hpp"
#include "bopt/profiler.hpp"
#include "bopt/program.hpp"
#include "bopt/solvers/base.hpp"

namespace bopt {
namespace solvers {

/**
 * @brief Details for the qpOASES solver
 *
 */
struct qpoases_info : public solver_information<double> {
    qpOASES::QProblemStatus status;
    // Return status for the qpOASES solver
    int returnStatus;
    // Error code for the qpOASES solver
    int errorCode;
    // Number of working sets performed
    int nWSR;

    bopt_index number_of_solves = 0;
};

struct qpoases_options : public solver_options<double>,
                         public qpOASES::Options {
    // Number of working sets performed
    int nWSR = 100;
    bool perform_hotstart = false;
};

struct qpoases_data {
    Eigen::MatrixXd H;
    Eigen::VectorXd g;

    Eigen::MatrixXd A;

    Eigen::VectorXd ubA;
    Eigen::VectorXd lbA;

    Eigen::VectorXd lbx;
    Eigen::VectorXd ubx;
};

class qpoases_solver : public solver<double> {
   public:
    qpoases_data data;

    qpoases_solver() = default;
    qpoases_solver(MathematicalProgram<double>& program);

    ~qpoases_solver();

    qpoases_options& options() { return options_; }

    void reset();
    void solve(MathematicalProgram<double>& program);

   private:
    bool first_solve_ = true;
    int n_solves_ = 0;

    std::unique_ptr<qpOASES::SQProblem> qp_;

    qpoases_options options_;
    qpoases_info info_;
};

}  // namespace solvers
}  // namespace bopt

// #endif /* WITH_QPOASES */
#endif /* SOLVERS_QPOASES_H */
