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
struct qpoases_info : public solver_information<double, std::size_t> {
    qpOASES::QProblemStatus status;
    // Return status for the qpOASES solver
    int returnStatus;
    // Error code for the qpOASES solver
    int errorCode;
    // Number of working sets performed
    int nWSR;

    index_type number_of_solves = 0;
};

struct qpoases_options : public solver_options<double, std::size_t> {
    // Number of working sets performed
    int nWSR = 100;

    bool perform_hotstart;
};

template <typename MatrixType, typename VectorType>
struct qpoases_data {
    MatrixType H;
    VectorType g;

    MatrixType A;

    VectorType ubA;
    VectorType lbA;

    VectorType lbx;
    VectorType ubx;
};

class qpoases_solver_instance : public solver<double, std::size_t> {
   public:

    qpoases_data<Eigen::MatrixX<double>, Eigen::VectorX<double>> data;

    qpoases_solver_instance() = default;
    qpoases_solver_instance(mathematical_program<double>& prog);

    ~qpoases_solver_instance();

    void reset();
    void solve();

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
