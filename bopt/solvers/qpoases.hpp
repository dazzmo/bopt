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

namespace ublas = boost::numeric::ublas;

class qpOASESOptions : public OptionsBase {
   public:
    qpOASESOptions() {
        // Construct all options
        createIntegerOption("printLevel", qpOASES::PL_NONE);
        createIntegerOption("initialStatusBounds", qpOASES::ST_INACTIVE);
        createIntegerOption("numRegularisationSteps", qpOASES::ST_INACTIVE);
        createIntegerOption("numRefinementSteps", qpOASES::ST_INACTIVE);

        createBooleanOption("enableRamping", qpOASES::BT_FALSE);
        createBooleanOption("enableFarBounds", qpOASES::BT_FALSE);
        createBooleanOption("enableFlippingBounds", qpOASES::BT_FALSE);
        createBooleanOption("enableRegularisation", qpOASES::BT_FALSE);
        createBooleanOption("enableFullLITests", qpOASES::BT_FALSE);
        createBooleanOption("enableNZCTests", qpOASES::BT_FALSE);
        createBooleanOption("enableDriftCorrection", qpOASES::BT_FALSE);
        createBooleanOption("enableCholeskyRefactorization", qpOASES::BT_FALSE);
        createBooleanOption("enableEqualities", qpOASES::BT_FALSE);
        createBooleanOption("enableInertiaCorrection", qpOASES::BT_FALSE);

        createNumericalOption("terminationTolerance", 0.0);
        createNumericalOption("boundTolerance", 0.0);
        createNumericalOption("boundRelaxation", 0.0);
        createNumericalOption("epsNum", 0.0);
        createNumericalOption("epsDen", 0.0);
        createNumericalOption("maxPrimalJump", 0.0);
        createNumericalOption("maxDualJump", 0.0);
        createNumericalOption("initialRamping", 0.0);
        createNumericalOption("finalRamping", 0.0);
        createNumericalOption("initialFarBounds", 0.0);
        createNumericalOption("growFarBounds", 0.0);
        createNumericalOption("epsFlipping", 0.0);
        createNumericalOption("growFarBounds", 0.0);
        createNumericalOption("epsRegularisation", 0.0);
        createNumericalOption("epsIterRef", 0.0);
        createNumericalOption("epsLITests", 0.0);
        createNumericalOption("epsNZCTests", 0.0);
        createNumericalOption("rCondMin", 0.0);
    }

   private:
};

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
    typedef ublas::matrix<double> matrix_data_type;
    typedef std::vector<double> vector_data_type;

    qpoases_data<matrix_data_type, vector_data_type> data;

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
