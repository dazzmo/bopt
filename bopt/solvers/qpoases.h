#ifndef SOLVERS_QPOASES_H
#define SOLVERS_QPOASES_H

// #ifdef WITH_QPOASES

#include <qpOASES.hpp>

#include "bopt/logging.hpp"
#include "bopt/profiler.hpp"
#include "bopt/program.h"
#include "bopt/solvers/base.h"

namespace bopt {
namespace solvers {

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
struct qpOASESSolverInfo : public SolverBase::SolverInformation {
    qpOASES::QProblemStatus status;
    // Return status for the qpOASES solver
    int returnStatus;
    // Error code for the qpOASES solver
    int errorCode;
    // Number of working sets performed
    int nWSR;

    Index number_of_solves = 0;
};

struct qpOASESSolverOptions : public SolverBase::SolverOptions {
    // Number of working sets performed
    int nWSR;

    bool perform_hotstart;
};

struct qpOASESSolverResults : public SolverBase::ResultsData {};

class qpOASESSolverInstanceBase {};

class qpOASESSolverInstance {};

class qpOASESSparseSolverInstance {
   public:
    qpOASESSparseSolverInstance() = default;
    qpOASESSparseSolverInstance(MathematicalProgram& program);

    ~QPOASESSolverInstance();

    void reset();
    void solve();

   private:
    std::unique_ptr<qpOASES::QProblem> qp_;

    struct matrix_data {
        CompressedColumnStorageFormat H;
        CompressedColumnStorageFormat g;
        CompressedColumnStorageFormat A;
        CompressedColumnStorageFormat lbA;
        CompressedColumnStorageFormat ubA;
    };

    struct qpoases_matrix_data {
        qpOASES::SparseMatrix H;
        qpOASES::SparseMatrix g;
        qpOASES::SparseMatrix A;
        qpOASES::SparseMatrix lbA;
        qpOASES::SparseMatrix ubA;
    };

    matrix_data matrix_data_;
    qpoases_matrix_data qpoases_matrix_data_;

    qpOASESSolverInfo info_;
    qpOASESSolverOptions options_;
    qpOASESSolverResults results_;
};

class QPOASESSolverInstance : public SolverBase {
   public:
    struct QPOASESProgramData {
       public:
        QPOASESProgramData() = default;
        ~QPOASESProgramData() = default;

        QPOASESProgramData(const Index& nx, const Index& ng) {
            // todo - use qpOASES::Matrix
            H = MatrixX::Zero(nx, nx);
            g = VectorX::Zero(nx);
            A = MatrixX::Zero(ng, nx);

            lbx = VectorX::Zero(nx);
            ubx = VectorX::Zero(nx);

            constexpr double inf = std::numeric_limits<double>::infinity();
            lbx.setConstant(-inf);
            ubx.setConstant(inf);

            lbA = VectorX::Zero(ng);
            ubA = VectorX::Zero(ng);
        }

        MatrixX H;
        VectorX g;

        MatrixX A;

        VectorX lbx;
        VectorX ubx;

        VectorX lbA;
        VectorX ubA;
    };

    QPOASESSolverInstance() = default;
    QPOASESSolverInstance(MathematicalProgram& prog);

    ~QPOASESSolverInstance();

    void reset();
    void solve();
    void solveDense();
    void solveSparse();

    /**
     * @brief Get the current return status of the program
     *
     * @return const qpOASES::QProblemStatus&
     */
    qpOASES::QProblemStatus GetProblemStatus() const;

    /**
     * @brief Returns the primal solution of the most recent program, if
     * successful, otherwise returns the last successful primal solution.
     *
     * @return const Eigen::VectorXd&
     */
    const Eigen::VectorXd& getPrimalSolution();

   private:
    bool first_solve_ = true;
    int n_solves_ = 0;

    std::unique_ptr<qpOASES::SQProblem> qp_dense_;
    std::unique_ptr<qpOASES::SparseSolver> qp_sparse_;

    qpOASESSolverResults results_;
    qpOASESSolverOptions options_;
    qpOASESSolverInfo info_;

    QPOASESProgramData data_;
};

}  // namespace solvers
}  // namespace bopt

// #endif /* WITH_QPOASES */
#endif /* SOLVERS_QPOASES_H */
