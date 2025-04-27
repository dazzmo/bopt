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

struct QPData {
    typedef Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        MatrixXdRowMajor;

    QPData(const MathematicalProgram& program) {
        // Create matrix data
        H.resize(program.numVariables(), program.numVariables());
        H.setZero();

        g.resize(program.numVariables());
        g.setZero();

        A.resize(program.numConstraints(), program.numVariables());
        A.setZero();

        lbA.resize(program.numConstraints());
        ubA.resize(program.numConstraints());

        lbx.resize(program.numVariables());
        ubx.resize(program.numVariables());

        lbx = program.variableLowerBounds();
        ubx = program.variableUpperBounds();
    }

    void clear() {
        H.setZero();
        g.setZero();
        A.setZero();
        ubA.setZero();
        lbA.setZero();
    }

    MatrixXdRowMajor H;
    Eigen::VectorXd g;

    MatrixXdRowMajor A;

    Eigen::VectorXd ubA;
    Eigen::VectorXd lbA;

    Eigen::VectorXd lbx;
    Eigen::VectorXd ubx;
};

class qpoases_solver : public solver<double> {
   public:
    qpoases_solver() = default;
    qpoases_solver(MathematicalProgram& program);

    ~qpoases_solver();

    qpoases_options& options() { return options_; }

    void reset();
    void solve(MathematicalProgram& program);

   private:
    bool first_solve_ = true;
    int n_solves_ = 0;

    std::vector<Binding<DenseLinearCost>> dense_linear_costs_;
    std::vector<Binding<SparseLinearCost>> sparse_linear_costs_;

    std::vector<Binding<DenseQuadraticCost>> dense_quadratic_costs_;
    std::vector<Binding<SparseQuadraticCost>> sparse_quadratic_costs_;

    std::vector<Binding<DenseLinearConstraint>> dense_linear_constraints_;
    std::vector<Binding<SparseLinearConstraint>> sparse_linear_constraints_;

    // std::vector<LinearCostData> linear_cost_data_;
    // std::vector<ConstraintData> bounding_box_constraint_data_;

    std::unique_ptr<qpOASES::SQProblem> qp_;

    qpoases_options options_;
    qpoases_info info_;

    QPData data_;
};

}  // namespace solvers
}  // namespace bopt

// #endif /* WITH_QPOASES */
#endif /* SOLVERS_QPOASES_H */
