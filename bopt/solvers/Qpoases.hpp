#pragma once

#include <boost/numeric/ublas/matrix.hpp>
#include <qpOASES.hpp>

#include "bopt/Logging.hpp"
#include "bopt/Profiler.hpp"
#include "bopt/Program.hpp"
#include "bopt/solvers/SolverBase.hpp"

namespace bopt {
namespace solvers {

namespace internal {

/**
 * @brief Details for the qpOASES solver
 *
 */
struct QpoasesSolverInfo : public SolverInfoBase {
    qpOASES::QProblemStatus status;
    // Return status for the qpOASES solver
    int returnStatus;
    // Error code for the qpOASES solver
    int errorCode;
    // Number of working sets performed
    int nWSR;
};

struct QpoasesData {
    using VectorX = typename MathTypes<Real>::VectorX;
    using MatrixX = typename MathTypes<Real>::MatrixX;
    using MatrixXRM =
        Eigen::Matrix<Real, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

    QpoasesData(const Index& nx, const Index& nc)
        : H(MatrixXRM::Zero(nx, nx)),
          g(VectorX::Zero(nx)),
          c(0),
          xlb(VectorX::Constant(nx, -1e9)),
          xub(VectorX::Constant(nx, 1e9)),
          A(MatrixXRM::Zero(nc, nx)),
          Alb(VectorX::Zero(nc)),
          Aub(VectorX::Zero(nc)) {}

    /// @brief Quadratic cost hessian component
    MatrixXRM H;
    /// @brief Quadratic cost linear component
    VectorX g;
    /// @brief Quadratic cost linear component
    Real c;

    /// @brief Variable lower bound
    VectorX xlb;
    /// @brief Variable upper bound
    VectorX xub;

    /// @brief Constraint matrix
    MatrixXRM A;
    /// @brief Constraint lower bound
    VectorX Alb;
    /// @brief Constraint upper bound
    VectorX Aub;

    void clear() {
        H.setZero();
        g.setZero();
        A.setZero();
    }
};

}  // namespace internal

class QpoasesSolver : public SolverBase<internal::QpoasesSolverInfo> {
    using VectorX = typename MathTypes<Real>::VectorX;
    using MatrixX = typename MathTypes<Real>::MatrixX;
    using SparseMatrix = typename MathTypes<Real>::SparseMatrix;
    using SparseVector = typename MathTypes<Real>::SparseVector;

   public:
    QpoasesSolver() = default;
    QpoasesSolver(MathematicalProgram& program);

    ~QpoasesSolver();

    void initImpl() override;
    void solveImpl() override;

    const internal::QpoasesSolverInfo& getInfo() const override {
        return info_;
    }

    const SolverResultsBase& getResults() const override { return results_; }

    struct QpoasesOptions : public qpOASES::Options {
        // Number of working sets performed
        int nWSR = 100;
        bool perform_hotstart = false;
    };

    qpOASES::Options& getOptions() { return options_; }

    void setNumberOfWorkingSetRecalculations(const int& nWSR) { nWSR_ = nWSR; }
    void enableHotStarting() { hotstarting_ = true; }
    void disableHotStarting() { hotstarting_ = false; }

   private:
    std::unique_ptr<qpOASES::SQProblem> qp_;

    std::vector<Binding<LinearCostTpl<Real>>> dense_linear_costs_;
    std::vector<Binding<LinearCostTpl<Real, SparsityType::SPARSE>>>
        sparse_linear_costs_;

    std::vector<Binding<QuadraticCostTpl<Real>>> dense_quadratic_costs_;
    std::vector<Binding<QuadraticCostTpl<Real, SparsityType::SPARSE>>>
        sparse_quadratic_costs_;

    std::vector<Binding<LinearConstraintTpl<Real>>> dense_linear_constraints_;
    std::vector<Binding<LinearConstraintTpl<Real, SparsityType::SPARSE>>>
        sparse_linear_constraints_;

    internal::QpoasesSolverInfo info_;
    qpOASES::Options options_;
    std::unique_ptr<internal::QpoasesData> data_;
    SolverResultsBase results_;

    int nWSR_{100};
    bool hotstarting_{false};
};

}  // namespace solvers
}  // namespace bopt
