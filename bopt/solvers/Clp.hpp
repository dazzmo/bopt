#pragma once

#include "bopt/Logging.hpp"
#include "bopt/Profiler.hpp"
#include "bopt/solvers/SolverBase.hpp"

// COIN-related
#include <coin/ClpConfig.h>
#include <coin/CoinUtilsConfig.h>

#include <coin/ClpSimplex.hpp>

#include "coin/CoinBuild.hpp"
#include "coin/CoinHelperFunctions.hpp"
#include "coin/CoinModel.hpp"
#include "coin/CoinTime.hpp"

namespace bopt {
namespace solvers {

namespace internal {

struct ClpData {
    using VectorX = typename MathTypes<Real>::VectorX;
    ClpData(const Index& nx, const Index& nc)
        : c(VectorX::Zero(nx)),
          xlb(VectorX::Zero(nx)),
          xub(VectorX::Zero(nx)),
          Alb(VectorX::Zero(nc)),
          Aub(VectorX::Zero(nc)) {}
    /// @brief Cost coefficient vector
    VectorX c;
    /// @brief Variable lower bound
    VectorX xlb;
    /// @brief Variable upper bound
    VectorX xub;
    /// @brief Constriant lower bound
    VectorX Alb;
    /// @brief Constriant upper bound
    VectorX Aub;
};

}  // namespace internal

/**
 * @brief Details for the Clp solver
 *
 */
struct ClpInfo {
    Index number_of_solves = 0;
};

class ClpSolver : public SolverBase<ClpInfo> {
   public:
    using VectorX = typename SolverBase<ClpInfo>::VectorX;

    ClpSolver() = default;
    ClpSolver(MathematicalProgram& program)
        : SolverBase(program, "ClpSolver"),
          model_(std::make_unique<ClpSimplex>()) {
        model_->resize(program.numConstraints(), program.numVariables());
        // Get the binding vectors
        dense_linear_costs_ = program.getCosts<DenseLinearCostTpl<Real>>();
        dense_linear_constraints_ =
            program.getConstraints<DenseLinearConstraintTpl<Real>>();
    }

    ~ClpSolver() {}

    VectorX getPrimalSolution() const {
        model_->primal();
        const Real* solution = model_->primalColumnSolution();
        VectorX x = Eigen::Map<const VectorX>(
            solution, this->getProgram().numVariables());
        return x;
    }

   protected:
    void reset();

    void initImpl() {
        data_ = std::make_unique<internal::ClpData>(
            getProgram().numVariables(), getProgram().numConstraints());
    }

    void solveImpl() {
        /** Linear costs **/
        {
            bopt::Profiler profiler("Clp: linear costs");
            // Dense costs
            for (auto& binding : dense_linear_costs_) {
                const auto& cost = binding.get();
                const auto& d = binding.getData();
                const auto& indices = binding.getIndexManager().getIndices();
                cost->evalCoefficients(*d);
                for (Index i = 0; i < d->a.size(); ++i) {
                    data_->c[indices[i]] += d->a[i];
                }
            }
            // Sparse costs
            for (auto& binding : sparse_linear_costs_) {
                const auto& cost = binding.get();
                const auto& d = binding.getData();
                const auto& indices = binding.getIndexManager().getIndices();
                cost->evalCoefficients(*d);

                for (int k = 0; k < d->a.outerSize(); ++k) {
                    for (SparseEvaluatorTraits<Real>::VectorType::InnerIterator
                             it(d->a, k);
                         it; ++it) {
                        data_->c[indices[it.row()]] += it.value();
                    }
                }
            }
        }
        // Access all linear constraints and add each as a row
        Index nx = getProgram().numVariables();
        Index nc = getProgram().numConstraints();
        // Create matrix
        CoinPackedMatrix* matrix = new CoinPackedMatrix(false, 0, 0);
        matrix->setDimensions(0, nx);
        /** Linear constraints **/
        {
            bopt::Profiler profiler("Clp: linear constraints");

            Index c_idx = 0;

            // Dense constraints
            for (auto& binding : dense_linear_constraints_) {
                const auto& c = binding.get();
                const auto& d = binding.getData();
                const auto& indices = binding.getIndexManager().getIndices();

                const Index m = c->numOutputs();

                c->evalCoefficients(*d);
                c->evalBounds(*d);

                for (Index i = 0; i < d->A.rows(); ++i) {
                    CoinIndexedVector row;
                    for (Index j = 0; j < d->A.cols(); ++j) {
                        row.insert(indices[j], d->A(i, j));
                    }
                    matrix->appendRow(row.getNumElements(), row.getIndices(),
                                      row.denseVector());
                }
                data_->Alb.middleRows(c_idx, m) = d->lb;
                data_->Aub.middleRows(c_idx, m) = d->ub;
                c_idx += m;
            }

            // Sparse constraints
            for (auto& binding : sparse_linear_constraints_) {
                const auto& c = binding.get();
                auto& d = binding.getData();
                const auto& indices = binding.getIndexManager().getIndices();

                const Index m = c->numOutputs();

                c->evalCoefficients(*d);
                c->evalBounds(*d);

                CoinIndexedVector row;
                for (int k = 0; k < d->A.outerSize(); ++k) {
                    for (SparseEvaluatorTraits<Real>::MatrixType::InnerIterator
                             it(d->A, k);
                         it; ++it) {
                        row.insert(indices[it.col()], it.value());
                    }
                }
                matrix->appendRow(row.getNumElements(), row.getIndices(),
                                  row.denseVector());
                data_->Alb.middleRows(c_idx, m) = d->lb;
                data_->Aub.middleRows(c_idx, m) = d->ub;
                c_idx += m;
            }
        }

        model_->loadProblem(*matrix, data_->xlb.data(), data_->xub.data(),
                            data_->c.data(), data_->Alb.data(),
                            data_->Aub.data());
        // Delete matrix after using it
        delete matrix;
    }

   private:
    std::vector<Binding<DenseLinearCostTpl<Real>>> dense_linear_costs_;
    std::vector<Binding<SparseLinearCostTpl<Real>>> sparse_linear_costs_;

    std::vector<Binding<DenseLinearConstraintTpl<Real>>>
        dense_linear_constraints_;
    std::vector<Binding<SparseLinearConstraintTpl<Real>>>
        sparse_linear_constraints_;

    std::unique_ptr<ClpSimplex> model_;
    std::unique_ptr<internal::ClpData> data_;
};

}  // namespace solvers
}  // namespace bopt
