#pragma once

#include <boost/functional/hash.hpp>

#include "bopt/Constraints.hpp"
#include "bopt/Costs.hpp"
#include "bopt/Logging.hpp"
#include "bopt/MathTypes.hpp"
#include "bopt/Profiler.hpp"
#include "bopt/solvers/SolverBase.hpp"

// COIN-related
#include <coin/ClpConfig.h>
#include <coin/CoinUtilsConfig.h>

#include <coin/ClpSimplex.hpp>

#include "coin/CoinBuild.hpp"
#include "coin/CoinHelperFunctions.hpp"
#include "coin/CoinModel.hpp"
#include "coin/CoinPackedVector.hpp"
#include "coin/CoinTime.hpp"

namespace bopt {
namespace solvers {

namespace internal {

struct ClpData {
    using VectorX = typename MathTypes<Real>::VectorX;
    using SparseMatrix = typename MathTypes<Real>::SparseMatrix;

    ClpData(const Index& nx, const Index& nc)
        : c(VectorX::Zero(nx)),
          xlb(VectorX::Constant(nx, -COIN_DBL_MAX)),
          xub(VectorX::Constant(nx, COIN_DBL_MAX)),
          Alb(VectorX::Zero(nc)),
          Aub(VectorX::Zero(nc)) {}
    /// @brief Cost coefficient vector
    VectorX c;
    /// @brief Variable lower bound
    VectorX xlb;
    /// @brief Variable upper bound
    VectorX xub;
    /// @brief Constraint lower bound
    VectorX Alb;
    /// @brief Constraint upper bound
    VectorX Aub;
    /// @brief Constraint matrix
    SparseMatrix A;
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
    using VectorX = typename MathTypes<Real>::VectorX;
    using MatrixX = typename MathTypes<Real>::MatrixX;
    using SparseMatrix = typename MathTypes<Real>::SparseMatrix;
    using SparseVector = typename MathTypes<Real>::SparseVector;

    ClpSolver() = default;
    ClpSolver(MathematicalProgram& program)
        : SolverBase(program, "ClpSolver"),
          model_(std::make_unique<ClpSimplex>()) {
        model_->resize(program.numConstraints(), program.numVariables());
        // Get the binding vectors
        dense_linear_costs_ = program.getCostBindings<LinearCostTpl<Real>>();
        sparse_linear_costs_ =
            program
                .getCostBindings<LinearCostTpl<Real, SparsityType::SPARSE>>();
        dense_linear_constraints_ =
            program.getConstraintBindings<LinearConstraintTpl<Real>>();
        sparse_linear_constraints_ = program.getConstraintBindings<
            LinearConstraintTpl<Real, SparsityType::SPARSE>>();
    }

    ~ClpSolver() {}

    const SolverResultsBase& getResults() const override { return results_; }

    const ClpInfo& getInfo() const { return info_; }

   protected:
    void reset();

    void initImpl() {
        data_ = std::make_unique<internal::ClpData>(
            getProgram().numVariables(), getProgram().numConstraints());

        Index c_idx = 0;
        std::vector<Eigen::Triplet<Real>> triplets;
        // Dense constraints
        for (auto& b : dense_linear_constraints_) {
            const auto& c = *b.get();
            const auto& indices = b.getIndexManager().getIndices();
            for (Index row = 0; row < c.outputSize(); ++row) {
                for (Index col = 0; col < c.dimInputTangentSpace(); ++col) {
                    triplets.push_back(
                        Eigen::Triplet<Real>(c_idx + row, indices[col]));
                }
            }
            c_idx += c.outputSize();
        }
        // Sparse constraints
        for (auto& b : sparse_linear_constraints_) {
            const auto& c = *b.get();
            auto& d = *b.getData();
            const auto& indices = b.getIndexManager().getIndices();
            for (int k = 0; k < d.A.outerSize(); ++k) {
                for (SparseMatrix::InnerIterator it(d.A, k); it; ++it) {
                    triplets.push_back(Eigen::Triplet<Real>(c_idx + it.row(),
                                                            indices[it.col()]));
                }
            }
            c_idx += c.outputSize();
        }
        data_->A.resize(getProgram().numConstraints(),
                        getProgram().numVariables());
        data_->A.setFromTriplets(triplets.begin(), triplets.end());
        data_->A.makeCompressed();

        // Assemble look-up map for indices
        for (Index k = 0; k < data_->A.outerSize(); ++k) {
            Index inner_nz_cnt = 0;
            for (SparseMatrix::InnerIterator it(data_->A, k); it; ++it) {
                A_nz_map_.insert(
                    {{it.row(), it.col()},
                     data_->A.outerIndexPtr()[it.outer()] + inner_nz_cnt++});
            }
        }

        results_ = SolverResultsBase();
        results_.primal = VectorX::Zero(getProgram().numVariables());
    }

    void solveImpl() {
        /** Variable Bounds **/
        {
            data_->xlb = getProgram().variableLowerBounds();
            data_->xub = getProgram().variableUpperBounds();

            for (const auto& binding :
                 getProgram().getBoundingBoxConstraintBindings()) {
                const auto& c = *binding.get();
                const auto& indices = binding.getIndexManager().getIndices();

                VectorX lb(c.outputSize()), ub(c.outputSize());
                c.evalBoundingBoxBounds(lb, ub);

                data_->xlb(indices).array() =
                    data_->xlb(indices).array().max(lb.array());

                data_->xub(indices).array() =
                    data_->xub(indices).array().min(ub.array());
            }
        }

        /** Linear costs **/
        {
            bopt::Profiler profiler("Clp: linear costs");
            // Reset cost gradient
            data_->c.setZero();
            Real constant = 0;
            // Dense costs
            for (auto& binding : dense_linear_costs_) {
                const auto& cost = binding.get();
                const auto& d = binding.getData();
                const auto& indices = binding.getIndexManager().getIndices();
                cost->evalCoefficients(*d);
                for (Index i = 0; i < d->a.size(); ++i) {
                    data_->c[indices[i]] += d->a[i];
                }
                constant += d->b;
            }
            // Sparse costs
            for (auto& binding : sparse_linear_costs_) {
                const auto& cost = binding.get();
                const auto& d = binding.getData();
                const auto& indices = binding.getIndexManager().getIndices();
                cost->evalCoefficients(*d);

                for (int k = 0; k < d->a.outerSize(); ++k) {
                    for (SparseVector::InnerIterator it(d->a, k); it; ++it) {
                        data_->c[indices[it.row()]] += it.value();
                    }
                }
                constant += d->b;
            }

            model_->setObjectiveOffset(-constant);
        }
        /** Linear constraints **/
        {
            bopt::Profiler profiler("Clp: linear constraints");

            Index c_idx = 0;
            // Dense constraints
            for (auto& binding : dense_linear_constraints_) {
                const auto& c = binding.get();
                const auto& d = binding.getData();
                const auto& indices = binding.getIndexManager().getIndices();

                const Index m = c->outputSize();

                c->evalCoefficients(*d);

                for (Index i = 0; i < d->A.rows(); ++i) {
                    for (Index j = 0; j < d->A.cols(); ++j) {
                        data_->A
                            .valuePtr()[A_nz_map_.at({c_idx + i, indices[j]})] =
                            d->A(i, j);
                    }
                }

                c->evalBounds(data_->Alb.middleRows(c_idx, m),
                              data_->Aub.middleRows(c_idx, m));

                data_->Alb.middleRows(c_idx, m) -= d->b;
                data_->Aub.middleRows(c_idx, m) -= d->b;

                c_idx += m;
            }

            // Sparse constraints
            for (auto& binding : sparse_linear_constraints_) {
                const auto& c = binding.get();
                auto& d = binding.getData();
                const auto& indices = binding.getIndexManager().getIndices();

                const Index m = c->outputSize();

                c->evalCoefficients(*d);

                for (int k = 0; k < d->A.outerSize(); ++k) {
                    for (SparseMatrix::InnerIterator it(d->A, k); it; ++it) {
                        data_->A.valuePtr()[A_nz_map_.at(
                            {c_idx + it.row(), indices[it.col()]})] =
                            it.value();
                    }
                }

                c->evalBounds(data_->Alb.middleRows(c_idx, m),
                              data_->Aub.middleRows(c_idx, m));

                data_->Alb.middleRows(c_idx, m) -= d->b;
                data_->Aub.middleRows(c_idx, m) -= d->b;

                c_idx += m;
            }
        }

        // Ensure bounds are within the CLP limits
        data_->xub.array() = data_->xub.array().cwiseMin(COIN_DBL_MAX);
        data_->xlb.array() = data_->xlb.array().cwiseMax(-COIN_DBL_MAX);
        data_->Aub.array() = data_->Aub.array().cwiseMin(COIN_DBL_MAX);
        data_->Alb.array() = data_->Alb.array().cwiseMax(-COIN_DBL_MAX);

        {
            Profiler profiler("Clp: solve");
            model_->loadProblem(
                data_->A.cols(), data_->A.rows(), data_->A.outerIndexPtr(),
                data_->A.innerIndexPtr(), data_->A.valuePtr(),
                data_->xlb.data(), data_->xub.data(), data_->c.data(),
                data_->Alb.data(), data_->Aub.data(), nullptr);

            model_->primal();
        }

        /* Objective */
        results_.objective = model_->getObjValue();
        /* Primal solution */
        const Real* solution = model_->primalColumnSolution();
        results_.primal = Eigen::Map<const VectorX>(
            solution, this->getProgram().numVariables());
    }

   private:
    ClpInfo info_;

    SolverResultsBase results_;

    std::vector<Binding<LinearCostTpl<Real>>> dense_linear_costs_;
    std::vector<Binding<LinearCostTpl<Real, SparsityType::SPARSE>>>
        sparse_linear_costs_;

    std::vector<Binding<LinearConstraintTpl<Real>>> dense_linear_constraints_;
    std::vector<Binding<LinearConstraintTpl<Real, SparsityType::SPARSE>>>
        sparse_linear_constraints_;

    std::unique_ptr<ClpSimplex> model_;
    std::unique_ptr<internal::ClpData> data_;

    typedef std::pair<int, int> SparseMatrixIndices;

    // Hash for pairs of ints (to allow hashtable for (x,y) indices in sparse
    // matrices)
    struct hash_pair {
        std::size_t operator()(const SparseMatrixIndices& p) const {
            std::size_t seed = 0;
            boost::hash_combine(seed, p.first);
            boost::hash_combine(seed, p.second);
            return seed;
        }
    };

    /// @brief Lookup map from the non-zero entry (x, y) to its index in the
    /// nonzero vector
    std::unordered_map<SparseMatrixIndices, int, hash_pair> A_nz_map_;
};

}  // namespace solvers
}  // namespace bopt
