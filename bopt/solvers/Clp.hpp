#pragma once

#include <coin/ClpConfig.h>
#include <coin/CoinUtilsConfig.h>

#include <coin/ClpSimplex.hpp>

#include "bopt/logging.hpp"
#include "bopt/profiler.hpp"
#include "bopt/program.hpp"
#include "coin/CoinBuild.hpp"
#include "coin/CoinHelperFunctions.hpp"
#include "coin/CoinModel.hpp"
#include "coin/CoinTime.hpp"

namespace bopt {
namespace solvers {

/**
 * @brief Details for the Clp solver
 *
 */
struct ClpInfo {
    Index number_of_solves = 0;
};

class ClpSolver {
   public:
    ClpSolver() = default;
    ClpSolver(MathematicalProgram& program)
        : model_(std::make_unique<ClpSimplex>()) {
        model_->resize(program.numConstraints(), program.numVariables());
        // Get the binding vectors
        dense_linear_costs_ = program.getCosts<DenseLinearCostTpl<Real>>();
        dense_linear_constraints_ =
            program.getConstraints<DenseLinearConstraintTpl<Real>>();
    }

    ~ClpSolver() {}

    void reset();
    void solve(MathematicalProgram& program) {
        Eigen::VectorXd c(program.numVariables()), xlb(program.numVariables()),
            xub(program.numVariables()), Alb(program.numConstraints()),
            Aub(program.numConstraints());
        c.setZero();
        xlb.setConstant(COIN_DBL_MIN);
        xub.setConstant(COIN_DBL_MAX);
        Alb.setZero();
        Aub.setZero();

        {
            VLOG(10) << "Clp:linear costs";
            bopt::profiler profiler("Clp: linear costs");
            // Dense costs
            for (auto& binding : dense_linear_costs_) {
                const auto& cost = binding.get();
                const auto& d = binding.data();
                const auto& indices = binding.indices().indices();
                cost->evalCoefficients(*d);
                for (Index i = 0; i < d->a.size(); ++i) {
                    c[indices[i]] += d->a[i];
                }
            }
            // Sparse costs
            for (auto& binding : sparse_linear_costs_) {
                const auto& cost = binding.get();
                const auto& d = binding.data();
                const auto& indices = binding.indices().indices();
                cost->evalCoefficients(*d);

                for (int k = 0; k < d->a.outerSize(); ++k) {
                    for (SparseFunctionTraits<Real>::OutputVector::InnerIterator
                             it(d->a, k);
                         it; ++it) {
                        c[indices[it.row()]] += it.value();
                    }
                }
            }
        }
        // Access all linear constraints and add each as a row
        Index nx = program.numVariables();
        Index nc = program.numConstraints();
        CoinPackedMatrix* matrix = new CoinPackedMatrix(false, 0, 0);
        matrix->setDimensions(0, nx);
        /** Linear constraints **/
        {
            VLOG(10) << "Clp:linear constraints";
            bopt::profiler profiler("Clp: linear constraints");

            Index idx = 0;

            // Dense constraints
            for (auto& binding : dense_linear_constraints_) {
                const auto& c = binding.get();
                const auto& d = binding.data();
                const auto& indices = binding.indices().indices();

                const Index m = c->getOutputDimension();

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
                Alb.middleRows(idx, m) = d->lb;
                Aub.middleRows(idx, m) = d->ub;
                idx += m;
            }

            // // Sparse constraints
            // for (auto& binding : sparse_linear_constraints_) {
            //     const auto& c = binding.get();
            //     const auto& d = binding.data();
            //     const auto& indices = binding.indices().indices();
            //     c->evalCoefficients(*d);
            //     c->evalBounds(*d);

            //     for (int k = 0; k < d->A.outerSize(); ++k) {
            //         CoinIndexedVector row;
            //         for (SparseFunctionTraits<Real>::OutputVector::InnerIterator
            //                  it(d->A, k);
            //              it; ++it) {
            //             row.insert(indices[it.row()], it.value());
            //         }
            //     }
            //     matrix->appendRow(row.getNumElements(), row.getIndices(),
            //                       row.denseVector());
            //     Alb.middleRows(idx, m) = d->lb;
            //     Aub.middleRows(idx, m) = d->ub;
            //     idx += m;
            // }
        }

        std::cout << matrix->getNumElements() << std::endl;

        model_->loadProblem(*matrix, xlb.data(), xub.data(), c.data(),
                            Alb.data(), Aub.data());
        std::cout << model_->getNumElements() << std::endl;
        model_->primal();
        const Real* solution = model_->primalColumnSolution();
        for (int i = 0; i < nx; ++i)
            std::cout << "x[" << i << "] = " << solution[i] << std::endl;
        delete matrix;
    }

   private:
    bool first_solve_ = true;
    int n_solves_ = 0;

    std::vector<Binding<DenseLinearCostTpl<Real>>> dense_linear_costs_;
    std::vector<Binding<SparseLinearCostTpl<Real>>> sparse_linear_costs_;

    std::vector<Binding<DenseLinearConstraintTpl<Real>>>
        dense_linear_constraints_;
    std::vector<Binding<SparseLinearConstraintTpl<Real>>>
        sparse_linear_constraints_;

    std::unique_ptr<ClpSimplex> model_;
};

}  // namespace solvers
}  // namespace bopt
