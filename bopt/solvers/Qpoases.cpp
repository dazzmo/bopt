#include "bopt/solvers/qpoases.hpp"

namespace bopt {
namespace solvers {

qpoases_solver::qpoases_solver(MathematicalProgram& program)
    : solver(program), data_(program) {
    LOG(INFO) << "qpoases_solver::qpoases_solver";

    // Create problem
    int nx = program.numVariables();
    int ng = program.numConstraints();

    qp_ = std::make_unique<qpOASES::SQProblem>(nx, ng);

    // Store bindings of each set of constraints relevant to the problem
    dense_linear_costs_ = program.getCosts<DenseLinearCostTpl<Real>>();
    sparse_linear_costs_ = program.getCosts<SparseLinearCostTpl<Real>>();
    sparse_linear_costs_ = program.getCosts<SparseLinearCostTpl<Real>>();

    // std::vector<Binding<SparseLinearConstraintTpl<double>>> sparse_lin_con_;

    // Iterate through all constraints and evaluate them
}

qpoases_solver::~qpoases_solver() = default;

void qpoases_solver::solve(MathematicalProgram& program) {
    data_.clear();
    /** Bounding box constraints **/
    {
        // Variable bounds
        data_.lbx = program.variableLowerBounds();
        data_.ubx = program.variableUpperBounds();

        for (auto& binding : program.getBoundingBoxConstraints()) {
            const auto& c = *binding.get();
            auto& d = *binding.data();
            const auto& indices = binding.indices().indices();
            c.evalBounds(d);

            data_.lbx(indices).array() =
                data_.lbx(indices).array().max(d.lb.array());

            data_.ubx(indices).array() =
                data_.ubx(indices).array().min(d.ub.array());
        }
    }

    /** Linear costs **/
    {
        VLOG(10) << "qpoases:linear costs";
        bopt::profiler profiler("qpoases: linear costs");
        // Dense costs
        for (auto& binding : dense_linear_costs_) {
            const auto& c = binding.get();
            const auto& d = binding.data();
            const auto& indices = binding.indices().indices();
            c->evalCoefficients(*d);
            data_.g(indices) += d->a;
        }
        // Sparse costs
        for (auto& binding : sparse_linear_costs_) {
            const auto& c = binding.get();
            const auto& d = binding.data();
            const auto& indices = binding.indices().indices();
            c->evalCoefficients(*d);

            for (int k = 0; k < d->a.outerSize(); ++k) {
                for (SparseEvaluatorTraits<Real>::OutputVector::InnerIterator it(
                         d->a, k);
                     it; ++it) {
                    data_.g(indices[it.row()]) += it.value();
                }
            }
        }
    }

    /** Quadratic costs **/
    {
        VLOG(10) << "qpoases:quadratic costs";
        bopt::profiler profiler("qpoases: quadratic costs");
        // Dense costs
        for (auto& binding : dense_quadratic_costs_) {
            const auto& c = binding.get();
            const auto& d = binding.data();
            const auto& indices = binding.indices().indices();
            c->evalCoefficients(*d);

            // A
            data_.H(indices, indices) += d->A;
            // b
            data_.g(indices) += d->b;
        }
        // Sparse costs
        for (auto& binding : sparse_quadratic_costs_) {
            const auto& c = binding.get();
            const auto& d = binding.data();
            const auto& indices = binding.indices().indices();
            c->evalCoefficients(*d);

            // A
            for (int k = 0; k < d->A.outerSize(); ++k) {
                for (SparseMatrix<double>::InnerIterator it(d->A, k); it;
                     ++it) {
                    data_.H(indices[it.row()], indices[it.col()]) += it.value();
                }
            }
            // b
            for (int k = 0; k < d->b.outerSize(); ++k) {
                for (SparseVector<double>::InnerIterator it(d->b, k); it;
                     ++it) {
                    data_.g(indices[it.row()]) += it.value();
                }
            }
        }
    }

    /** Linear constraints **/
    {
        VLOG(10) << "linear constraints";
        bopt::profiler profiler("qpoases: linear constraints");

        Index c_idx = 0;

        // Dense constraints
        for (auto& binding : dense_linear_constraints_) {
            const auto& c = binding.get();
            const auto& d = binding.data();
            const auto& indices = binding.indices().indices();

            const Index m = c->numOutputs();

            c->evalCoefficients(*d);
            c->evalBounds(*d);

            data_.A.middleRows(c_idx, m) = d->A;
            data_.lbA.middleRows(c_idx, m) = d->lb;
            data_.ubA.middleRows(c_idx, m) = d->ub;
            c_idx += m;
        }

        // Dense constraints
        for (auto& binding : sparse_linear_constraints_) {
            const auto& c = binding.get();
            const auto& d = binding.data();
            const auto& indices = binding.indices().indices();

            const Index m = c->numOutputs();

            c->evalCoefficients(*d);
            c->evalBounds(*d);

            for (int k = 0; k < d->A.outerSize(); ++k) {
                for (SparseMatrix<double>::InnerIterator it(d->A, k); it;
                     ++it) {
                    data_.A(c_idx + it.row(), indices[it.col()]) = it.value();
                }
            }
            data_.lbA.middleRows(c_idx, m) = d->lb;
            data_.ubA.middleRows(c_idx, m) = d->ub;
            c_idx += m;
        }
    }

    int nWSR = options_.nWSR;

    qp_->setHessianType(qpOASES::HessianType::HST_POSDEF);
    // todo - set this only once?
    qp_->setOptions(options_);

    VLOG(10) << "H: " << data_.H;
    VLOG(10) << "g: " << data_.g;
    VLOG(10) << "A: " << data_.A;
    VLOG(10) << "lbA: " << data_.lbA;
    VLOG(10) << "ubA: " << data_.ubA;
    VLOG(10) << "lbx: " << data_.lbx;
    VLOG(10) << "ubx: " << data_.ubx;

    // Solve
    if (info_.number_of_solves > 0 && options_.perform_hotstart) {
        bopt::profiler profiler("qpoases: solve");
        // Use previous solution to hot-start the program
        // qpOASES::SymDenseMat(nx, nx, 0, data.H.data());
        qp_->hotstart(data_.H.data(), data_.g.data(), data_.A.data(),
                      data_.lbx.data(), data_.ubx.data(), data_.lbA.data(),
                      data_.ubA.data(), nWSR);
    } else {
        bopt::profiler profiler("qpoases: solve");
        // Initialise the program and solve it
        qp_->init(data_.H.data(), data_.g.data(), data_.A.data(),
                  data_.lbx.data(), data_.ubx.data(), data_.lbA.data(),
                  data_.ubA.data(), nWSR);
    }

    // Collect information
    info_.nWSR = nWSR;
    info_.status = qp_->getStatus();
    // info_.iterations = ;  // !

    info_.number_of_solves++;

    // Get results
    if (info_.status == qpOASES::QProblemStatus::QPS_SOLVED) {
        info_.success = true;
        qp_->getPrimalSolution(this->primal_solution().data());
        VLOG(10) << "primal_solution: " << this->primal_solution().transpose();
    }
};

void qpoases_solver::reset() { info_.number_of_solves = 0; }

}  // namespace solvers
}  // namespace bopt