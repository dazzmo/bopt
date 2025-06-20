#include "bopt/solvers/Qpoases.hpp"

namespace bopt {
namespace solvers {

QpoasesSolver::QpoasesSolver(MathematicalProgram& program)
    : SolverBase(program, "QPOASES Solver") {}

QpoasesSolver::~QpoasesSolver() = default;

void QpoasesSolver::initImpl() {
    // Create problem
    int nx = getProgram().numVariables();
    int ng = getProgram().numConstraints();

    // Store bindings of each set of constraints relevant to the problem
    dense_linear_costs_ = getProgram().getCostBindings<LinearCostTpl<Real>>();
    sparse_linear_costs_ =
        getProgram()
            .getCostBindings<LinearCostTpl<Real, SparsityType::SPARSE>>();

    dense_quadratic_costs_ =
        getProgram().getCostBindings<QuadraticCostTpl<Real>>();
    sparse_quadratic_costs_ =
        getProgram()
            .getCostBindings<QuadraticCostTpl<Real, SparsityType::SPARSE>>();

    dense_linear_constraints_ =
        getProgram().getConstraintBindings<LinearConstraintTpl<Real>>();
    sparse_linear_constraints_ =
        getProgram()
            .getConstraintBindings<
                LinearConstraintTpl<Real, SparsityType::SPARSE>>();

    qp_ = std::make_unique<qpOASES::SQProblem>(nx, ng);
    data_ = std::make_unique<internal::QpoasesData>(
        getProgram().numVariables(), getProgram().numConstraints());

    qp_->setHessianType(qpOASES::HessianType::HST_POSDEF);
    qp_->setOptions(options_);
}

void QpoasesSolver::solveImpl() {
    data_->clear();
    /** Bounding box constraints **/
    {
        // Variable bounds
        data_->xlb = getProgram().variableLowerBounds();
        data_->xub = getProgram().variableUpperBounds();

        for (auto& binding : getProgram().getBoundingBoxConstraintBindings()) {
            const auto& c = *binding.get();
            auto& d = *binding.getData();
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
        bopt::Profiler profiler("qpoases: linear costs");
        // Dense costs
        for (auto& binding : dense_linear_costs_) {
            const auto& c = binding.get();
            const auto& d = binding.getData();
            const auto& indices = binding.getIndexManager().getIndices();
            c->evalCoefficients(*d);
            data_->g(indices) += d->a;
            data_->c += d->b;
        }
        // Sparse costs
        for (auto& binding : sparse_linear_costs_) {
            const auto& c = binding.get();
            const auto& d = binding.getData();
            const auto& indices = binding.getIndexManager().getIndices();
            c->evalCoefficients(*d);

            for (int k = 0; k < d->a.outerSize(); ++k) {
                for (SparseVector::InnerIterator it(d->a, k); it; ++it) {
                    data_->g(indices[it.row()]) += it.value();
                }
            }
            data_->c += d->b;
        }
    }

    /** Quadratic costs **/
    {
        bopt::Profiler profiler("qpoases: quadratic costs");
        // Dense costs
        for (auto& binding : dense_quadratic_costs_) {
            const auto& c = binding.get();
            const auto& d = binding.getData();
            const auto& indices = binding.getIndexManager().getIndices();
            c->evalCoefficients(*d);

            // A
            data_->H(indices, indices) += d->A;
            // b
            data_->g(indices) += d->b;
            // c
            data_->c += d->c;
        }
        // Sparse costs
        for (auto& binding : sparse_quadratic_costs_) {
            const auto& c = binding.get();
            const auto& d = binding.getData();
            const auto& indices = binding.getIndexManager().getIndices();
            c->evalCoefficients(*d);

            // A
            for (int k = 0; k < d->A.outerSize(); ++k) {
                for (SparseMatrix::InnerIterator it(d->A, k); it; ++it) {
                    data_->H(indices[it.row()], indices[it.col()]) +=
                        it.value();
                }
            }
            // b
            for (int k = 0; k < d->b.outerSize(); ++k) {
                for (SparseVector::InnerIterator it(d->b, k); it; ++it) {
                    data_->g(indices[it.row()]) += it.value();
                }
            }
            // c
            data_->c += d->c;
        }
    }

    /** Linear constraints **/
    {
        bopt::Profiler profiler("qpoases: linear constraints");

        Index c_idx = 0;

        // Dense constraints
        for (auto& binding : dense_linear_constraints_) {
            const auto& c = binding.get();
            const auto& d = binding.getData();
            const auto& indices = binding.getIndexManager().getIndices();

            const Index m = c->outputSize();

            c->evalCoefficients(*d);
            data_->A.middleRows(c_idx, m) = d->A;

            c->evalBounds(data_->Alb.middleRows(c_idx, m),
                          data_->Aub.middleRows(c_idx, m));
            data_->Alb.middleRows(c_idx, m) -= d->b;
            data_->Aub.middleRows(c_idx, m) -= d->b;
            c_idx += m;
        }

        // Dense constraints
        for (auto& binding : sparse_linear_constraints_) {
            const auto& c = binding.get();
            const auto& d = binding.getData();
            const auto& indices = binding.getIndexManager().getIndices();

            const Index m = c->outputSize();

            c->evalCoefficients(*d);

            for (int k = 0; k < d->A.outerSize(); ++k) {
                for (SparseMatrix::InnerIterator it(d->A, k); it; ++it) {
                    data_->A(c_idx + it.row(), indices[it.col()]) = it.value();
                }
            }

            c->evalBounds(data_->Alb.middleRows(c_idx, m),
                          data_->Aub.middleRows(c_idx, m));

            data_->Alb.middleRows(c_idx, m) -= d->b;
            data_->Aub.middleRows(c_idx, m) -= d->b;

            c_idx += m;
        }
    }

    // Solve
    {
        bopt::Profiler profiler("qpoases: solve");
        if (info_.num_iterations > 0 && hotstarting_) {
            // Use previous solution to hot-start the program
            qp_->hotstart(data_->H.data(), data_->g.data(), data_->A.data(),
                          data_->xlb.data(), data_->xub.data(),
                          data_->Alb.data(), data_->Aub.data(), nWSR_);
        } else {
            // Initialise the program and solve it
            qp_->init(data_->H.data(), data_->g.data(), data_->A.data(),
                      data_->xlb.data(), data_->xub.data(), data_->Alb.data(),
                      data_->Aub.data(), nWSR_);
        }
    }

    // Collect information
    info_.nWSR = nWSR_;
    info_.status = qp_->getStatus();

    info_.num_iterations++;

    // Get results
    if (info_.status == qpOASES::QProblemStatus::QPS_SOLVED) {
        info_.success = true;

        results_.objective = qp_->getObjVal() + data_->c;
        results_.primal = VectorX::Zero(getProgram().numVariables());
        qp_->getPrimalSolution(results_.primal.data());
    }
};

}  // namespace solvers
}  // namespace bopt