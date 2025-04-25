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
    // {
    //     bopt::profiler profiler("qpoases: bounding box constraints");
    //     int i = 0;
    //     for (auto& binding : program.boundingBoxConstraints()) {
    //         ConstraintData& cdata = bounding_box_constraint_data_[i];
    //         // todo - manage between program constraint and bounding box
    //         binding.get()->evalBounds(cdata);
    //         data.lbx(binding.indices().indices()) << cdata.lb;
    //         data.ubx(binding.indices().indices()) << cdata.ub;
    //     }
    // }

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
                for (SparseFunctionTraits<Real>::OutputVector::InnerIterator it(
                         d->a, k);
                     it; ++it) {
                    data_.g(indices[it.row()]) += it.value();
                }
            }
        }
    }

    /** Quadratic costs **/
    // {
    //     bopt::profiler profiler("qpoases: quadratic costs");
    //     VLOG(10) << "qpoases:quadratic costs";
    //     int i = 0;
    //     for (auto& binding : program.quadraticCosts()) {
    //         auto& c = *binding.get();
    //         const auto& indices = binding.indices().indices();

    //         // Create vector
    //         QuadraticCostData& cdata = quadratic_cost_data_[i];
    //         if (cdata.A_s.nonZeros()) {
    //             c.evalSparseCoefficients(cdata);
    //             // A
    //             for (int k = 0; k < cdata.A_s.outerSize(); ++k) {
    //                 for (SparseMatrix<double>::InnerIterator it(cdata.A_s,
    //                 k);
    //                      it; ++it) {
    //                     data.H(indices[it.row()], indices[it.col()]) +=
    //                         it.value();
    //                 }
    //             }
    //             // b
    //             for (int k = 0; k < cdata.b_s.outerSize(); ++k) {
    //                 for (SparseVector<double>::InnerIterator it(cdata.b_s,
    //                 k);
    //                      it; ++it) {
    //                     data.g(indices[it.row()]) += it.value();
    //                 }
    //             }
    //         } else {
    //             c.evalCoefficients(cdata);
    //             data.H(indices, indices) += cdata.A;
    //             data.g(indices) += cdata.b;
    //         }
    //         i++;
    //     }
    // }

    // /** Linear constraints **/
    // {
    //     bopt::profiler profiler("qpoases: linear constraints");

    //     VLOG(10) << "qpoases:linear constraints";
    //     int row = 0;
    //     int i = 0;
    //     for (auto& binding : program.linearConstraints()) {
    //         auto& c = *binding.get();
    //         const auto& indices = binding.indices().indices();

    //         // Create vector
    //         LinearConstraintData& cdata = linear_constraint_data_[i];
    //         // todo - if sparse
    //         if (cdata.A_s.nonZeros()) {
    //             c.evalSparseCoefficients(cdata);
    //             // A
    //             for (int k = 0; k < cdata.A_s.outerSize(); ++k) {
    //                 for (SparseMatrix<double>::InnerIterator it(cdata.A_s,
    //                 k);
    //                      it; ++it) {
    //                     data.A(row + it.row(), indices[it.col()]) =
    //                     it.value();
    //                 }
    //             }
    //         } else {
    //             c.evalCoefficients(cdata);
    //             data.A.middleRows(row, c.getOutputDimension()) = cdata.A;
    //         }

    //         // Evaluate bounds
    //         c.evalBounds(cdata);
    //         data.lbA.middleRows(row, c.getOutputDimension()) = cdata.lb;
    //         data.ubA.middleRows(row, c.getOutputDimension()) = cdata.ub;

    //         // Increment
    //         row += c.getOutputDimension();
    //         i++;
    //     }
    // }

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