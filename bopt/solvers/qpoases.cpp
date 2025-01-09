#include "bopt/solvers/qpoases.hpp"

#include "boost/numeric/ublas/io.hpp"
#include "bopt/sparse.hpp"

namespace bopt {
namespace solvers {

qpoases_solver_instance::qpoases_solver_instance(
    mathematical_program<double>& program)
    : solver(program) {
    LOG(INFO) << "qpoases_solver_instance::qpoases_solver_instance";

    // Create problem
    int nx = program.n_variables();
    int ng = program.n_constraints();

    qp_ = std::make_unique<qpOASES::SQProblem>(nx, ng);

    // Create matrix data
    data.H.resize(nx, nx);
    data.H.clear();

    data.g.resize(nx);

    data.A.resize(ng, nx);
    data.A.clear();

    data.lbA.resize(ng);
    data.ubA.resize(ng);

    data.lbx.resize(nx);
    data.ubx.resize(nx);

    for (int i = 0; i < nx; ++i) {
        set(data.lbx, i, program.variable_bounds()[i].lower);
        set(data.ubx, i, program.variable_bounds()[i].upper);
    }
}

qpoases_solver_instance::~qpoases_solver_instance() = default;

void qpoases_solver_instance::solve() {
    Eigen::MatrixXd tmp;

    /** Linear costs **/
    VLOG(10) << "qpoases:linear costs";
    for (const auto& binding : program().linear_costs()) {
        const auto& x_indices = binding.input_indices.indices;
        if (binding.get()->eval_a(binding.get().buffer.a.dense) ==
            evaluator::return_status::NotImplemented) {
            if (binding.get()->eval_a(binding.get().buffer.a.sparse) ==
                evaluator::return_status::NotImplemented) {
                throw std::runtime_error("no method implemented for eval_a");
            } else {
                // Compute through sparse view
                binding.get().buffer.a.dense = binding.get().buffer.a.sparse;
            }
        }
        data.g(x_indices) += binding.get().buffer.a.dense;
    }

    /** Quadratic costs **/
    VLOG(10) << "qpoases:quadratic costs";
    for (const auto& binding : program().quadratic_costs()) {
        const auto& x_indices = binding.input_indices[0];
        if (binding.get()->eval_A(binding.get().buffer.A.dense) ==
            evaluator::return_status::NotImplemented) {
            // Throw a warning
        }

        if (binding.is_block()) {
            // data.H.middleRows(x_indices[0], x_indices.size()) +=
            //     binding.get().buffer_A.dense;
        } else {
        }

        if (binding.get()->eval_b(binding.get().buffer.b.dense) ==
            evaluator::return_status::NotImplemented) {
            // Throw a warning
        }
        set_block(data.g, tmp, x_indices, {0}, BinaryOpAddTo{});
    }

    /** Linear constraints **/
    VLOG(10) << "qpoases:linear constraints";
    Eigen::VectorX<value_type> tmp;
    for (const auto& binding : program().linear_constraints()) {
        const auto& x_indices = binding.input_indices[0];
        binding.get()->get_sparsity_A(tmp);
        binding.get()->eval_A(tmp);
        set_block(data.A, tmp, x_indices, {0}, BinaryOpAddTo{});
        // todo - set_block(data.lbA, tmp, x_indices, {0}, BinaryOpAddTo{});
    }

    int nWSR = options_.nWSR;

    qp_->setHessianType(qpOASES::HessianType::HST_POSDEF);

    // Solve
    if (info_.number_of_solves > 0 && options_.perform_hotstart) {
        profiler("qpoases_solver");
        // Use previous solution to hot-start the program
        // qpOASES::SymDenseMat(nx, nx, 0, data.H.data());
        qp_->hotstart(data.H.data().begin(), data.g.data(),
                      data.A.data().begin(), data.lbx.data(), data.ubx.data(),
                      data.lbA.data(), data.ubA.data(), nWSR);
    } else {
        profiler("qpoases_solver");
        // Initialise the program and solve it
        qp_->init(data.H.data().begin(), data.g.data(), data.A.data().begin(),
                  data.lbx.data(), data.ubx.data(), data.lbA.data(),
                  data.ubA.data(), nWSR);
    }

    // Collect information
    info_.nWSR = nWSR;
    info_.status = qp_->getStatus();
    // info_.iterations = ;  // !

    info_.number_of_solves++;

    // Get results
    if (info_.status == qpOASES::QProblemStatus::QPS_SOLVED) {
        info_.success = true;
        // qp_->getPrimalSolution(results_.x.data());
    }
};

void qpoases_solver_instance::reset() { info_.number_of_solves = 0; }

}  // namespace solvers
}  // namespace bopt