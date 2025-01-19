#include "bopt/solvers/qpoases.hpp"

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

    data.g.resize(nx);

    data.A.resize(ng, nx);

    data.lbA.resize(ng);
    data.ubA.resize(ng);

    data.lbx.resize(nx);
    data.ubx.resize(nx);

    data.lbx = program.variables_lower_bound();
    data.ubx = program.variables_upper_bound();

    for (auto& binding : program.bounding_box_constraints()) {
        data.lbx(binding.indices().indices()) << binding.get()->lower_bound();
        data.ubx(binding.indices().indices()) << binding.get()->upper_bound();
    }

    VLOG(10) << "lbx: " << data.lbx.transpose();
    VLOG(10) << "ubx: " << data.ubx.transpose();
}

qpoases_solver_instance::~qpoases_solver_instance() = default;

void qpoases_solver_instance::solve(mathematical_program<double>& program) {
    Eigen::MatrixXd tmp;

    /** Linear costs **/
    VLOG(10) << "qpoases:linear costs";
    for (auto& binding : program.linear_costs()) {
        const auto& x_indices = binding.indices().indices();
        if (binding.get()->eval_a(binding.get()->buffer_a().dense) ==
            evaluator::return_status::NotImplemented) {
            if (binding.get()->eval_a(binding.get()->buffer_a().sparse) ==
                evaluator::return_status::NotImplemented) {
                throw std::runtime_error("no method implemented for eval_a");
            } else {
                // Compute through sparse view
                binding.get()->buffer_a().dense =
                    binding.get()->buffer_a().sparse;
            }
        }
        data.g(x_indices) += binding.get()->buffer_a().dense;
    }

    /** Quadratic costs **/
    VLOG(10) << "qpoases:quadratic costs";
    for (auto& binding : program.quadratic_costs()) {
        const auto& x_indices = binding.indices().indices();
        // References to matrix data
        Eigen::MatrixXd& A = binding.get()->buffer_A().dense;
        Eigen::VectorXd& b = binding.get()->buffer_b().dense;

        if (binding.get()->eval_A(A) ==
            evaluator::return_status::NotImplemented) {
            if (binding.get()->eval_A(binding.get()->buffer_A().sparse) ==
                evaluator::return_status::NotImplemented) {
                throw std::runtime_error("no method implemented for eval_A");
            } else {
                // Compute through sparse view
                A = binding.get()->buffer_A().sparse;
            }
        }

        // Perform block insert for lower-triangular hessian
        // todo - use lower triangular
        if (binding.indices().is_block()) {
            data.H.block(x_indices[0], x_indices[0], x_indices.size(),
                         x_indices.size()) += A;
        } else {
            data.H(x_indices, x_indices) += A;
        }

        if (binding.get()->eval_b(b) ==
            evaluator::return_status::NotImplemented) {
            if (binding.get()->eval_b(binding.get()->buffer_b().sparse) ==
                evaluator::return_status::NotImplemented) {
                throw std::runtime_error("no method implemented for eval_b");
            } else {
                // Compute through sparse view
                b = binding.get()->buffer_b().sparse;
            }
        }
        data.g(x_indices) += A * b;
    }

    /** Linear constraints **/
    VLOG(10) << "qpoases:linear constraints";
    for (auto& binding : program.linear_constraints()) {
        const auto& x_indices = binding.indices().indices();
        // References to matrix data
        Eigen::MatrixXd& A = binding.get()->buffer_A().dense;
        Eigen::VectorXd& b = binding.get()->buffer_b().dense;

        if (binding.get()->eval_A(A) ==
            evaluator::return_status::NotImplemented) {
            if (binding.get()->eval_A(binding.get()->buffer_A().sparse) ==
                evaluator::return_status::NotImplemented) {
                throw std::runtime_error("no method implemented for eval_A");
            } else {
                // Compute through sparse view
                A = binding.get()->buffer_A().sparse;
            }
        }
        // Perform block insert
        if (binding.indices().is_block()) {
            data.A.block(x_indices[0], x_indices[0], x_indices.size(),
                         x_indices.size()) += A;
        } else {
            data.A(x_indices, x_indices) += A;
        }

        if (binding.get()->eval_b(b) ==
            evaluator::return_status::NotImplemented) {
            if (binding.get()->eval_b(binding.get()->buffer_b().sparse) ==
                evaluator::return_status::NotImplemented) {
                throw std::runtime_error("no method implemented for eval_b");
            } else {
                // Compute through sparse view
                b = binding.get()->buffer_b().sparse;
            }
        }
        data.lbA(x_indices) = binding.get()->lower_bound() - b;
        data.ubA(x_indices) = binding.get()->upper_bound() - b;
    }

    int nWSR = options_.nWSR;
    
    qp_->setHessianType(qpOASES::HessianType::HST_POSDEF);

    // Solve
    if (info_.number_of_solves > 0 && options_.perform_hotstart) {
        profiler("qpoases_solver");
        // Use previous solution to hot-start the program
        // qpOASES::SymDenseMat(nx, nx, 0, data.H.data());
        qp_->hotstart(data.H.data(), data.g.data(), data.A.data(),
                      data.lbx.data(), data.ubx.data(), data.lbA.data(),
                      data.ubA.data(), nWSR);
    } else {
        profiler("qpoases_solver");
        // Initialise the program and solve it
        qp_->init(data.H.data(), data.g.data(), data.A.data(), data.lbx.data(),
                  data.ubx.data(), data.lbA.data(), data.ubA.data(), nWSR);
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

void qpoases_solver_instance::reset() { info_.number_of_solves = 0; }

}  // namespace solvers
}  // namespace bopt