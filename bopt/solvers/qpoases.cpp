#include "bopt/solvers/qpoases.hpp"

namespace bopt {
namespace solvers {

qpoases_solver::qpoases_solver(MathematicalProgram& program) : solver(program) {
    LOG(INFO) << "qpoases_solver::qpoases_solver";

    // Create problem
    int nx = program.n_variables();
    int ng = program.n_constraints();

    qp_ = std::make_unique<qpOASES::SQProblem>(nx, ng);

    // Create matrix data
    data.H.resize(nx, nx);
    data.H.setZero();

    data.g.resize(nx);
    data.g.setZero();

    data.A.resize(ng, nx);
    data.A.setZero();

    data.lbA.resize(ng);
    data.ubA.resize(ng);

    data.lbx.resize(nx);
    data.ubx.resize(nx);

    data.lbx = program.variables_lower_bound();
    data.ubx = program.variables_upper_bound();

    VLOG(10) << "lbx: " << data.lbx.transpose();
    VLOG(10) << "ubx: " << data.ubx.transpose();
}

qpoases_solver::~qpoases_solver() = default;

void qpoases_solver::solve(MathematicalProgram& program) {
    Eigen::MatrixXd tmp;

    /** Bounding box constraints **/
    {
        bopt::profiler profiler("qpoases: bounding box constraints");
        for (auto& binding : program.BoundingBoxConstraints()) {
            data.lbx(binding.indices().indices())
                << binding.get()->lowerBound();
            data.ubx(binding.indices().indices())
                << binding.get()->upperBound();
        }
    }

    /** Linear costs **/
    {
        bopt::profiler profiler("qpoases: linear costs");
        VLOG(10) << "qpoases:linear costs";
        for (auto& binding : program.linear_costs()) {
            const auto& c = *binding.get();
            const auto& indices = binding.indices().indices();

            // Create vector
            VectorXd a(c.dim_input());
            if (c.a_has_nz_only()) a.resize(c.a_sparsity_pattern()->size());
            int cnt = 0;
            if (c.a_sparsity_pattern().has_value()) {
                // Sparse insert
                for (const auto& xy : *c.a_sparsity_pattern()) {
                    double& entry = data.g(indices[xy.first]);
                    if (c.a_has_nz_only()) {
                        entry += a[cnt++];
                    } else {
                        entry += a(xy.first);
                    }
                }
            } else {
                // Perform block insert
                if (binding.indices().is_block()) {
                    data.g.middleRows(indices[0], indices.size()) += a;
                } else {
                    data.g(indices, indices) += a;
                }
            }

            // Whether to also include the constant value
            double b;
        }
    }

    /** Quadratic costs **/
    {
        bopt::profiler profiler("qpoases: quadratic costs");
        VLOG(10) << "qpoases:quadratic costs";
        for (auto& binding : program.quadratic_costs()) {
            auto& c = *binding.get();
            const auto& indices = binding.indices().indices();

            // A
            VLOG(10) << "A";
            MatrixXd A(c.dim_input(), c.dim_input());
            if (c.A_has_nz_only()) A.resize(c.A_sparsity_pattern()->size(), 1);
            // Evaluate A matrix
            c.evalA(A);

            if (c.A_sparsity_pattern().has_value()) {
                int cnt = 0;
                for (const auto& xy : *c.A_sparsity_pattern()) {
                    double& Hij = data.H(indices[xy.first], indices[xy.second]);
                    // todo - ensure this is lower triangular
                    if (c.A_has_nz_only()) {
                        Hij += A(cnt++);
                    } else {
                        Hij += A(xy.first, xy.second);
                    }
                }
            } else {
                // Perform block insert
                if (binding.indices().is_block()) {
                    data.H.block(indices[0], indices[0], indices.size(),
                                 indices.size()) += A;
                } else {
                    data.H(indices, indices) += A;
                }
            }

            // b
            VLOG(10) << "b";
            VectorXd b(c.dim_input());
            if (c.b_has_nz_only()) b.resize(c.b_sparsity_pattern()->size());
            VLOG(10) << b;
            c.evalb(b);

            if (c.b_sparsity_pattern().has_value()) {
                int cnt = 0;
                for (const auto& xy : *c.b_sparsity_pattern()) {
                    double& gi = data.g(indices[xy.first]);
                    if (c.b_has_nz_only()) {
                        gi += b[cnt++];
                    } else {
                        gi += b(xy.first);
                    }
                }
            } else {
                // Perform block insert
                if (binding.indices().is_block()) {
                    data.g.middleRows(indices[0], indices.size()) += b;
                } else {
                    data.g(indices) += b;
                }
            }
        }
    }

    /** Linear constraints **/
    {
        bopt::profiler profiler("qpoases: linear constraints");

        VLOG(10) << "qpoases:linear constraints";
        int row = 0;
        for (auto& binding : program.linear_constraints()) {
            auto& c = *binding.get();
            const auto& indices = binding.indices().indices();

            MatrixXd A(c.dim_output(), c.dim_input());
            if (c.A_has_nz_only()) A.resize(c.A_sparsity_pattern()->size(), 1);
            // Evaluate A matrix
            c.evalA(A);

            if (c.A_sparsity_pattern().has_value()) {
                int cnt = 0;
                for (const auto& xy : *c.A_sparsity_pattern()) {
                    double& Aij =
                        data.A(row + indices[xy.first], indices[xy.second]);
                    if (c.A_has_nz_only()) {
                        Aij = A(cnt++);
                    } else {
                        Aij = A(xy.first, xy.second);
                    }
                }
            } else {
                // Perform block insert
                if (binding.indices().is_block()) {
                    data.A.block(indices[0], indices[0], indices.size(),
                                 indices.size()) += A;
                } else {
                    data.A(indices, indices) += A;
                }
            }

            data.lbA.middleRows(row, c.dim_output()) << c.lowerBound();
            data.ubA.middleRows(row, c.dim_output()) << c.upperBound();

            row += c.dim_output();
        }
    }

    int nWSR = options_.nWSR;

    qp_->setHessianType(qpOASES::HessianType::HST_POSDEF);
    // todo - set this only once?
    qp_->setOptions(options_);

    VLOG(10) << "H: " << data.H;
    VLOG(10) << "g: " << data.g;
    VLOG(10) << "A: " << data.A;
    VLOG(10) << "lbA: " << data.lbA;
    VLOG(10) << "ubA: " << data.ubA;

    // Solve
    if (info_.number_of_solves > 0 && options_.perform_hotstart) {
        bopt::profiler profiler("qpoases: solve");
        // Use previous solution to hot-start the program
        // qpOASES::SymDenseMat(nx, nx, 0, data.H.data());
        qp_->hotstart(data.H.data(), data.g.data(), data.A.data(),
                      data.lbx.data(), data.ubx.data(), data.lbA.data(),
                      data.ubA.data(), nWSR);
    } else {
        bopt::profiler profiler("qpoases: solve");
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

void qpoases_solver::reset() { info_.number_of_solves = 0; }

}  // namespace solvers
}  // namespace bopt