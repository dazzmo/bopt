#include "bopt/solvers/ipopt.hpp"

#include "bopt/logging.hpp"

namespace bopt {
namespace solvers {

ipopt_program_instance::ipopt_program_instance(MathematicalProgram& program)
    : Ipopt::TNLP(),
      program_(program),
      cache_(program.numVariables(), program.numConstraints()) {
    // Create data
    dense_costs_ = program.getCosts<DenseCost>();
    sparse_costs_ = program.getCosts<SparseCost>();

    dense_constraints_ = program.getConstraints<DenseConstraint>();
    sparse_constraints_ = program.getConstraints<SparseConstraint>();

    // Create the jacobian for the problem
    VLOG(10) << "Constraint Jacobian";
    // Construct constraint jacobian and lagrangian
    Index c_idx = 0;
    std::vector<Eigen::Triplet<Real>> triplets;
    // Dense constraint jacobian
    for (auto& b : dense_constraints_) {
        const auto& c = *b.get();
        auto& d = *b.data();
        const auto& indices = b.indices().indices();
        for (Index row = 0; row < c.getOutputDimension(); ++row) {
            for (Index col = 0; col < c.getInputTangentSpaceDimension();
                 ++col) {
                triplets.push_back(
                    Eigen::Triplet<Real>(c_idx + row, indices[col]));
            }
        }
        c_idx += c.getOutputDimension();
    }
    // Sparse constraint jacobians
    for (auto& b : sparse_constraints_) {
        const auto& c = *b.get();
        auto& d = *b.data();
        const auto& indices = b.indices().indices();
        for (int k = 0; k < d.Jx.outerSize(); ++k) {
            for (SparseMatrix<Real>::InnerIterator it(d.Jx, k); it; ++it) {
                triplets.push_back(
                    Eigen::Triplet<Real>(c_idx + it.row(), indices[it.col()]));
            }
        }
        c_idx += c.getOutputDimension();
    }

    cache_.constraint_jacobian.setFromTriplets(triplets.begin(),
                                               triplets.end());
    cache_.constraint_jacobian.makeCompressed();

    // Assemble look-up map for indices
    for (Index k = 0; k < cache_.constraint_jacobian.outerSize(); ++k) {
        Index inner_nz_cnt = 0;
        for (SparseMatrix<Real>::InnerIterator it(cache_.constraint_jacobian,
                                                  k);
             it; ++it) {
            jac_nz_map_.insert(
                {{it.row(), it.col()},
                 cache_.constraint_jacobian.outerIndexPtr()[it.outer()] +
                     inner_nz_cnt++});
        }
    }

    VLOG(10) << cache_.constraint_jacobian;

    // Construct lagrangian hessian
    triplets.clear();
    VLOG(10) << "Lagrangian Hessian";
    for (auto& b : dense_costs_) {
        const auto& c = *b.get();
        auto& d = *b.data();
        const auto& indices = b.indices().indices();
        // Dense output - currently use block insert
        for (Index row = 0; row < c.getInputTangentSpaceDimension(); ++row) {
            for (Index col = 0; col <= row; ++col) {
                triplets.push_back(
                    Eigen::Triplet<Real>(indices[row], indices[col]));
            }
        }
    }
    for (auto& b : dense_constraints_) {
        const auto& c = *b.get();
        auto& d = *b.data();
        const auto& indices = b.indices().indices();
        // Dense output - currently use block insert
        for (Index row = 0; row < c.getInputTangentSpaceDimension(); ++row) {
            for (Index col = 0; col <= row; ++col) {
                triplets.push_back(
                    Eigen::Triplet<Real>(indices[row], indices[col]));
            }
        }
    }

    for (auto& b : sparse_costs_) {
        const auto& c = *b.get();
        auto& d = *b.data();
        const auto& indices = b.indices().indices();
        // Dense output - currently use block insert
        for (int k = 0; k < d.Hxx.outerSize(); ++k) {
            for (SparseMatrix<Real>::InnerIterator it(d.Hxx, k); it; ++it) {
                triplets.push_back(
                    Eigen::Triplet<Real>(indices[it.row()], indices[it.col()]));
            }
        }
    }
    for (auto& b : sparse_constraints_) {
        const auto& c = *b.get();
        auto& d = *b.data();
        const auto& indices = b.indices().indices();
        for (int k = 0; k < d.Hxx.outerSize(); ++k) {
            for (SparseMatrix<Real>::InnerIterator it(d.Hxx, k); it; ++it) {
                triplets.push_back(
                    Eigen::Triplet<Real>(indices[it.row()], indices[it.col()]));
            }
        }
    }

    // Convert to compressed form
    cache_.lagrangian_hessian.setFromTriplets(triplets.begin(), triplets.end());
    // Assemble look-up map for indices
    for (Index k = 0; k < cache_.lagrangian_hessian.outerSize(); ++k) {
        Index inner_nz_cnt = 0;
        for (SparseMatrix<Real>::InnerIterator it(cache_.lagrangian_hessian, k);
             it; ++it) {
            lag_hes_nz_map_.insert(
                {{it.row(), it.col()},
                 cache_.lagrangian_hessian.outerIndexPtr()[it.outer()] +
                     inner_nz_cnt++});
        }
    }
    VLOG(10) << cache_.lagrangian_hessian;
}

bool ipopt_program_instance::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                                          Index& nnz_h_lag,
                                          IndexStyleEnum& index_style) {
    VLOG(10) << "get_nlp_info()";
    n = program().numVariables();
    m = program().numConstraints();

    VLOG(10) << "n = " << n;
    VLOG(10) << "m = " << m;

    nnz_jac_g = cache_.constraint_jacobian.nonZeros();
    nnz_h_lag = cache_.lagrangian_hessian.nonZeros();

    index_style = TNLP::C_STYLE;

    return true;
}

bool ipopt_program_instance::eval_f(Index n, const Number* x, bool new_x,
                                    Number& obj_value) {
    bopt::profiler profiler("ipopt_program_instance::eval_f");
    VLOG(10) << "eval_f()";

    if (new_x) {
        std::copy_n(x, n, cache_.primal_vector.data());
    }

    // Update caches
    cache_.objective = 0.0;

    // Dense costs
    for (auto& binding : dense_costs_) {
        auto& c = *binding.get();
        auto& d = *binding.data();
        const auto& indices = binding.indices().indices();
        const auto& xi = cache_.primal_vector(indices);

        // Evaluate objective
        c.eval(xi, d);
        cache_.objective += c.scaling_factor() * d.y;
    }

    // Sparse costs
    // todo - maybe make a function for this to avoid code repetition
    for (auto& binding : sparse_costs_) {
        auto& c = *binding.get();
        auto& d = *binding.data();
        const auto& indices = binding.indices().indices();
        const auto& xi = cache_.primal_vector(indices);

        // Evaluate objective
        c.eval(xi, d);
        cache_.objective += c.scaling_factor() * d.y;
    }

    // Set objective to most recently cached value
    VLOG(10) << "f: " << cache_.objective;
    obj_value = cache_.objective;
    return true;
}

bool ipopt_program_instance::eval_grad_f(Index n, const Number* x, bool new_x,
                                         Number* grad_f) {
    bopt::profiler profiler("ipopt_program_instance: eval_grad_f");
    VLOG(10) << "eval_grad_f()";

    if (new_x) {
        std::copy_n(x, n, cache_.primal_vector.data());
    }

    // Update caches
    cache_.objective_gradient.setZero();

    // Dense costs
    for (auto& binding : dense_costs_) {
        auto& c = *binding.get();
        auto& d = *binding.data();
        const auto& indices = binding.indices().indices();
        const auto& xi = cache_.primal_vector(indices);

        // Evaluate objective
        c.evalGradients(xi, d, true, false);
        cache_.objective_gradient(indices) += c.scaling_factor() * d.gx;
    }

    // Sparse costs
    // todo - maybe make a function for this to avoid code repetition
    for (auto& binding : sparse_costs_) {
        auto& c = *binding.get();
        auto& d = *binding.data();
        const auto& indices = binding.indices().indices();
        const auto& xi = cache_.primal_vector(indices);

        c.evalGradients(xi, d, true, false);

        for (int k = 0; k < d.gx.outerSize(); ++k) {
            for (SparseVector<Real>::InnerIterator it(d.gx, k); it; ++it) {
                cache_.objective_gradient[indices[it.row()]] +=
                    c.scaling_factor() * it.value();
            }
        }
    }

    // TODO - See about mapping these
    VLOG(10) << "grad_f : " << cache_.objective_gradient.transpose();
    std::copy_n(cache_.objective_gradient.data(), n, grad_f);
    return true;
}

bool ipopt_program_instance::eval_g(Index n, const Number* x, bool new_x,
                                    Index m, Number* g) {
    bopt::profiler profiler("ipopt_program_instance::eval_g");
    VLOG(10) << "eval_g()";
    if (new_x) {
        std::copy_n(x, n, cache_.primal_vector.data());
    }

    Index c_idx = 0;
    // Dense constraints
    for (auto& binding : dense_constraints_) {
        auto& c = *binding.get();
        auto& d = *binding.data();
        const auto& indices = binding.indices().indices();
        const auto& xi = cache_.primal_vector(indices);

        c.eval(xi, d);
        cache_.constraint_vector.middleRows(c_idx, c.getOutputDimension()) =
            d.y;
        c_idx += c.getOutputDimension();
    }

    // Sparse constraints
    // todo - maybe make a function for this to avoid code repetition
    for (auto& binding : sparse_constraints_) {
        auto& c = *binding.get();
        auto& d = *binding.data();
        const auto& indices = binding.indices().indices();
        const auto& xi = cache_.primal_vector(indices);

        c.eval(xi, d);
        cache_.constraint_vector.middleRows(c_idx, c.getOutputDimension()) =
            d.y;
        c_idx += c.getOutputDimension();
    }

    VLOG(10) << "c : " << cache_.constraint_vector.transpose();
    std::copy_n(cache_.constraint_vector.data(), m, g);
    return true;
};

bool ipopt_program_instance::eval_jac_g(Index n, const Number* x, bool new_x,
                                        Index m, Index nele_jac, Index* iRow,
                                        Index* jCol, Number* values) {
    if (values == NULL) {
        // Return the sparsity of the constraint Jacobian
        int cnt = 0;
        for (int k = 0; k < cache_.constraint_jacobian.outerSize(); ++k) {
            for (Eigen::SparseMatrix<Real>::InnerIterator it(
                     cache_.constraint_jacobian, k);
                 it; ++it) {
                if (cnt > nele_jac) {
                    return false;
                }
                iRow[cnt] = it.row();
                jCol[cnt] = it.col();
                cnt++;
            }
        }

    } else {
        bopt::profiler profiler("ipopt_program_instance: eval_jac_g");
        VLOG(10) << "eval_jac_g()";
        if (new_x) {
            std::copy_n(x, n, cache_.primal_vector.data());
        }

        // Update caches
        Index c_idx = 0;
        // Dense constraints
        for (auto& binding : dense_constraints_) {
            auto& c = *binding.get();
            auto& d = *binding.data();
            const auto& indices = binding.indices().indices();
            const auto& xi = cache_.primal_vector(indices);

            c.evalJacobians(xi, d, true, false);
            for (Index row = 0; row < c.getOutputDimension(); ++row) {
                for (Index col = 0; col < c.getInputTangentSpaceDimension();
                     ++col) {
                    cache_.constraint_jacobian.valuePtr()[jac_nz_map_.at(
                        {c_idx + row, indices[col]})] = d.Jx(row, col);
                }
            }
            c_idx += c.getOutputDimension();
        }

        // Sparse constraints
        // todo - maybe make a function for this to avoid code repetition
        for (auto& binding : sparse_constraints_) {
            auto& c = *binding.get();
            auto& d = *binding.data();
            const auto& indices = binding.indices().indices();
            const auto& xi = cache_.primal_vector(indices);

            c.evalJacobians(xi, d, true, false);

            for (int k = 0; k < d.Jx.outerSize(); ++k) {
                for (SparseMatrix<Real>::InnerIterator it(d.Jx, k); it; ++it) {
                    cache_.constraint_jacobian.valuePtr()[jac_nz_map_.at(
                        {c_idx + it.row(), indices[it.col()]})] = it.value();
                }
            }
            c_idx += c.getOutputDimension();
        }

        // Update caches
        VLOG(10) << "jac : " << cache_.constraint_jacobian;
        std::copy_n(cache_.constraint_jacobian.valuePtr(), nele_jac, values);
        VLOG(10) << "finished";
    }
    return true;
}

bool ipopt_program_instance::eval_h(Index n, const Number* x, bool new_x,
                                    Number obj_factor, Index m,
                                    const Number* lambda, bool new_lambda,
                                    Index nele_hess, Index* iRow, Index* jCol,
                                    Number* values) {
    VLOG(10) << "eval_h()";
    if (values == NULL) {
        // Return the sparsity of the constraint Jacobian
        int cnt = 0;
        for (int k = 0; k < cache_.lagrangian_hessian.outerSize(); ++k) {
            for (Eigen::SparseMatrix<Real>::InnerIterator it(
                     cache_.lagrangian_hessian, k);
                 it; ++it) {
                if (cnt > nele_hess) {
                    return false;
                }
                iRow[cnt] = it.row();
                jCol[cnt] = it.col();
                cnt++;
            }
        }
        return true;

    } else {
        bopt::profiler profiler("ipopt_program_instance: eval_h");
        VLOG(10) << "eval_h()";
        if (new_x) {
            std::copy_n(x, n, cache_.primal_vector.data());
        }
        if (new_lambda) {
            std::copy_n(lambda, m, cache_.dual_vector.data());
        }

        // Costs
        // Dense costs
        VLOG(10) << "dense cost";
        for (auto& binding : dense_costs_) {
            auto& c = *binding.get();
            auto& d = *binding.data();
            const auto& indices = binding.indices().indices();
            const auto& xi = cache_.primal_vector(indices);

            c.evalHessians(xi, d, true, false, false);
            for (Index row = 0; row < c.getInputTangentSpaceDimension();
                 ++row) {
                for (Index col = 0; col < row; ++col) {
                    cache_.lagrangian_hessian.valuePtr()[lag_hes_nz_map_.at(
                        {indices[row], indices[col]})] +=
                        obj_factor * d.Hxx(row, col);
                }
            }
        }

        // Sparse costs
        VLOG(10) << "sparse cost";
        // todo - maybe make a function for this to avoid code repetition
        for (auto& binding : sparse_costs_) {
            auto& c = *binding.get();
            auto& d = *binding.data();
            const auto& indices = binding.indices().indices();
            const auto& xi = cache_.primal_vector(indices);

            c.evalHessians(xi, d, true, false, false);
            for (int k = 0; k < d.Hxx.outerSize(); ++k) {
                for (SparseMatrix<Real>::InnerIterator it(d.Hxx, k); it; ++it) {
                    cache_.lagrangian_hessian.valuePtr()[lag_hes_nz_map_.at(
                        {indices[it.row()], indices[it.col()]})] +=
                        obj_factor * it.value();
                }
            }
        }

        // Constraints
        Index c_idx = 0;
        // Dense constraints
        VLOG(10) << "dense constraint";
        for (auto& binding : dense_constraints_) {
            auto& c = *binding.get();
            auto& d = *binding.data();
            const auto& indices = binding.indices().indices();
            const auto& xi = cache_.primal_vector(indices);
            const auto& li =
                cache_.dual_vector.middleRows(c_idx, c.getOutputDimension());

            c.evalHessians(xi, li, d, true, false, false);

            for (Index row = 0; row < c.getInputTangentSpaceDimension();
                 ++row) {
                for (Index col = 0; col < row; ++col) {
                    cache_.lagrangian_hessian.valuePtr()[lag_hes_nz_map_.at(
                        {indices[row], indices[col]})] = d.Hxx(row, col);
                }
            }
            c_idx += c.getOutputDimension();
        }

        // Sparse constraints
        VLOG(10) << "sparse constraint";
        // todo - maybe make a function for this to avoid code repetition
        for (auto& binding : sparse_constraints_) {
            auto& c = *binding.get();
            auto& d = *binding.data();
            const auto& indices = binding.indices().indices();
            const auto& xi = cache_.primal_vector(indices);
            const auto& li =
                cache_.dual_vector.middleRows(c_idx, c.getOutputDimension());

            c.evalHessians(xi, li, d, true, false, false);

            for (int k = 0; k < d.Hxx.outerSize(); ++k) {
                for (SparseMatrix<Real>::InnerIterator it(d.Hxx, k); it; ++it) {
                    cache_.lagrangian_hessian.valuePtr()[lag_hes_nz_map_.at(
                        {indices[it.row()], indices[it.col()]})] +=
                        obj_factor * it.value();
                }
            }
            c_idx += c.getOutputDimension();
        }

        // Update caches
        VLOG(10) << "hes : " << cache_.lagrangian_hessian;
        std::copy_n(cache_.lagrangian_hessian.valuePtr(), nele_hess, values);
        VLOG(10) << "finished";
        return true;
    }
}

bool ipopt_program_instance::get_bounds_info(Index n, Number* x_l, Number* x_u,
                                             Index m, Number* g_l,
                                             Number* g_u) {
    VLOG(10) << "get_bounds_info()";

    // Variable bounds
    cache_.variables_lower_bound = program().variableLowerBounds();
    cache_.variables_upper_bound = program().variableUpperBounds();

    auto bb = program_.getBoundingBoxConstraints();
    for (auto& binding : bb) {
        const auto& c = *binding.get();
        auto& d = *binding.data();
        const auto& indices = binding.indices().indices();
        c.evalBounds(d);

        cache_.variables_lower_bound(indices).array() =
            cache_.variables_lower_bound(indices).array().max(d.lb.array());

        cache_.variables_upper_bound(indices).array() =
            cache_.variables_upper_bound(indices).array().min(d.ub.array());
    }

    VLOG(10) << cache_.variables_lower_bound.transpose();
    VLOG(10) << cache_.variables_upper_bound.transpose();

    std::copy_n(cache_.variables_lower_bound.data(), n, x_l);
    std::copy_n(cache_.variables_upper_bound.data(), n, x_u);

    // Constraint bounds
    Index c_idx = 0;
    for (auto& binding : dense_constraints_) {
        const auto& c = *binding.get();
        auto& d = *binding.data();
        c.evalBounds(d);
        cache_.constraint_lower_bound.middleRows(c_idx, c.getOutputDimension())
            << d.lb;
        cache_.constraint_upper_bound.middleRows(c_idx, c.getOutputDimension())
            << d.ub;
        c_idx += c.getOutputDimension();
    }

    VLOG(10) << cache_.constraint_lower_bound.transpose();
    VLOG(10) << cache_.constraint_upper_bound.transpose();

    std::copy_n(cache_.constraint_lower_bound.data(), m, g_l);
    std::copy_n(cache_.constraint_upper_bound.data(), m, g_u);

    return true;
}

bool ipopt_program_instance::get_starting_point(Index n, bool init_x, Number* x,
                                                bool init_z, Number* z_L,
                                                Number* z_U, Index m,
                                                bool init_lambda,
                                                Number* lambda) {
    VLOG(10) << "get_starting_point()";
    VLOG(10) << "x0: " << program().variableInitialValues().transpose();

    assert(init_z == false);
    assert(init_lambda == false);

    if (init_x) {
        std::copy_n(program().variableInitialValues().data(), n, x);
    }

    return true;
}

void ipopt_program_instance::finalize_solution(
    Ipopt::SolverReturn status, Index n, const Number* x, const Number* z_L,
    const Number* z_U, Index m, const Number* g, const Number* lambda,
    Number obj_value, const Ipopt::IpoptData* ip_data,
    Ipopt::IpoptCalculatedQuantities* ip_cq) {
    VLOG(10) << "finalize_solution()";
    for (Index i = 0; i < n; ++i) {
        std::cout << x[i] << std::endl;
    }
}

ipopt_solver::ipopt_solver(MathematicalProgram& program) : solver(program) {
    // Create program instance
    nlp_ = new ipopt_program_instance(program);

    // Create application
    app_ = IpoptApplicationFactory();
    // Initialize the IpoptApplication and process the options
    Ipopt::ApplicationReturnStatus status;
    status = app_->Initialize();
    if (status != Ipopt::Solve_Succeeded) {
        LOG(INFO) << std::endl
                  << std::endl
                  << "*** Error during initialization!" << std::endl;
    }
}

int ipopt_solver::solve() {
    // Ask Ipopt to solve the problem
    Ipopt::ApplicationReturnStatus status;
    {
        profiler profiler("ipopt_solver solve");
        status = app_->OptimizeTNLP(nlp_);
    }

    if (status == Ipopt::ApplicationReturnStatus::Solve_Succeeded) {
        LOG(INFO) << "*** The problem solved!" << std::endl;
    } else {
        LOG(INFO) << "*** The problem FAILED!" << std::endl;
    }

    return (int)status;
}

}  // namespace solvers
}  // namespace bopt
