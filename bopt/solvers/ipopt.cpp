#include "bopt/solvers/ipopt.hpp"

namespace bopt {
namespace solvers {

ipopt_solver_instance::ipopt_solver_instance(
    mathematical_program<double>& program)
    : Ipopt::TNLP(),
      solver(program),
      program_(program),
      cache_(program.n_variables(), program.n_constraints()) {
    // Construct constraint jacobian
    get_constraint_jacobian(cache_.constraint_jacobian, program.n_variables(),
                            program.get_all_constraints());
}

bool ipopt_solver_instance::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                                         Index& nnz_h_lag,
                                         IndexStyleEnum& index_style) {
    VLOG(10) << "get_nlp_info()";
    n = program().n_variables();
    m = program().n_constraints();

    nnz_jac_g = cache_.constraint_jacobian.nonZeros();
    // nnz_h_lag = data.lagrangian_hessian.nnz;

    index_style = TNLP::C_STYLE;

    return true;
}

bool ipopt_solver_instance::eval_f(Index n, const Number* x, bool new_x,
                                   Number& obj_value) {
    bopt::profiler profiler("ipopt_solver_instance::eval_f");
    VLOG(10) << "eval_f()";

    if (new_x) {
        std::memcpy(cache_.primal_vector.data(), x, n);
    }

    // Update caches
    cache_.objective = 0.0;

    for (auto& binding : costs_) {
        binding.get()->eval(cache_.primal_vector, binding.get()->buffer());
        cache_.objective += binding.get()->buffer();
    }

    // Set objective to most recently cached value
    obj_value = cache_.objective;
    return true;
}

bool ipopt_solver_instance::eval_grad_f(Index n, const Number* x, bool new_x,
                                        Number* grad_f) {
    bopt::profiler profiler("ipopt_solver_instance::eval_grad_f");
    VLOG(10) << "eval_grad_f()";

    if (new_x) {
        std::memcpy(cache_.primal_vector.data(), x, n);
    }

    // Update caches
    for (auto& binding : costs_) {
        cost_tpl<Number>::dense_vector_t& grd =
            binding.get()->buffer_gradient().dense;

        binding.get()->eval_gradient(
            cache_.primal_vector(binding.indices().indices()), grd);

        VLOG(10) << "grd : " << grd.transpose();
        cache_.objective_gradient(binding.indices().indices()) += grd;
    }

    // TODO - See about mapping these
    std::memcpy(grad_f, cache_.objective_gradient.data(), n);
    VLOG(10) << "grad_f : " << cache_.objective_gradient.transpose();
    return true;
}

bool ipopt_solver_instance::eval_g(Index n, const Number* x, bool new_x,
                                   Index m, Number* g) {
    bopt::profiler profiler("ipopt_solver_instance::eval_g");
    VLOG(10) << "eval_g()";
    if (new_x) {
        std::memcpy(cache_.primal_vector.data(), x, n);
    }

    // Update caches
    std::size_t idx = 0;
    // Update caches
    for (auto& binding : constraints_) {
        constraint_tpl<Number>::dense_vector_t& g = binding.get()->buffer();

        binding.get()->eval(cache_.primal_vector(binding.indices().indices()),
                            g);

        VLOG(10) << "g : " << g.transpose();
        if (binding.indices().is_block()) {
            cache_.constraint_vector.block(binding.indices().indices()[0], 0,
                                           binding.get()->sz_out(), 1)
                << g;
        } else {
            cache_.constraint_vector(binding.indices().indices()) = g;
        }
    }
    VLOG(10) << "c : " << cache_.constraint_vector.transpose();
    std::memcpy(g, cache_.constraint_vector.data(), m);
    return true;
};

bool ipopt_solver_instance::eval_jac_g(Index n, const Number* x, bool new_x,
                                       Index m, Index nele_jac, Index* iRow,
                                       Index* jCol, Number* values) {
    bopt::profiler profiler("ipopt_solver_instance::eval_jac_g");
    VLOG(10) << "eval_jac_g()";
    if (values == NULL) {
        // Return the sparsity of the constraint Jacobian
        int cnt = 0;
        for (int k = 0; k < cache_.constraint_jacobian.outerSize(); ++k) {
            for (Eigen::SparseMatrix<double>::InnerIterator it(
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
        if (new_x) {
            std::memcpy(cache_.primal_vector.data(), x, n);
        }

        eval_constraint_jacobian(cache_.primal_vector,
                                 cache_.constraint_jacobian, constraints_);

        // Update caches
        VLOG(10) << "jac : " << cache_.constraint_jacobian;
        std::memcpy(values, cache_.constraint_jacobian.valuePtr(), nele_jac);
    }
    return true;
}

bool ipopt_solver_instance::eval_h(Index n, const Number* x, bool new_x,
                                   Number obj_factor, Index m,
                                   const Number* lambda, bool new_lambda,
                                   Index nele_hess, Index* iRow, Index* jCol,
                                   Number* values) {
    bopt::profiler profiler("ipopt_solver_instance::eval_h");
    VLOG(10) << "eval_h()";
    if (values == NULL) {
        // Return the sparsity of the constraint Jacobian
        int cnt = 0;
        for (int k = 0; k < cache_.lag_hes.outerSize(); ++k) {
            for (Eigen::SparseMatrix<double>::InnerIterator it(cache_.lag_hes,
                                                               k);
                 it; ++it) {
                if (cnt > nele_hess) {
                    return false;
                }
                iRow[cnt] = it.row();
                jCol[cnt] = it.col();
                cnt++;
            }
        }

    } else {
        if (new_x) mapVector(cache_.primal, x, n);
        if (new_lambda) mapVector(cache_.dual, lambda, m);

        // Reset cache for hessian
        cache_.lag_hes *= 0.0;

        // Objective hessian
        for (auto& b : program().f().all()) {
            Eigen::MatrixXd H =
                Eigen::MatrixXd::Zero(b.x().size(), b.x().size());
            auto x_idx = program().x().getIndices(b.x());
            b.get()->hessian(cache_.primal(x_idx), obj_factor, H);
            updateSparseMatrix(cache_.lag_hes, H, x_idx, x_idx, Operation::ADD);
        }
        // Constraint hessians
        std::size_t idx = 0;
        for (auto& b : program().g().all()) {
            Eigen::MatrixXd H =
                Eigen::MatrixXd::Zero(b.x().size(), b.x().size());
            auto x_idx = program().x().getIndices(b.x());
            Eigen::VectorXd lam_i =
                cache_.dual.middleRows(idx, b.get()->size());
            b.get()->hessian(cache_.primal(x_idx), lam_i, H);
            updateSparseMatrix(cache_.lag_hes, H, x_idx, x_idx, Operation::ADD);
        }

        VLOG(10) << "H = ";
        VLOG(10) << cache_.lag_hes;

        copy(cache_.lag_hes, values, nele_hess);
    }
    return true;
}

bool ipopt_solver_instance::get_bounds_info(Index n, Number* x_l, Number* x_u,
                                            Index m, Number* g_l, Number* g_u) {
    VLOG(10) << "get_bounds_info()";

    auto bb = program().bounding_box_constraints();

    VLOG(10) << cache_.variables_lower_bound.transpose();
    VLOG(10) << cache_.variables_upper_bound.transpose();

    // Decision variable bounds
    // todo - copy n

    // Constraint bounds
    for (auto& binding : constraints_) {
        if (binding.indices().is_block()) {
            cache_.constraint_lower_bound.block(binding.indices().indices()[0],
                                                0, binding.get()->sz_out(), 1)
                << binding.get()->lower_bound();

            cache_.constraint_upper_bound.block(binding.indices().indices()[0],
                                                0, binding.get()->sz_out(), 1)
                << binding.get()->upper_bound();
        } else {
            cache_.constraint_lower_bound(binding.indices().indices()) =
                binding.get()->lower_bound();
            cache_.constraint_upper_bound(binding.indices().indices()) =
                binding.get()->upper_bound();
        }
    }

    VLOG(10) << cache_.constraint_lower_bound.transpose();
    VLOG(10) << cache_.constraint_upper_bound.transpose();

    // todo - copy n
    return true;
}

bool ipopt_solver_instance::get_starting_point(Index n, bool init_x, Number* x,
                                               bool init_z, Number* z_L,
                                               Number* z_U, Index m,
                                               bool init_lambda,
                                               Number* lambda) {
    VLOG(10) << "get_starting_point()";

    assert(init_z == false);
    assert(init_lambda == false);

    if (init_x) {
        std::memcpy(x, program().variables_initial_value().data(), n);
    }

    return true;
}

void ipopt_solver_instance::finalize_solution(
    SolverReturn status, Index n, const Number* x, const Number* z_L,
    const Number* z_U, Index m, const Number* g, const Number* lambda,
    Number obj_value, const IpoptData* ip_data,
    IpoptCalculatedQuantities* ip_cq) {
    VLOG(10) << "finalize_solution()";
    for (Index i = 0; i < n; ++i) {
        VLOG(10) << x[i];
    }
}

int IpoptSolver::solve() {
    // Create a new instance of your nlp
    //  (use a SmartPtr, not raw)
    Ipopt::SmartPtr<Ipopt::TNLP> nlp = new ipopt_solver_instance(program_);

    Ipopt::SmartPtr<IpoptApplication> app = IpoptApplicationFactory();
    app->Options()->SetNumericValue("tol", 1e-3);
    app->Options()->SetStringValue("mu_strategy", "adaptive");

    // Initialize the IpoptApplication and process the options
    Ipopt::ApplicationReturnStatus status;
    status = app->Initialize();
    if (status != Ipopt::Solve_Succeeded) {
        LOG(INFO) << std::endl
                  << std::endl
                  << "*** Error during initialization!" << std::endl;
        return (int)status;
    }

    // Ask Ipopt to solve the problem
    status = app->OptimizeTNLP(nlp);

    if (status == Solve_Succeeded) {
        LOG(INFO) << "*** The problem solved!" << std::endl;
    } else {
        LOG(INFO) << "*** The problem FAILED!" << std::endl;
    }

    return (int)status;
}

}  // namespace solvers
}  // namespace bopt
