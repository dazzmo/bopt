#include "bopt/solvers/ipopt.hpp"

#include "bopt/logging.hpp"

namespace bopt {
namespace solvers {

ipopt_program_instance::ipopt_program_instance(MathematicalProgram& program)
    : Ipopt::TNLP(),
      program_(program),
      cache_(program.n_variables(), program.n_constraints()) {
    costs_ = program.get_all_costs();
    constraints_ = program.get_all_constraints();

    VLOG(10) << "Constraint Jacobian";
    // Construct constraint jacobian and lagrangian
    int idx = 0;
    for (auto& b : constraints_) {
        auto& c = *b.get();
        VLOG(10) << c;
        if (c.jacobian_x_sparsity_pattern().has_value()) {
            // Sparse insert
            for (auto xy : *c.jacobian_x_sparsity_pattern()) {
                cache_.constraint_jacobian.insert(
                    idx + xy.first, b.indices().indices()[xy.second]);
            }
        } else {
            // Dense output - currently use block insert
            for (Index row = 0; row < c.dim_output(); ++row) {
                for (Index col = 0; col < c.dim_tangent_space(); ++col) {
                    cache_.constraint_jacobian.insert(
                        idx + row, b.indices().indices()[col]);
                }
            }
        }
        idx += c.dim_output();
    }
    VLOG(10) << cache_.constraint_jacobian;

    // Construct lagrangian hessian
    VLOG(10) << "Lagrangian Hessian";
    for (auto& b : costs_) {
        auto& c = *b.get();
        VLOG(10) << c;
        if (c.hessian_xx_sparsity_pattern().has_value()) {
            // Sparse insert
            for (auto xy : *c.hessian_xx_sparsity_pattern()) {
                cache_.lagrangian_hessian.insert(
                    b.indices().indices()[xy.first],
                    b.indices().indices()[xy.second]);
            }
        } else {
            // Dense output - currently use block insert
            for (Index row = 0; row < c.dim_tangent_space(); ++row) {
                for (Index col = 0; col <= row; ++col) {
                    cache_.lagrangian_hessian.insert(
                        b.indices().indices()[row], b.indices().indices()[col]);
                }
            }
        }
    }
    idx = 0;
    for (auto& b : constraints_) {
        auto& c = *b.get();
        VLOG(10) << c;
        if (c.hessian_xx_sparsity_pattern().has_value()) {
            // Sparse insert
            for (auto xy : *c.hessian_xx_sparsity_pattern()) {
                cache_.lagrangian_hessian.insert(
                    b.indices().indices()[xy.first],
                    b.indices().indices()[xy.second]);
            }
        } else {
            // Dense output - currently use block insert
            // Dense output - currently use block insert
            for (Index row = 0; row < c.dim_tangent_space(); ++row) {
                for (Index col = 0; col <= row; ++col) {
                    cache_.lagrangian_hessian.insert(
                        b.indices().indices()[row], b.indices().indices()[col]);
                }
            }
        }
        idx += c.dim_output();
    }
    VLOG(10) << cache_.lagrangian_hessian;
}

bool ipopt_program_instance::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                                          Index& nnz_h_lag,
                                          IndexStyleEnum& index_style) {
    VLOG(10) << "get_nlp_info()";
    n = program().n_variables();
    m = program().n_constraints();

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

    for (auto& binding : costs_) {
        auto& obj = *binding.get();
        const auto& indices = binding.indices().indices();
        const auto& xi = cache_.primal_vector(indices);
        // Evaluate objective
        double f;
        binding.get()->eval(xi, Eigen::Map<VectorXd>(&f, 1));
        cache_.objective += f;
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
    for (auto& binding : costs_) {
        auto& obj = *binding.get();
        const auto& indices = binding.indices().indices();
        const auto& xi = cache_.primal_vector(indices);

        VectorXd grd(obj.dim_input());
        if (obj.jacobian_x_nz_only())
            grd.resize(obj.jacobian_x_sparsity_pattern()->size());

        // Evaluate
        obj.evalJacobian(xi, grd);

        int cnt = 0;
        if (obj.jacobian_x_sparsity_pattern().has_value()) {
            // Sparse insert
            for (const auto& xy : *obj.jacobian_x_sparsity_pattern()) {
                double& entry = cache_.objective_gradient(indices[xy.first]);
                if (obj.jacobian_x_nz_only()) {
                    entry += grd[cnt++];
                } else {
                    entry += grd(xy.first);
                }
            }
        } else {
            // Perform block insert
            if (binding.indices().is_block()) {
                cache_.objective_gradient.middleRows(indices[0],
                                                     indices.size()) += grd;
            } else {
                cache_.objective_gradient(indices) += grd;
            }
        }

        VLOG(10) << "grd : " << grd.transpose();
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

    // Update caches
    std::size_t idx = 0;
    // Update caches
    for (auto& binding : constraints_) {
        auto& c = *binding.get();

        VectorXd gi(c.dim_output());

        auto indices = binding.indices().indices();
        const auto& xi = cache_.primal_vector(indices);

        c.eval(xi, gi);

        // Add constraints to vector
        // todo - add constraint indexing or permutation
        cache_.constraint_vector.middleRows(idx, c.dim_output()) = gi;
        idx += c.dim_output();
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
        bopt::profiler profiler("ipopt_program_instance: eval_jac_g");
        VLOG(10) << "eval_jac_g()";
        if (new_x) {
            std::copy_n(x, n, cache_.primal_vector.data());
        }

        int idx = 0;
        for (auto& binding : constraints_) {
            auto& c = *binding.get();
            const auto& indices = binding.indices().indices();
            Eigen::Ref<const VectorXd> xi = cache_.primal_vector(indices);

            MatrixXd jac(c.dim_output(), c.dim_tangent_space());
            if (c.jacobian_x_nz_only())
                jac.resize(c.jacobian_x_sparsity_pattern()->size(), 1);
            c.evalJacobian(xi, jac);

            VLOG(10) << "jac eval: " << jac;

            if (c.jacobian_x_sparsity_pattern().has_value()) {
                if (c.jacobian_x_nz_only()) {
                    int cnt = 0;
                    for (const auto& xy : *c.jacobian_x_sparsity_pattern()) {
                        cache_.constraint_jacobian.coeffRef(
                            idx + xy.first, indices[xy.second]) = jac(cnt++);
                    }
                } else {
                    for (const auto& xy : *c.jacobian_x_sparsity_pattern()) {
                        cache_.constraint_jacobian.coeffRef(
                            idx + xy.first, indices[xy.second]) =
                            jac(xy.first, xy.second);
                    }
                }

            } else {
                // Dense insert
                for (Index row = 0; row < c.dim_output(); ++row) {
                    for (Index col = 0; col < c.dim_tangent_space(); ++col) {
                        cache_.constraint_jacobian.coeffRef(
                            idx + row, indices[col]) = jac(row, col);
                    }
                }
            }

            idx += binding.get()->dim_output();
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
            for (Eigen::SparseMatrix<double>::InnerIterator it(
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

    } else {
        bopt::profiler profiler("ipopt_program_instance: eval_h");
        VLOG(10) << "eval_h()";
        if (new_x) {
            std::copy_n(x, n, cache_.primal_vector.data());
        }
        if (new_lambda) {
            std::copy_n(lambda, m, cache_.dual_vector.data());
        }

        for (auto& binding : costs_) {
            auto& c = *binding.get();
            const auto& indices = binding.indices().indices();
            const auto& xi = cache_.primal_vector(indices);

            MatrixXd hes(c.dim_tangent_space(), c.dim_tangent_space());
            if (c.hessian_xx_nz_only())
                hes.resize(c.hessian_xx_sparsity_pattern()->size(), 1);
            c.evalHessian(xi, Eigen::Map<VectorXd>(&obj_factor, 1), hes);
            VLOG(10) << "hes eval: " << hes;

            if (c.hessian_xx_sparsity_pattern().has_value()) {
                if (c.hessian_xx_nz_only()) {
                    int cnt = 0;
                    for (const auto& xy : *c.hessian_xx_sparsity_pattern()) {
                        cache_.lagrangian_hessian.coeffRef(
                            indices[xy.first], indices[xy.second]) +=
                            hes(cnt++);
                    }
                } else {
                    for (const auto& xy : *c.hessian_xx_sparsity_pattern()) {
                        cache_.lagrangian_hessian.coeffRef(
                            indices[xy.first], indices[xy.second]) +=
                            hes(xy.first, xy.second);
                    }
                }

            } else {
                // Dense insert
                for (Index row = 0; row < c.dim_tangent_space(); ++row) {
                    for (Index col = 0; col <= row; ++col) {
                        cache_.constraint_jacobian.coeffRef(
                            indices[row], indices[col]) += hes(row, col);
                    }
                }
            }
        }

        int idx = 0;
        for (auto& binding : constraints_) {
            auto& c = *binding.get();
            const auto& indices = binding.indices().indices();
            const auto& xi = cache_.primal_vector(indices);
            const auto& li = cache_.dual_vector.middleRows(idx, c.dim_output());

            MatrixXd hes(c.dim_output(), c.dim_tangent_space());
            if (c.hessian_xx_nz_only())
                hes.resize(c.hessian_xx_sparsity_pattern()->size(), 1);
            c.evalHessian(xi, li, hes);

            VLOG(10) << "hes eval: " << hes;

            if (c.hessian_xx_sparsity_pattern().has_value()) {
                if (c.hessian_xx_nz_only()) {
                    int cnt = 0;
                    for (const auto& xy : *c.hessian_xx_sparsity_pattern()) {
                        cache_.constraint_jacobian.coeffRef(
                            indices[xy.first], indices[xy.second]) = hes(cnt++);
                    }
                } else {
                    for (const auto& xy : *c.hessian_xx_sparsity_pattern()) {
                        cache_.constraint_jacobian.coeffRef(
                            indices[xy.first], indices[xy.second]) =
                            hes(xy.first, xy.second);
                    }
                }

            } else {
                // Dense insert
                for (Index row = 0; row < c.dim_tangent_space(); ++row) {
                    for (Index col = 0; col <= row; ++col) {
                        cache_.constraint_jacobian.coeffRef(
                            indices[row], indices[col]) += hes(row, col);
                    }
                }
            }
            idx += binding.get()->dim_output();
        }
        return true;
    }
}

bool ipopt_program_instance::get_bounds_info(Index n, Number* x_l, Number* x_u,
                                             Index m, Number* g_l,
                                             Number* g_u) {
    VLOG(10) << "get_bounds_info()";

    // Variable bounds
    cache_.variables_lower_bound = program().variables_lower_bound();
    cache_.variables_upper_bound = program().variables_upper_bound();

    // Bounding box constraints
    for (const auto& b : program().BoundingBoxConstraints()) {
        const auto& indices = b.indices().indices();
        for (int i = 0; i < indices.size(); ++i) {
            int idx = indices[i];
            cache_.variables_lower_bound[idx] = std::max(
                b.get()->lowerBound()[i], cache_.variables_lower_bound[idx]);
            cache_.variables_upper_bound[idx] = std::min(
                b.get()->upperBound()[i], cache_.variables_upper_bound[idx]);
        }
    }

    VLOG(10) << cache_.variables_lower_bound.transpose();
    VLOG(10) << cache_.variables_upper_bound.transpose();

    std::copy_n(cache_.variables_lower_bound.data(), n, x_l);
    std::copy_n(cache_.variables_upper_bound.data(), n, x_u);

    // Constraint bounds
    int cnt = 0;
    for (auto& binding : constraints_) {
        cache_.constraint_lower_bound.middleRows(cnt,
                                                 binding.get()->dim_output())
            << binding.get()->lowerBound();

        cache_.constraint_upper_bound.middleRows(cnt,
                                                 binding.get()->dim_output())
            << binding.get()->upperBound();
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
    VLOG(10) << "x0: " << program().variables_initial_value().transpose();

    assert(init_z == false);
    assert(init_lambda == false);

    if (init_x) {
        std::copy_n(program().variables_initial_value().data(), n, x);
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
        VLOG(10) << x[i];
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
    status = app_->OptimizeTNLP(nlp_);

    if (status == Ipopt::ApplicationReturnStatus::Solve_Succeeded) {
        LOG(INFO) << "*** The problem solved!" << std::endl;
    } else {
        LOG(INFO) << "*** The problem FAILED!" << std::endl;
    }

    return (int)status;
}

}  // namespace solvers
}  // namespace bopt
