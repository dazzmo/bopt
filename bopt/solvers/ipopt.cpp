#include "bopt/solvers/ipopt.hpp"

#include "bopt/logging.hpp"

namespace bopt {
namespace solvers {

ipopt_program_instance::ipopt_program_instance(MathematicalProgram& program)
    : Ipopt::TNLP(),
      program_(program),
      cache_(program.n_variables(), program.n_constraints()) {
    // Create data
    costs_ = program.get_all_costs();
    constraints_ = program.getAllConstraints();

    VLOG(10) << "Data";
    cost_data_.reserve(costs_.size());
    constraint_data_.reserve(constraints_.size());
    VLOG(10) << "Data Initialisation";
    for (const auto& c : costs_) {
        VLOG(10) << *c.get();
        cost_data_.push_back(CostData(*c.get()));
        VLOG(10) << "Done";
    }
    for (const auto& c : constraints_) {
        VLOG(10) << *c.get();
        constraint_data_.push_back(ConstraintData(*c.get()));
        VLOG(10) << "Done";
    }

    VLOG(10) << "Constraint Jacobian";
    // Construct constraint jacobian and lagrangian
    int idx = 0;
    int i = 0;
    std::vector<Eigen::Triplet<double>> triplets;
    for (auto& b : constraints_) {
        auto& c = *b.get();
        const auto& cdata = constraint_data_[i];
        VLOG(10) << c;
        if (cdata.Jx_s.nonZeros()) {
            for (int k = 0; k < cdata.Jx_s.outerSize(); ++k) {
                for (SparseMatrix<double>::InnerIterator it(cdata.Jx_s, k); it;
                     ++it) {
                    triplets.push_back(Eigen::Triplet<double>(
                        idx + it.row(), b.indices().indices()[it.col()]));
                }
            }
        } else {
            // Dense output - currently use block insert
            for (Index row = 0; row < c.dim_output(); ++row) {
                for (Index col = 0; col < c.dim_tangent_space(); ++col) {
                    triplets.push_back(Eigen::Triplet<double>(
                        idx + row, b.indices().indices()[col]));
                }
            }
        }
        i++;
        idx += c.dim_output();
    }
    cache_.constraint_jacobian.setFromTriplets(triplets.begin(),
                                               triplets.end());
    cache_.constraint_jacobian.makeCompressed();
    // Assemble look-up map for indices
    for (int k = 0; k < cache_.constraint_jacobian.outerSize(); ++k) {
        int inner_nz_cnt = 0;
        for (SparseMatrix<double>::InnerIterator it(cache_.constraint_jacobian,
                                                    k);
             it; ++it) {
            jac_nnz_map_.insert(
                {{it.row(), it.col()},
                 cache_.constraint_jacobian.outerIndexPtr()[it.outer()] +
                     inner_nz_cnt++});
        }
    }

    VLOG(10) << cache_.constraint_jacobian;

    // Construct lagrangian hessian
    i = 0;
    triplets.clear();
    VLOG(10) << "Lagrangian Hessian";
    for (auto& b : costs_) {
        auto& c = *b.get();
        const auto& cdata = cost_data_[i];
        VLOG(10) << c;
        if (cdata.Hxx_s.nonZeros()) {
            for (int k = 0; k < cdata.Hxx_s.outerSize(); ++k) {
                for (SparseMatrix<double>::InnerIterator it(cdata.Hxx_s, k); it;
                     ++it) {
                    triplets.push_back(Eigen::Triplet<double>(
                        b.indices().indices()[it.row()],
                        b.indices().indices()[it.col()]));
                }
            }
        } else {
            // Dense output - currently use block insert
            for (Index row = 0; row < c.dim_tangent_space(); ++row) {
                for (Index col = 0; col <= row; ++col) {
                    triplets.push_back(
                        Eigen::Triplet<double>(b.indices().indices()[row],
                                               b.indices().indices()[col]));
                }
            }
        }
        i++;
    }

    i = 0;
    for (auto& b : constraints_) {
        auto& c = *b.get();
        const auto& cdata = constraint_data_[i];
        VLOG(10) << c;
        if (cdata.Hxx_s.nonZeros()) {
            for (int k = 0; k < cdata.Hxx_s.outerSize(); ++k) {
                for (SparseMatrix<double>::InnerIterator it(cdata.Hxx_s, k); it;
                     ++it) {
                    triplets.push_back(Eigen::Triplet<double>(
                        b.indices().indices()[it.row()],
                        b.indices().indices()[it.col()]));
                }
            }
        } else {
            // Dense output - currently use block insert
            for (Index row = 0; row < c.dim_tangent_space(); ++row) {
                for (Index col = 0; col <= row; ++col) {
                    triplets.push_back(
                        Eigen::Triplet<double>(b.indices().indices()[row],
                                               b.indices().indices()[col]));
                }
            }
        }
        i++;
    }

    // Convert to compressed form
    cache_.lagrangian_hessian.setFromTriplets(triplets.begin(), triplets.end());
    // Assemble look-up map for indices
    for (int k = 0; k < cache_.lagrangian_hessian.outerSize(); ++k) {
        int inner_nz_cnt = 0;
        for (SparseMatrix<double>::InnerIterator it(cache_.lagrangian_hessian,
                                                    k);
             it; ++it) {
            lag_hes_nnz_map_.insert(
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

    int i = 0;
    for (auto& binding : costs_) {
        auto& obj = *binding.get();
        const auto& indices = binding.indices().indices();
        const auto& xi = cache_.primal_vector(indices);
        CostData& cdata = cost_data_[i];

        // Evaluate objective
        binding.get()->eval(xi, cdata);
        cache_.objective += obj.scaling_factor() * cdata.f;
        i++;
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

    int i = 0;
    for (auto& binding : costs_) {
        auto& obj = *binding.get();
        const auto& indices = binding.indices().indices();
        const auto& xi = cache_.primal_vector(indices);

        CostData& cdata = cost_data_[i];

        if (cdata.gx_s.nonZeros()) {
            binding.get()->evalSparseGradients(xi, cdata, true, false);
            for (int k = 0; k < cdata.gx_s.outerSize(); ++k) {
                for (SparseVector<double>::InnerIterator it(cdata.gx_s, k); it;
                     ++it) {
                    VLOG(10)
                        << it.row() << " " << it.col() << " " << it.index();

                    cache_.objective_gradient[indices[it.row()]] +=
                        obj.scaling_factor() * it.value();
                }
            }
            VLOG(10) << "grd : " << cdata.gx_s.transpose();

        } else {
            VLOG(10) << "Eval dense grad_f";
            // Evaluate objective gradient
            binding.get()->evalGradients(xi, cdata, true, false);
            cache_.objective_gradient(indices) +=
                obj.scaling_factor() * cdata.gx;
            VLOG(10) << "grd : " << cdata.gx.transpose();
        }
        i++;
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
    int i = 0;
    for (auto& binding : constraints_) {
        auto& con = *binding.get();
        const auto& indices = binding.indices().indices();
        const auto& xi = cache_.primal_vector(indices);

        ConstraintData& cdata = constraint_data_[i];

        // Evaluate constraint
        binding.get()->eval(xi, cdata);
        cache_.constraint_vector.middleRows(idx, con.dim_output()) = cdata.y;

        i++;
        VLOG(10) << "gi : " << cdata.y.transpose();
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

        // Update caches
        std::size_t idx = 0;
        // Update caches
        int i = 0;
        for (auto& binding : constraints_) {
            auto& con = *binding.get();
            const auto& indices = binding.indices().indices();
            const auto& xi = cache_.primal_vector(indices);

            ConstraintData& cdata = constraint_data_[i];

            // Evaluate constraint jacobian
            if (cdata.Jx_s.nonZeros()) {
                binding.get()->evalSparseJacobians(xi, cdata, true, false);
                for (int k = 0; k < cdata.Jx_s.outerSize(); ++k) {
                    for (SparseMatrix<double>::InnerIterator it(cdata.Jx_s, k);
                         it; ++it) {
                        VLOG(10)
                            << it.row() << " " << it.col() << " " << it.index();
                        VLOG(10) << jac_nnz_map_.at(
                            {idx + it.row(), indices[it.col()]});
                        cache_.constraint_jacobian.valuePtr()[jac_nnz_map_.at(
                            {idx + it.row(), indices[it.col()]})] = it.value();
                    }
                }
            } else {
                binding.get()->evalJacobians(xi, cdata, true, false);
                for (Index row = 0; row < con.dim_output(); ++row) {
                    for (Index col = 0; col < con.dim_tangent_space(); ++col) {
                        cache_.constraint_jacobian.valuePtr()[jac_nnz_map_.at(
                            {idx + row, indices[col]})] = cdata.Jx(row, col);
                    }
                }
            }

            i++;
            idx += con.dim_output();
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
        int i = 0;
        for (auto& binding : costs_) {
            auto& con = *binding.get();
            const auto& indices = binding.indices().indices();
            const auto& xi = cache_.primal_vector(indices);

            CostData& cdata = cost_data_[i];

            // Evaluate constraint jacobian
            if (cdata.Hxx_s.nonZeros()) {
                binding.get()->evalSparseHessians(xi, cdata, true, false);
                for (int k = 0; k < cdata.Hxx_s.outerSize(); ++k) {
                    for (SparseMatrix<double>::InnerIterator it(cdata.Hxx_s, k);
                         it; ++it) {
                        cache_.lagrangian_hessian
                            .valuePtr()[lag_hes_nnz_map_.at(
                                {indices[it.row()], indices[it.col()]})] +=
                            obj_factor * it.value();
                    }
                }
            } else {
                binding.get()->evalHessians(xi, cdata, true, false);
                for (Index row = 0; row < con.dim_input(); ++row) {
                    for (Index col = 0; col < row; ++col) {
                        cache_.lagrangian_hessian
                            .valuePtr()[lag_hes_nnz_map_.at(
                                {indices[row], indices[col]})] +=
                            obj_factor * cdata.Hxx(row, col);
                    }
                }
            }
            i++;
        }
        // Constraints
        int idx = 0;
        i = 0;
        for (auto& binding : constraints_) {
            auto& con = *binding.get();
            const auto& indices = binding.indices().indices();
            const auto& xi = cache_.primal_vector(indices);
            const auto& li =
                cache_.dual_vector.middleRows(idx, con.dim_output());

            ConstraintData& cdata = constraint_data_[i];

            // Evaluate constraint jacobian
            if (cdata.Jx_s.nonZeros()) {
                binding.get()->evalSparseHessians(xi, li, cdata, true, false);
                for (int k = 0; k < cdata.Hxx_s.outerSize(); ++k) {
                    for (SparseMatrix<double>::InnerIterator it(cdata.Hxx_s, k);
                         it; ++it) {
                        cache_.lagrangian_hessian
                            .valuePtr()[lag_hes_nnz_map_.at(
                                {indices[it.row()], indices[it.col()]})] +=
                            it.value();
                    }
                }
            } else {
                binding.get()->evalHessians(xi, li, cdata, true, false);
                for (Index row = 0; row < con.dim_input(); ++row) {
                    for (Index col = 0; col < row; ++col) {
                        cache_.lagrangian_hessian
                            .valuePtr()[lag_hes_nnz_map_.at(
                                {indices[row], indices[col]})] +=
                            cdata.Hxx(row, col);
                    }
                }
            }

            i++;
            idx += con.dim_output();
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

    // Bounding box constraints
    for (const auto& b : program().boundingBoxConstraints()) {
        auto& con = *b.get();
        const auto& indices = b.indices().indices();
        // for (int i = 0; i < indices.size(); ++i) {
        //     int idx = indices[i];
        //     cache_.variables_lower_bound[idx] = std::max(
        //         b.get()->lowerBound()[i], cache_.variables_lower_bound[idx]);
        //     cache_.variables_upper_bound[idx] = std::min(
        //         b.get()->upperBound()[i], cache_.variables_upper_bound[idx]);
        // }
    }

    VLOG(10) << cache_.variables_lower_bound.transpose();
    VLOG(10) << cache_.variables_upper_bound.transpose();

    std::copy_n(cache_.variables_lower_bound.data(), n, x_l);
    std::copy_n(cache_.variables_upper_bound.data(), n, x_u);

    // Constraint bounds
    int i = 0;
    int cnt = 0;
    for (auto& binding : constraints_) {
        Constraint& con = *binding.get();
        ConstraintData& cdata = constraint_data_[i];
        con.evalBounds(cdata);
        cache_.constraint_lower_bound.middleRows(cnt, con.dim_output())
            << cdata.lb;
        cache_.constraint_upper_bound.middleRows(cnt, con.dim_output())
            << cdata.ub;
        i++;
        cnt += con.dim_output();
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
