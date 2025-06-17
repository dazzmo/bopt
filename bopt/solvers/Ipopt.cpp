#include "bopt/solvers/Ipopt.hpp"

#include "bopt/Logging.hpp"

namespace bopt {
namespace solvers {
namespace internal {
IpoptProgramInstance::IpoptProgramInstance(MathematicalProgram& program)
    : Ipopt::TNLP(),
      program_(program),
      cache_(program.numVariables(), program.numConstraints()),
      primal_solution_(VectorX::Zero(program.numVariables())) {
    // Create data
    dense_costs_ =
        program.getCostBindings<CostTpl<Real, SparsityType::DENSE>>();
    sparse_costs_ =
        program.getCostBindings<CostTpl<Real, SparsityType::SPARSE>>();

    dense_constraints_ =
        program
            .getConstraintBindings<ConstraintTpl<Real, SparsityType::DENSE>>();
    sparse_constraints_ =
        program
            .getConstraintBindings<ConstraintTpl<Real, SparsityType::SPARSE>>();

    // Create the jacobian for the problem
    // Logger::debug() << "Constraint Jacobian";
    // Construct constraint jacobian and lagrangian
    Index c_idx = 0;
    std::vector<Eigen::Triplet<Number>> triplets;
    // Dense constraint jacobian
    for (auto& b : dense_constraints_) {
        const auto& c = *b.get();
        const auto& indices = b.getIndexManager().getIndices();
        for (Index row = 0; row < c.outputSize(); ++row) {
            for (Index col = 0; col < c.dimInputTangentSpace(); ++col) {
                triplets.push_back(
                    Eigen::Triplet<Number>(c_idx + row, indices[col]));
            }
        }
        c_idx += c.outputSize();
    }
    // Sparse constraint jacobians
    for (auto& b : sparse_constraints_) {
        const auto& c = *b.get();
        auto& d = *b.getData();
        const auto& indices = b.getIndexManager().getIndices();
        for (int k = 0; k < d.Jx.outerSize(); ++k) {
            for (SparseMatrix::InnerIterator it(d.Jx, k); it; ++it) {
                triplets.push_back(Eigen::Triplet<Number>(c_idx + it.row(),
                                                          indices[it.col()]));
            }
        }
        c_idx += c.outputSize();
    }

    cache_.constraint_jacobian.setFromTriplets(triplets.begin(),
                                               triplets.end());
    cache_.constraint_jacobian.makeCompressed();

    // Assemble look-up map for indices
    for (Index k = 0; k < cache_.constraint_jacobian.outerSize(); ++k) {
        Index inner_nz_cnt = 0;
        for (SparseMatrix::InnerIterator it(cache_.constraint_jacobian, k); it;
             ++it) {
            jac_nz_map_.insert(
                {{it.row(), it.col()},
                 cache_.constraint_jacobian.outerIndexPtr()[it.outer()] +
                     inner_nz_cnt++});
        }
    }

    // Logger::debug() << cache_.constraint_jacobian;

    // Construct lagrangian hessian
    triplets.clear();
    // Logger::debug() << "Lagrangian Hessian";
    for (auto& b : dense_costs_) {
        const auto& c = *b.get();
        auto& d = *b.getData();
        const auto& indices = b.getIndexManager().getIndices();
        // Dense output - currently use block insert
        for (Index row = 0; row < c.dimInputTangentSpace(); ++row) {
            for (Index col = 0; col <= row; ++col) {
                triplets.push_back(
                    Eigen::Triplet<Number>(indices[row], indices[col]));
            }
        }
    }
    for (auto& b : dense_constraints_) {
        const auto& c = *b.get();
        auto& d = *b.getData();
        const auto& indices = b.getIndexManager().getIndices();
        // Dense output - currently use block insert
        for (Index row = 0; row < c.dimInputTangentSpace(); ++row) {
            for (Index col = 0; col <= row; ++col) {
                triplets.push_back(
                    Eigen::Triplet<Number>(indices[row], indices[col]));
            }
        }
    }

    for (auto& b : sparse_costs_) {
        const auto& c = *b.get();
        auto& d = *b.getData();
        const auto& indices = b.getIndexManager().getIndices();
        // Dense output - currently use block insert
        for (int k = 0; k < d.Hxx.outerSize(); ++k) {
            for (SparseMatrix::InnerIterator it(d.Hxx, k); it; ++it) {
                triplets.push_back(Eigen::Triplet<Number>(indices[it.row()],
                                                          indices[it.col()]));
            }
        }
    }
    for (auto& b : sparse_constraints_) {
        const auto& c = *b.get();
        auto& d = *b.getData();
        const auto& indices = b.getIndexManager().getIndices();
        for (int k = 0; k < d.Hxx.outerSize(); ++k) {
            for (SparseMatrix::InnerIterator it(d.Hxx, k); it; ++it) {
                triplets.push_back(Eigen::Triplet<Number>(indices[it.row()],
                                                          indices[it.col()]));
            }
        }
    }

    // Convert to compressed form
    cache_.lagrangian_hessian.setFromTriplets(triplets.begin(), triplets.end());
    // Assemble look-up map for indices
    for (Index k = 0; k < cache_.lagrangian_hessian.outerSize(); ++k) {
        Index inner_nz_cnt = 0;
        for (SparseMatrix::InnerIterator it(cache_.lagrangian_hessian, k); it;
             ++it) {
            lag_hes_nz_map_.insert(
                {{it.row(), it.col()},
                 cache_.lagrangian_hessian.outerIndexPtr()[it.outer()] +
                     inner_nz_cnt++});
        }
    }
    // Logger::debug() << cache_.lagrangian_hessian;
}

bool IpoptProgramInstance::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                                        Index& nnz_h_lag,
                                        IndexStyleEnum& index_style) {
    // Logger::debug() << "get_nlp_info()";
    n = program().numVariables();
    m = program().numConstraints();

    // Logger::debug() << "n = " << n;
    // Logger::debug() << "m = " << m;

    nnz_jac_g = cache_.constraint_jacobian.nonZeros();
    nnz_h_lag = cache_.lagrangian_hessian.nonZeros();

    index_style = TNLP::C_STYLE;

    return true;
}

bool IpoptProgramInstance::eval_f(Index n, const Number* x, bool new_x,
                                  Number& obj_value) {
    bopt::Profiler profiler("IpoptProgramInstance::eval_f");
    // Logger::debug() << "eval_f()";

    if (new_x) {
        std::copy_n(x, n, cache_.primal_vector.data());
    }

    // Update caches
    cache_.objective = 0.0;

    // Dense costs
    for (auto& binding : dense_costs_) {
        auto& c = *binding.get();
        auto& d = *binding.getData();
        const auto& indices = binding.getIndexManager().getIndices();
        const auto& xi = cache_.primal_vector(indices);

        // Evaluate objective
        // fixme - Cost scaling factor!
        c.eval(xi, d);
        cache_.objective += 1.0 * d.y;
    }

    // Sparse costs
    // todo - maybe make a function for this to avoid code repetition
    for (auto& binding : sparse_costs_) {
        auto& c = *binding.get();
        auto& d = *binding.getData();
        const auto& indices = binding.getIndexManager().getIndices();
        const auto& xi = cache_.primal_vector(indices);

        // Evaluate objective
        c.eval(xi, d);
        cache_.objective += 1.0 * d.y;
    }

    // Set objective to most recently cached value
    // Logger::debug() << "f: " << cache_.objective;
    obj_value = cache_.objective;
    return true;
}

bool IpoptProgramInstance::eval_grad_f(Index n, const Number* x, bool new_x,
                                       Number* grad_f) {
    bopt::Profiler profiler("IpoptProgramInstance: eval_grad_f");
    // Logger::debug() << "eval_grad_f()";

    if (new_x) {
        std::copy_n(x, n, cache_.primal_vector.data());
    }

    // Update caches
    cache_.objective_gradient.setZero();

    // Dense costs
    for (auto& binding : dense_costs_) {
        auto& c = *binding.get();
        auto& d = *binding.getData();
        const auto& indices = binding.getIndexManager().getIndices();
        const auto& xi = cache_.primal_vector(indices);

        // Evaluate objective
        c.evalGradients(xi, d, GradientEvaluationFlags(true, false));
        cache_.objective_gradient(indices) += 1.0 * d.gx;
    }

    // Sparse costs
    // todo - maybe make a function for this to avoid code repetition
    for (auto& binding : sparse_costs_) {
        auto& c = *binding.get();
        auto& d = *binding.getData();
        const auto& indices = binding.getIndexManager().getIndices();
        const auto& xi = cache_.primal_vector(indices);

        c.evalGradients(xi, d, GradientEvaluationFlags(true, false));

        for (int k = 0; k < d.gx.outerSize(); ++k) {
            for (SparseVector::InnerIterator it(d.gx, k); it; ++it) {
                cache_.objective_gradient[indices[it.row()]] +=
                    1.0 * it.value();
            }
        }
    }

    // TODO - See about mapping these
    // Logger::debug() << "grad_f : " << cache_.objective_gradient.transpose();
    std::copy_n(cache_.objective_gradient.data(), n, grad_f);
    return true;
}

bool IpoptProgramInstance::eval_g(Index n, const Number* x, bool new_x, Index m,
                                  Number* g) {
    bopt::Profiler profiler("IpoptProgramInstance::eval_g");
    // Logger::debug() << "eval_g()";
    if (new_x) {
        std::copy_n(x, n, cache_.primal_vector.data());
    }

    Index c_idx = 0;
    // Dense constraints
    for (auto& binding : dense_constraints_) {
        auto& c = *binding.get();
        auto& d = *binding.getData();
        const auto& indices = binding.getIndexManager().getIndices();
        const auto& xi = cache_.primal_vector(indices);

        c.eval(xi, d);
        cache_.constraint_vector.middleRows(c_idx, c.outputSize()) = d.y;
        c_idx += c.outputSize();
    }

    // Sparse constraints
    // todo - maybe make a function for this to avoid code repetition
    for (auto& binding : sparse_constraints_) {
        auto& c = *binding.get();
        auto& d = *binding.getData();
        const auto& indices = binding.getIndexManager().getIndices();
        const auto& xi = cache_.primal_vector(indices);

        c.eval(xi, d);
        cache_.constraint_vector.middleRows(c_idx, c.outputSize()) = d.y;
        c_idx += c.outputSize();
    }

    // Logger::debug() << "c : " << cache_.constraint_vector.transpose();
    std::copy_n(cache_.constraint_vector.data(), m, g);
    return true;
};

bool IpoptProgramInstance::eval_jac_g(Index n, const Number* x, bool new_x,
                                      Index m, Index nele_jac, Index* iRow,
                                      Index* jCol, Number* values) {
    if (values == NULL) {
        // Return the sparsity of the constraint Jacobian
        int cnt = 0;
        for (int k = 0; k < cache_.constraint_jacobian.outerSize(); ++k) {
            for (SparseMatrix::InnerIterator it(cache_.constraint_jacobian, k);
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
        bopt::Profiler profiler("IpoptProgramInstance: eval_jac_g");
        // Logger::debug() << "eval_jac_g()";
        if (new_x) {
            std::copy_n(x, n, cache_.primal_vector.data());
        }

        // Update caches
        Index c_idx = 0;
        // Dense constraints
        for (auto& binding : dense_constraints_) {
            auto& c = *binding.get();
            auto& d = *binding.getData();
            const auto& indices = binding.getIndexManager().getIndices();
            const auto& xi = cache_.primal_vector(indices);

            c.evalJacobians(xi, d, JacobianEvaluationFlags(true, false));
            for (Index row = 0; row < c.outputSize(); ++row) {
                for (Index col = 0; col < c.dimInputTangentSpace(); ++col) {
                    cache_.constraint_jacobian.valuePtr()[jac_nz_map_.at(
                        {c_idx + row, indices[col]})] = d.Jx(row, col);
                }
            }
            c_idx += c.outputSize();
        }

        // Sparse constraints
        // todo - maybe make a function for this to avoid code repetition
        for (auto& binding : sparse_constraints_) {
            auto& c = *binding.get();
            auto& d = *binding.getData();
            const auto& indices = binding.getIndexManager().getIndices();
            const auto& xi = cache_.primal_vector(indices);

            c.evalJacobians(xi, d, JacobianEvaluationFlags(true, false));

            for (int k = 0; k < d.Jx.outerSize(); ++k) {
                for (SparseMatrix::InnerIterator it(d.Jx, k); it; ++it) {
                    cache_.constraint_jacobian.valuePtr()[jac_nz_map_.at(
                        {c_idx + it.row(), indices[it.col()]})] = it.value();
                }
            }
            c_idx += c.outputSize();
        }

        // Update caches
        // Logger::debug() << "jac : " << cache_.constraint_jacobian;
        std::copy_n(cache_.constraint_jacobian.valuePtr(), nele_jac, values);
        // Logger::debug() << "finished";
    }
    return true;
}

bool IpoptProgramInstance::eval_h(Index n, const Number* x, bool new_x,
                                  Number obj_factor, Index m,
                                  const Number* lambda, bool new_lambda,
                                  Index nele_hess, Index* iRow, Index* jCol,
                                  Number* values) {
    // Logger::debug() << "eval_h()";
    if (values == NULL) {
        // Return the sparsity of the constraint Jacobian
        int cnt = 0;
        for (int k = 0; k < cache_.lagrangian_hessian.outerSize(); ++k) {
            for (SparseMatrix::InnerIterator it(cache_.lagrangian_hessian, k);
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
        bopt::Profiler profiler("IpoptProgramInstance: eval_h");
        // Logger::debug() << "eval_h()";
        if (new_x) {
            std::copy_n(x, n, cache_.primal_vector.data());
        }
        if (new_lambda) {
            std::copy_n(lambda, m, cache_.dual_vector.data());
        }

        // Costs
        // Dense costs
        // Logger::debug() << "dense cost";
        for (auto& binding : dense_costs_) {
            auto& c = *binding.get();
            auto& d = *binding.getData();
            const auto& indices = binding.getIndexManager().getIndices();
            const auto& xi = cache_.primal_vector(indices);

            c.evalHessians(xi, d, HessianEvaluationFlags(true, false, false));
            for (Index row = 0; row < c.dimInputTangentSpace(); ++row) {
                for (Index col = 0; col < row; ++col) {
                    cache_.lagrangian_hessian.valuePtr()[lag_hes_nz_map_.at(
                        {indices[row], indices[col]})] +=
                        obj_factor * d.Hxx(row, col);
                }
            }
        }

        // Sparse costs
        // Logger::debug() << "sparse cost";
        // todo - maybe make a function for this to avoid code repetition
        for (auto& binding : sparse_costs_) {
            auto& c = *binding.get();
            auto& d = *binding.getData();
            const auto& indices = binding.getIndexManager().getIndices();
            const auto& xi = cache_.primal_vector(indices);

            c.evalHessians(xi, d, HessianEvaluationFlags(true, false, false));
            for (int k = 0; k < d.Hxx.outerSize(); ++k) {
                for (SparseMatrix::InnerIterator it(d.Hxx, k); it; ++it) {
                    cache_.lagrangian_hessian.valuePtr()[lag_hes_nz_map_.at(
                        {indices[it.row()], indices[it.col()]})] +=
                        obj_factor * it.value();
                }
            }
        }

        // Constraints
        Index c_idx = 0;
        // Dense constraints
        // Logger::debug() << "dense constraint";
        for (auto& binding : dense_constraints_) {
            auto& c = *binding.get();
            auto& d = *binding.getData();
            const auto& indices = binding.getIndexManager().getIndices();
            const auto& xi = cache_.primal_vector(indices);
            const auto& li =
                cache_.dual_vector.middleRows(c_idx, c.outputSize());

            c.evalHessians(xi, li, d,
                           HessianEvaluationFlags(true, false, false));

            for (Index row = 0; row < c.dimInputTangentSpace(); ++row) {
                for (Index col = 0; col < row; ++col) {
                    cache_.lagrangian_hessian.valuePtr()[lag_hes_nz_map_.at(
                        {indices[row], indices[col]})] = d.Hxx(row, col);
                }
            }
            c_idx += c.outputSize();
        }

        // Sparse constraints
        // Logger::debug() << "sparse constraint";
        // todo - maybe make a function for this to avoid code repetition
        for (auto& binding : sparse_constraints_) {
            auto& c = *binding.get();
            auto& d = *binding.getData();
            const auto& indices = binding.getIndexManager().getIndices();
            const auto& xi = cache_.primal_vector(indices);
            const auto& li =
                cache_.dual_vector.middleRows(c_idx, c.outputSize());

            c.evalHessians(xi, li, d,
                           HessianEvaluationFlags(true, false, false));

            for (int k = 0; k < d.Hxx.outerSize(); ++k) {
                for (SparseMatrix::InnerIterator it(d.Hxx, k); it; ++it) {
                    cache_.lagrangian_hessian.valuePtr()[lag_hes_nz_map_.at(
                        {indices[it.row()], indices[it.col()]})] +=
                        obj_factor * it.value();
                }
            }
            c_idx += c.outputSize();
        }

        // Update caches
        // Logger::debug() << "hes : " << cache_.lagrangian_hessian;
        std::copy_n(cache_.lagrangian_hessian.valuePtr(), nele_hess, values);
        // Logger::debug() << "finished";
        return true;
    }
}

bool IpoptProgramInstance::get_bounds_info(Index n, Number* x_l, Number* x_u,
                                           Index m, Number* g_l, Number* g_u) {
    // Logger::debug() << "get_bounds_info()";

    // Variable bounds
    cache_.variables_lower_bound = program().variableLowerBounds();
    cache_.variables_upper_bound = program().variableUpperBounds();

    auto bb = program_.getBoundingBoxConstraintBindings();
    for (auto& binding : bb) {
        const auto& c = *binding.get();
        const auto& indices = binding.getIndexManager().getIndices();

        VectorX lb(c.outputSize()), ub(c.outputSize());
        c.evalBoundingBoxBounds(lb, ub);

        cache_.variables_lower_bound(indices).array() =
            cache_.variables_lower_bound(indices).array().max(lb.array());

        cache_.variables_upper_bound(indices).array() =
            cache_.variables_upper_bound(indices).array().min(ub.array());
    }

    // Logger::debug() << cache_.variables_lower_bound.transpose();
    // Logger::debug() << cache_.variables_upper_bound.transpose();

    std::copy_n(cache_.variables_lower_bound.data(), n, x_l);
    std::copy_n(cache_.variables_upper_bound.data(), n, x_u);

    // Constraint bounds
    Index c_idx = 0;
    for (auto& binding : dense_constraints_) {
        const auto& c = *binding.get();
        c.evalBounds(
            cache_.constraint_lower_bound.middleRows(c_idx, c.outputSize()),
            cache_.constraint_upper_bound.middleRows(c_idx, c.outputSize()));
        c_idx += c.outputSize();
    }

    // Logger::debug() << cache_.constraint_lower_bound.transpose();
    // Logger::debug() << cache_.constraint_upper_bound.transpose();

    std::copy_n(cache_.constraint_lower_bound.data(), m, g_l);
    std::copy_n(cache_.constraint_upper_bound.data(), m, g_u);

    return true;
}

bool IpoptProgramInstance::get_starting_point(Index n, bool init_x, Number* x,
                                              bool init_z, Number* z_L,
                                              Number* z_U, Index m,
                                              bool init_lambda,
                                              Number* lambda) {
    // Logger::debug() << "get_starting_point()";
    // Logger::debug() << "x0: " <<
    // program().variableInitialValues().transpose();

    assert(init_z == false);
    assert(init_lambda == false);

    if (init_x) {
        std::copy_n(program().variableInitialValues().data(), n, x);
    }

    return true;
}

void IpoptProgramInstance::finalize_solution(
    Ipopt::SolverReturn status, Index n, const Number* x, const Number* z_L,
    const Number* z_U, Index m, const Number* g, const Number* lambda,
    Number obj_value, const Ipopt::IpoptData* ip_data,
    Ipopt::IpoptCalculatedQuantities* ip_cq) {
    // Logger::debug() << "finalize_solution()";

    for (Index i = 0; i < n; ++i) {
        primal_solution_[i] = x[i];
    }
}
}  // namespace internal

IpoptSolver::IpoptSolver(MathematicalProgram& program)
    : SolverBase<SolverInfoBase>(program, "IpoptSolver") {}

void IpoptSolver::initImpl() {
    // Create program instance
    instance_ = std::make_unique<internal::IpoptProgramInstance>(getProgram());

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

void IpoptSolver::solveImpl() {
    // Ask Ipopt to solve the problem
    Ipopt::ApplicationReturnStatus status;
    {
        Profiler profiler("IpoptSolver solve");
        status = app_->OptimizeTNLP(
            static_cast<Ipopt::SmartPtr<Ipopt::TNLP>>(instance_.get()));
    }

    solver_info_.success =
        (status == Ipopt::ApplicationReturnStatus::Solve_Succeeded);
}

IpoptSolver::VectorX IpoptSolver::getPrimalSolution() const {
    return instance_->getPrimalSolution();
}

}  // namespace solvers
}  // namespace bopt
