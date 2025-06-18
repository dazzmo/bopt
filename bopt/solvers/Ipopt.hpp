#ifndef SOLVERS_IPOPT_H
#define SOLVERS_IPOPT_H

#include <boost/functional/hash.hpp>
#include <coin-or/IpIpoptApplication.hpp>
#include <coin-or/IpTNLP.hpp>

#include "bopt/Logging.hpp"
#include "bopt/Profiler.hpp"
#include "bopt/Program.hpp"
#include "bopt/solvers/SolverBase.hpp"

namespace bopt {
namespace solvers {

namespace internal {

struct IpoptData {
    using Scalar = Ipopt::Number;
    using VectorX = typename MathTypes<Scalar>::VectorX;
    using MatrixX = typename MathTypes<Scalar>::MatrixX;
    using SparseMatrix = typename MathTypes<Scalar>::SparseMatrix;

    IpoptData(const Index& n, const Index& m) {
        primal_vector = VectorX::Zero(n);
        dual_vector = VectorX::Zero(m);
        variables_lower_bound =
            VectorX::Constant(n, -std::numeric_limits<Scalar>::max());
        variables_upper_bound =
            VectorX::Constant(n, std::numeric_limits<Scalar>::max());

        objective_gradient = VectorX::Zero(n);

        constraint_vector = VectorX::Zero(m);
        constraint_lower_bound = VectorX::Zero(m);
        constraint_upper_bound = VectorX::Zero(m);

        constraint_jacobian.resize(m, n);
        lagrangian_hessian.resize(n, n);
    }

    VectorX primal_vector;
    VectorX dual_vector;

    Scalar objective;
    VectorX objective_gradient;

    SparseMatrix lagrangian_hessian;

    VectorX constraint_vector;
    SparseMatrix constraint_jacobian;

    VectorX constraint_lower_bound;
    VectorX constraint_upper_bound;

    VectorX variables_lower_bound;
    VectorX variables_upper_bound;
};

class IpoptProgramInstance : public Ipopt::TNLP {
    using Index = Ipopt::Index;
    using Number = Ipopt::Number;

    using VectorX = typename MathTypes<Number>::VectorX;
    using MatrixX = typename MathTypes<Number>::MatrixX;
    using SparseMatrix = typename MathTypes<Number>::SparseMatrix;
    using SparseVector = typename MathTypes<Number>::SparseVector;

   public:
    IpoptProgramInstance(MathematicalProgram& program);

    ~IpoptProgramInstance() { VLOG(10) << "Destructing!"; }

    const SolverResultsBase& getResults() const { return results_; }

   private:
    bool get_nlp_info(Index& n, Index& m, Index& nnz_jac_g, Index& nnz_h_lag,
                      IndexStyleEnum& index_style);

    bool get_bounds_info(Index n, Number* x_l, Number* x_u, Index m,
                         Number* g_l, Number* g_u);

    bool get_starting_point(Index n, bool init_x, Number* x, bool init_z,
                            Number* z_L, Number* z_U, Index m, bool init_lambda,
                            Number* lambda);

    bool eval_f(Index n, const Number* x, bool new_x, Number& obj_value);

    bool eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f);

    bool eval_g(Index n, const Number* x, bool new_x, Index m, Number* g);

    bool eval_jac_g(Index n, const Number* x, bool new_x, Index m,
                    Index nele_jac, Index* iRow, Index* jCol, Number* values);

    bool eval_h(Index n, const Number* x, bool new_x, Number obj_factor,
                Index m, const Number* lambda, bool new_lambda, Index nele_hess,
                Index* iRow, Index* jCol, Number* values);

    void finalize_solution(Ipopt::SolverReturn status, Index n, const Number* x,
                           const Number* z_L, const Number* z_U, Index m,
                           const Number* g, const Number* lambda,
                           Number obj_value, const Ipopt::IpoptData* ip_data,
                           Ipopt::IpoptCalculatedQuantities* ip_cq);

   private:
    MathematicalProgram& program_;
    internal::IpoptData cache_;

    SolverResultsBase results_;
    VectorX primal_solution_;

    std::vector<Binding<CostTpl<Real>>> dense_costs_;
    std::vector<Binding<CostTpl<Real, SparsityType::SPARSE>>> sparse_costs_;

    std::vector<Binding<ConstraintTpl<Real>>> dense_constraints_;
    std::vector<Binding<ConstraintTpl<Real, SparsityType::SPARSE>>>
        sparse_constraints_;

    typedef std::pair<int, int> SparseMatrixIndices;

    // Hash for pairs of ints (to allow hashtable for (x,y) indices in sparse
    // matrices)
    struct hash_pair {
        std::size_t operator()(const SparseMatrixIndices& p) const {
            std::size_t seed = 0;
            boost::hash_combine(seed, p.first);
            boost::hash_combine(seed, p.second);
            return seed;
        }
    };

    /// @brief Lookup map from the non-zero entry (x, y) to its index in the
    /// nonzero vector
    std::unordered_map<SparseMatrixIndices, int, hash_pair> jac_nz_map_;
    /// @brief Lookup map from the non-zero entry (x, y) to its index in the
    /// nonzero vector
    std::unordered_map<SparseMatrixIndices, int, hash_pair> lag_hes_nz_map_;

    MathematicalProgram& program() { return program_; }
};

}  // namespace internal

class IpoptSolver : public SolverBase<SolverInfoBase> {
   public:
    using Base = SolverBase<SolverInfoBase>;
    using SolverInfo = typename Base::SolverInfo;
    using VectorX = typename Base::VectorX;

    IpoptSolver(MathematicalProgram& program);
    ~IpoptSolver() { instance_.reset(); }

    const SolverResultsBase& getResults() const override;

    const SolverInfo& getInfo() const override { return solver_info_; }

    Ipopt::SmartPtr<Ipopt::OptionsList> options() { return app_->Options(); }

   protected:
    void initImpl() override;
    void solveImpl() override;

   private:
    std::unique_ptr<internal::IpoptProgramInstance> instance_;
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app_;

    SolverInfo solver_info_;
};

}  // namespace solvers
}  // namespace bopt

#endif /* SOLVERS_IPOPT_H */
