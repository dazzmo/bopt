#ifndef SOLVERS_IPOPT_H
#define SOLVERS_IPOPT_H

#include <boost/functional/hash.hpp>
#include <coin-or/IpIpoptApplication.hpp>
#include <coin-or/IpTNLP.hpp>

#include "bopt/logging.hpp"
#include "bopt/profiler.hpp"
#include "bopt/program.hpp"
#include "bopt/solvers/base.hpp"

namespace bopt {
namespace solvers {

struct ipopt_data {
    ipopt_data(const bopt_index& n, const bopt_index& m) {
        primal_vector = VectorXd::Zero(n);
        dual_vector = VectorXd::Zero(m);
        variables_lower_bound =
            VectorXd::Constant(n, -std::numeric_limits<double>::max());
        variables_upper_bound =
            VectorXd::Constant(n, std::numeric_limits<double>::max());

        objective_gradient = VectorXd::Zero(n);

        constraint_vector = VectorXd::Zero(m);
        constraint_lower_bound = VectorXd::Zero(m);
        constraint_upper_bound = VectorXd::Zero(m);

        constraint_jacobian.resize(m, n);
        lagrangian_hessian.resize(n, n);
    }

    VectorXd primal_vector;
    VectorXd dual_vector;

    double objective;
    VectorXd objective_gradient;

    SparseMatrix<double> lagrangian_hessian;

    VectorXd constraint_vector;
    SparseMatrix<double> constraint_jacobian;

    VectorXd constraint_lower_bound;
    VectorXd constraint_upper_bound;

    VectorXd variables_lower_bound;
    VectorXd variables_upper_bound;
};

class ipopt_program_instance : public Ipopt::TNLP {
    using Index = Ipopt::Index;
    using Number = Ipopt::Number;

   public:
    ipopt_program_instance(MathematicalProgram& program);

    ~ipopt_program_instance() { VLOG(10) << "Destructing!"; }

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
    ipopt_data cache_;

    std::vector<Binding<Cost>> costs_;
    std::vector<Binding<Constraint>> constraints_;

    std::vector<CostData> cost_data_;
    std::vector<ConstraintData> constraint_data_;

    typedef std::pair<int, int> SparseMatrixIndices;

    // Hash for pairs of ints (to allow hashtable for (x,y) indices in sparse matrices)
    struct hash_pair {
        std::size_t operator()(const SparseMatrixIndices& p) const {
            std::size_t seed = 0;
            boost::hash_combine(seed, p.first);
            boost::hash_combine(seed, p.second);
            return seed;
        }
    };

    std::unordered_map<SparseMatrixIndices, int, hash_pair> jac_nnz_map_;
    std::unordered_map<SparseMatrixIndices, int, hash_pair> lag_hes_nnz_map_;

    MathematicalProgram& program_;
    MathematicalProgram& program() { return program_; }
};

class ipopt_solver : public solver<double> {
   public:
    ipopt_solver(MathematicalProgram& program);
    int solve();

    Ipopt::SmartPtr<Ipopt::OptionsList> options() { return app_->Options(); }

   private:
    Ipopt::SmartPtr<Ipopt::TNLP> nlp_;
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app_;
};

}  // namespace solvers
}  // namespace bopt

#endif /* SOLVERS_IPOPT_H */
