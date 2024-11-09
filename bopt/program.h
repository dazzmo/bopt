#ifndef OPTIMISATION_PROGRAM_H
#define OPTIMISATION_PROGRAM_H

#include <casadi/casadi.hpp>

#include "bopt/ad/casadi/codegen.hpp"
#include "bopt/ad/casadi/eigen.hpp"
#include "bopt/ad/casadi/function.hpp"
#include "bopt/binding.h"
#include "bopt/common.hpp"
#include "bopt/constraints.hpp"
#include "bopt/costs.hpp"
#include "bopt/logging.hpp"
#include "bopt/profiler.hpp"

namespace bopt {

// Forward declaration of SolverBase
namespace solvers {
class SolverBase;
}

/**
 * @brief Vector of cost bindings of the form \f$ \bm c(x, p) =
 * [c_0(x, p), c_1(x, p), \hdots , c_n(x, p)]
 *
 */
class CostBindingSet {
   public:
    Index numberOfCosts() const {
        return generic_.size() + linear_.size() + quadratic_.size();
    }

    /**
     * @brief Appends a generic constraint to the set
     *
     * @param con
     * @param x
     * @param p
     * @return Binding<Cost>
     */
    void addCost(const std::shared_ptr<Cost> &con,
                 const std::vector<std::vector<Variable>> &in) {
        // Create a binding for the constraint
        generic_.push_back(Binding<Cost>(con, in));
    }

    const std::vector<Binding<Cost>> &getCosts() const { return generic_; }

    /**
     * @brief Appends a generic constraint to the set
     *
     * @param con
     * @param x
     * @param p
     * @return Binding<Cost>
     */
    void addLinearCost(const std::shared_ptr<LinearCost> &con,
                       const std::vector<std::vector<Variable>> &in) {
        // Create a binding for the constraint
        linear_.push_back(Binding<LinearCost>(con, in));
    }

    const std::vector<Binding<LinearCost>> &getLinearCosts() const {
        return linear_;
    }

    /**
     * @brief Appends a quadratic constraint to the set
     *
     * @param con
     * @param x
     * @param p
     */
    void addQuadraticCost(const std::shared_ptr<QuadraticCost> &con,
                          const std::vector<std::vector<Variable>> &in) {
        // Create a binding for the constraint
        quadratic_.push_back(Binding<QuadraticCost>(con, in));
    }

    const std::vector<Binding<QuadraticCost>> &getQuadraticCosts() const {
        return quadratic_;
    }

    std::vector<Binding<Cost>> getAllCosts() const {
        std::vector<Binding<Cost>> vec;
        vec.insert(vec.begin(), generic_.begin(), generic_.end());
        vec.insert(vec.end(), linear_.begin(), linear_.end());
        vec.insert(vec.end(), quadratic_.begin(), quadratic_.end());
        // Return vector of all costs
        return vec;
    }

   private:
    // Cost bindings
    std::vector<Binding<Cost>> generic_;
    std::vector<Binding<LinearCost>> linear_;
    std::vector<Binding<QuadraticCost>> quadratic_;
};

template <typename T>
bopt::CompressedColumnStorageFormat getJacobianSparsityPattern(
    std::vector<Binding<T>> &c, const VariableIndexMap &x) {
    // Determine size of the the vector for the bindings
    int sz = 0;
    for (const auto &ci : c) {
        sz += ci.get()->n_out;
    }

    // Create sparse matrix for the gradient
    CompressedColumnStorageFormat jac(sz, x.size());

    bopt_int c_idx = 0;

    for (auto &ci : c) {
        // For each non-zero element of the hessian, get their variable
        // coordinates and convert to their vector locations based on x
        int idx_x, idx_y;
        evaluator_out_info<Binding<T>> info;
        ci.get()->jac_info(info);

        // Determine locations for each non-zero entry
        for (int col = 0; col < info.out_m; ++col) {
            int start = indptr[col];
            int end = indptr[col + 1];

            for (int row = start; row < end; ++row) {
                // Get index of entry
                int idx = ind[row];
                // Add entry to full Jacobian
                jac.insert(x.index(idx), c_idx + col);
            }
        }

        idx += m;
    }

    return jac;
}

template <typename T>
bopt::CompressedColumnStorageFormat getHessianSparsityPattern(
    std::vector<Binding<T>> &c, const VariableIndexMap &x) {
    // Create sparse matrix for the gradient
    CompressedColumnStorageFormat hes(x.size(), x.size());

    for (auto &ci : c) {
        // For each non-zero element of the hessian, get their variable
        // coordinates and convert to their vector locations based on x
        int idx_x, idx_y;
        bopt_int n, m, nnz;
        bopt_int *ind, *indptr;
        ci.get()->hes_info(&n, &m, &nnz, ind, indptr);

        // Determine locations for each non-zero entry
        for (int col = 0; col < m; ++col) {
            int start = indptr[col];
            int end = indptr[col + 1];

            for (int row = start; row < end; ++row) {
                // Get index of entry
                int idx_r = ind[row];
                int idx_c = ind[col];
                // Add entry to full Jacobian
                hes.insert(x.index(idx_r), x.index(idx_c));
            }
        }
    }

    return hes;
}

/**
 * @brief Vector of constraint bindings of the form \f$ \bm c(x, p) =
 * [c_0(x, p), c_1(x, p), \hdots , c_n(x, p)]
 *
 */
class ConstraintBindingSet {
   public:
    Index numberOfConstraints() const {
        Index sz = 0;
        for (const auto &c : generic_) {
            bopt_int n;
            c.get()->con_info(&n, nullptr, nullptr, nullptr, nullptr);
            sz += n;
        }
        for (const auto &c : linear_) {
            bopt_int n;
            c.get()->con_info(&n, nullptr, nullptr, nullptr, nullptr);
            sz += n;
        }
        return sz;
    }

    /**
     * @brief Appends a generic constraint to the set
     *
     * @param con
     * @param x
     * @param p
     * @return Binding<Constraint>
     */
    void appendConstraint(const std::shared_ptr<Constraint> &con,
                          const std::vector<std::vector<Variable>> &in) {
        // Create a binding for the constraint
        generic_.push_back(Binding<Constraint>(con, in));
    }

    const std::vector<Binding<Constraint>> &getConstraints() const {
        return generic_;
    }

    /**
     * @brief Appends a generic constraint to the set
     *
     * @param con
     * @param x
     * @param p
     */
    void appendLinearConstraint(const std::shared_ptr<LinearConstraint> &con,
                                const std::vector<std::vector<Variable>> &in) {
        // Create a binding for the constraint
        linear_.push_back(Binding<LinearConstraint>(con, in));
    }

    const std::vector<Binding<LinearConstraint>> &getLinearConstraints() const {
        return linear_;
    }

    std::vector<Binding<Constraint>> getAllConstraints() const {
        std::vector<Binding<Constraint>> vec;
        vec.insert(vec.begin(), generic_.begin(), generic_.end());
        vec.insert(vec.end(), linear_.begin(), linear_.end());
        // Return vector of all constraints
        return vec;
    }

   private:
    // Constraint bindings
    std::vector<Binding<Constraint>> generic_;
    std::vector<Binding<LinearConstraint>> linear_;
};

/**
 * @brief Represents a generic mathematical program of the form \f$ \min f(x, p)
 * s.t. g_l \le g(x) \le q_u, x_l \le x \le x_u \f$
 *
 */
class MathematicalProgram : public ConstraintBindingSet, public CostBindingSet {
   public:
    friend class solvers::SolverBase;

    using Index = std::size_t;
    using String = std::string;

    MathematicalProgram() = default;
    MathematicalProgram(const String &name) : name_(name) {}

    /**
     * @brief Name of the program
     *
     * @return const String&
     */
    const String &name() const { return name_; }

    Index numberOfDecisionVariables() const { return x_idx_map_.size(); }

    Index numberOfParameters() const { return p_idx_map_.size(); }

    /**
     * @brief Decision variables initial values
     *
     * @return const std::vector<double>&
     */
    const std::vector<double> &x0() const { return x0_; }

    /**
     * @brief Decision variables upper bound vector
     *
     * @return const std::vector<double>&
     */
    const std::vector<double> &xbu() const { return xbu_; }

    /**
     * @brief Decision variables lower bound vector
     *
     * @return const std::vector<double>&
     */
    const std::vector<double> &xbl() const { return xbl_; }

    /**
     * @brief Parameter vector
     *
     * @return const std::vector<double>&
     */
    const std::vector<double> &p() const { return p_; }

    void addDecisionVariable(
        const Variable &v, const double &v0 = 0.0,
        const double &bl = -std::numeric_limits<double>::infinity(),
        const double &bu = std::numeric_limits<double>::infinity()) {
        x_idx_map_.add(v);

        // Update decision variable vector sizes
        x0_.emplace_back(v0);
        xbl_.emplace_back(bl);
        xbu_.emplace_back(bu);
    }

    void addParameter(const Variable &p, const double &val = 0.0) {
        p_idx_map_.add(p);
        // Update parameter vector sizes
        p_.emplace_back(val);
    }

    const Index &getDecisionVariableIndex(const Variable &v) const {
        return x_idx_map_.index(v);
    }

    std::vector<Index> getDecisionVariableIndices(
        const variable_vector_t &x) const {
        std::vector<Index> indices = {};
        for (const auto &xi : x) {
            indices.emplace_back(getDecisionVariableIndex(xi));
        }
        return indices;
    }

    const Index &getParameterIndex(const Variable &p) const {
        return p_idx_map_.index(p);
    }

    std::vector<Index> getParameterIndices(const variable_vector_t &p) const {
        std::vector<Index> indices = {};
        for (const auto &pi : p) {
            indices.emplace_back(getParameterIndex(pi));
        }
        return indices;
    }

    void setParameter(const Variable &p, const Scalar &val) {
        p_[p_idx_map_.index(p)] = val;
    }

    void setDecisionVariableInitialValue(const Variable &v, const Scalar &val) {
        x0_[x_idx_map_.index(v)] = val;
    }

    void setDecisionVariableBounds(const Variable &v, const Scalar &lb,
                                   const Scalar &ub) {
        const Index &idx = x_idx_map_.index(v);
        xbl_[idx] = lb;
        xbu_[idx] = ub;
    }

    /**
     * @brief The current index mapping used for the mathematical program for
     * all decision variables
     *
     * @return const VariableIndexMap&
     */
    const VariableIndexMap &getDecisionVariableIndexMap() const {
        return x_idx_map_;
    }

    /**
     * @brief The current index mapping used for the parameters within the
     * mathematical program
     *
     * @return const VariableIndexMap&
     */
    const VariableIndexMap &getParameterIndexMap() const { return p_idx_map_; }

   protected:
   private:
    // Probably need a specialised solver or at least a virtual method here
    // where it can be overwritten or something?

    // Name
    String name_;

    // Decision variables initial value
    std::vector<double> x0_;
    // Decision variables lower bound
    std::vector<double> xbl_;
    // Decision variables upper bound
    std::vector<double> xbu_;

    // Decision variable index map
    VariableIndexMap x_idx_map_;

    // Parameter values
    std::vector<double> p_;

    // Parameter index map
    VariableIndexMap p_idx_map_;
};

}  // namespace bopt

#endif /* OPTIMISATION_PROGRAM_H */
