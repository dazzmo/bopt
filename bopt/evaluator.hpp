#pragma once

#include <Eigen/Core>
#include <optional>
#include <unsupported/Eigen/AutoDiff>

#include "bopt/evaluator/base.hpp"
#include "bopt/evaluator/differentiable.hpp"
#include "bopt/evaluator/linear.hpp"
#include "bopt/evaluator/quadratic.hpp"

namespace bopt {

using Index = std::size_t;

class EvaluatorBase {
   public:
    /**
     * @brief Evaluates the expression using the input vector and the output
     * vector.
     *
     * @param x The input vector, composed as x = [v p] of size (num_var +
     * num_par x 1)
     * @param out
     */
    void eval(const Eigen::Ref<const VectorXd> &x, Eigen::Ref<VectorXd> out) {
        // BOPT_ASSERT(x.rows() == sz_in() && out.rows() == sz_out());
        evalImpl(x, out);
    }

    bool hasJacobian() const { return has_jacobian_; }

    void jacobian(const Eigen::Ref<const VectorXd> &x,
                  Eigen::Ref<MatrixXd> out) {
        jacobianImpl(x, out);
    }

    void jacobian(const Eigen::Ref<const VectorXd> &x,
                  SparseMatrix<double> &out) {
        jacobianImpl(x, out);
    }

    /**
     * @brief Provides a sparsity pattern for the evaluator Jacobian. If a
     * nullopt is provided, no sparse implementation is available to compute,
     * use dense.
     *
     * @return const std::optional<std::vector<std::pair<int, int>>>
     */
    const std::optional<std::vector<std::pair<int, int>>>
    jacobian_sparsity_pattern() const {
        return jacobian_sparsity_pattern_;
    }

    void setJacobianSparsityPattern(
        const std::vector<std::pair<int, int>> &pattern) {
        jacobian_sparsity_pattern_ = pattern;
    }

    bool hasHessian() const { return has_hessian_; }

    void hessian(const Eigen::Ref<const VectorXd> &x,
                 const Eigen::Ref<const VectorXd> &lamba,
                 Eigen::Ref<MatrixXd> out) {}

    /**
     * @brief Evaluation of the hessian of the evaluator output, as a sparse
     * matrix.
     *
     * @param x
     * @param lamba
     * @param out
     */
    void hessian(const Eigen::Ref<const VectorXd> &x,
                 const Eigen::Ref<const VectorXd> &lamba,
                 SparseMatrix<double> &out) {}

    const std::optional<std::vector<std::pair<int, int>>>
    hessian_sparsity_pattern() const {
        return hessian_sparsity_pattern_;
    }

    void setHessianSparsityPattern(
        const std::vector<std::pair<int, int>> &pattern) {
        hessian_sparsity_pattern_ = pattern;
    }

    const Index &n_inputs() const { return n_var_; }
    const Index &n_parameters() const { return n_par_; }
    const Index &n_outputs() const { return n_out_; }

    const Index &rows_jacobian() const { return rows_jacobian_; }
    const Index &cols_jacobian() const { return cols_jacobian_; }

    const Index &rows_hessian() const { return rows_hessian_; }
    const Index &cols_hessian() const { return cols_hessian_; }

    const std::string &description() const { return description_; }

    void setDescription(const std::string &description) {
        description_ = description;
    }

    const VectorXd &parameters() const { return parameters_; }

    void setParameters(const Eigen::Ref<const VectorXd> &p) { parameters_ = p; }

   protected:
    EvaluatorBase(const Index &n_inputs, const Index &n_outputs,
                  const std::string &description = "")
        : n_var_(n_inputs),
          n_out_(n_outputs),
          has_jacobian_(false),
          rows_jacobian_(0),
          cols_jacobian_(0),
          jacobian_sparsity_pattern_(std::nullopt),
          has_hessian_(false),
          rows_hessian_(0),
          cols_hessian_(0),
          hessian_sparsity_pattern_(std::nullopt),
          parameters_(VectorXd::Zero(0)),
          description_(description) {}

    void setNumberOutputs(const Index &n) { n_out_ = n; }

    void setJacobianSize(const Index &rows, const Index &cols) {
        rows_jacobian_ = rows;
        cols_jacobian_ = cols;
    }

    void setHessianSize(const Index &rows, const Index &cols) {
        rows_hessian_ = rows;
        cols_hessian_ = cols;
    }

    /**
     * @brief Set the flag to indicate whether computation of a Jacobian is
     * available.
     *
     * @param flag If true, indicates an implementation of jacobianImpl() is
     * available.
     */
    void setJacobianFlag(bool flag) { has_jacobian_ = flag; }

    /**
     * @brief Set the flag to indicate whether computation of a Hessian is
     * available.
     *
     * @param flag If true, indicates an implementation of hessianImpl() is
     * available.
     */
    void setHessianFlag(bool flag) { has_hessian_ = flag; }

    virtual void evalImpl(const Eigen::Ref<const VectorXd> &x,
                          Eigen::Ref<VectorXd> out) = 0;

    /**
     * @brief Implements dense evaluation of the expression Jacobian for the
     * given variables.
     *
     * @param x
     * @param out
     */
    virtual void jacobianImpl(const Eigen::Ref<const VectorXd> &x,
                              Eigen::Ref<VectorXd> out) {}

    /**
     * @brief Implements sparse evaluation of the expression Jacobian for the
     * given variables.
     *
     * @param x
     * @param out
     */
    virtual void jacobianImpl(const Eigen::Ref<const VectorXd> &x,
                              SparseMatrix<double> &out) {}

    /**
     * @brief Implements dense evaluation of the hessian of the expression $\f
     * \lambda^T y \f$
     *
     * @param x
     * @param lambda
     * @param out
     *
     * @note Only compute the lower-triangular component of the hessian.
     */
    virtual void hessianImpl(const Eigen::Ref<const VectorXd> &x,
                             const Eigen::Ref<const VectorXd> &lambda,
                             Eigen::Ref<MatrixXd> out) {}

    /**
     * @brief Implements sparse evaluation of the hessian of the expression $\f
     * \lambda^T y \f$
     *
     * @param x
     * @param lambda
     * @param out
     *
     * @note Only compute the lower-triangular component of the hessian.
     */
    virtual void hessianImpl(const Eigen::Ref<const VectorXd> &x,
                             const Eigen::Ref<const VectorXd> &lambda,
                             SparseMatrix<double> &out) {}

   private:
    Index n_var_;
    Index n_par_;
    Index n_out_;

    bool has_jacobian_;

    Index rows_jacobian_;
    Index cols_jacobian_;

    std::optional<std::vector<std::pair<int, int>>> jacobian_sparsity_pattern_;

    bool has_hessian_;

    Index rows_hessian_;
    Index cols_hessian_;

    std::optional<std::vector<std::pair<int, int>>> hessian_sparsity_pattern_;

    VectorXd parameters_;

    std::string description_;
};

std::ostream &operator<<(std::ostream &os, const EvaluatorBase &e);

class TestAutodiffModule {
   public:
    typedef Eigen::AutoDiffScalar<VectorXd> AD;
    typedef Eigen::VectorX<AD> VectorAD;
    typedef Eigen::AutoDiffScalar<VectorAD> ADD;
    // Vector with the ability to compute the hessian as well?
    typedef Eigen::VectorX<ADD> VectorADD;

    virtual void eval(const Eigen::Ref<const VectorADD> &x,
                      Eigen::Ref<VectorADD> y) = 0;
};

}  // namespace bopt