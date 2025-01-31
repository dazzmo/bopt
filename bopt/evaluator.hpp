#pragma once

#include <Eigen/Core>
#include <optional>
#include <unsupported/Eigen/AutoDiff>

#include "bopt/assert.hpp"
#include "bopt/common.hpp"
#include "bopt/evaluator/base.hpp"
#include "bopt/evaluator/differentiable.hpp"
#include "bopt/evaluator/linear.hpp"
#include "bopt/evaluator/quadratic.hpp"

namespace bopt {

using Index = Eigen::Index;

/**
 * @brief Evaluator class related the evaluation of a function \f( y = f_p(x)
 * \f)
 *
 */
class EvaluatorBase {
   public:
    using SparsityPattern = std::vector<std::pair<int, int>>;

    /**
     * @brief Evaluates the expression using the input vector and the output
     * vector.
     *
     * @param x The input vector, composed as \f( x = [v p] \f) of size (num_var
     * + num_par x 1)
     * @param out
     */
    void eval(const Eigen::Ref<const VectorXd> &x, Eigen::Ref<VectorXd> out) {
        BOPT_ASSERT(x.rows() == dim_input());
        BOPT_ASSERT(out.rows() == dim_output());
        checkVector(x);
        evalImpl(x, out);
        checkVector(out);
    }

    /**
     * @brief Computes the jacobian of the expression with respect to the vector
     * x. That is J(x) = [∂f/∂x]. If jacobian_x_sparsity_pattern().has_value()
     * == true, `jacobian` returns a vector of the nonzero elements of the
     * matrix.
     *
     * @param x
     * @param jacobian Either a dense matrix of size (n_output() x n_tangent())
     * or a vector of size jacobian_x_sparsity_pattern()->size()
     */
    void jacobian(const Eigen::Ref<const VectorXd> &x,
                  Eigen::Ref<MatrixXd> jacobian) {
        BOPT_ASSERT(x.rows() == dim_input());
        jacobianImpl(x, jacobian);
    }

    /**
     * @brief Computes the jacobian of the expression with respect to the vector
     * [x p]. That is J = [∂f/∂x ∂f/∂p]. If jacobian_x_sparsity_pattern() and
     * jacobian_p_sparsity_pattern() are not empty, `jacobian` returns a vector
     * of the nonzero elements of the matrix.
     *
     * @param x
     * @param p
     * @param jacobian
     */
    void jacobian(const Eigen::Ref<const VectorXd> &x,
                  const Eigen::Ref<const VectorXd> &p,
                  Eigen::Ref<MatrixXd> jacobian) {
        jacobianImpl(x, p, jacobian);
    }

    void hessian(const Eigen::Ref<const VectorXd> &x,
                 const Eigen::Ref<const VectorXd> &lambda,
                 Eigen::Ref<MatrixXd> out) {
        hessianImpl(x, lambda, out);
    }

    void hessian(const Eigen::Ref<const VectorXd> &x,
                 const Eigen::Ref<const VectorXd> &p,
                 const Eigen::Ref<const VectorXd> &lambda,
                 Eigen::Ref<MatrixXd> out) {
        hessianImpl(x, p, lambda, out);
    }

    /**
     * @brief Provides a sparsity pattern for the evaluator Jacobian. If a
     * nullopt is provided, no sparse implementation is available to compute,
     * use dense.
     *
     * @return const std::optional<SparsityPattern>
     */
    const std::optional<SparsityPattern> jacobian_x_sparsity_pattern() const {
        return jacobian_x_sparsity_pattern_;
    }

    const std::optional<SparsityPattern> jacobian_p_sparsity_pattern() const {
        return jacobian_p_sparsity_pattern_;
    }

    const std::optional<SparsityPattern> hessian_xx_sparsity_pattern() const {
        return hessian_xx_sparsity_pattern_;
    }

    const std::optional<SparsityPattern> hessian_xp_sparsity_pattern() const {
        return hessian_xp_sparsity_pattern_;
    }

    const std::optional<SparsityPattern> hessian_pp_sparsity_pattern() const {
        return hessian_pp_sparsity_pattern_;
    }

    /**
     * @brief Set the sparsity patterns for the Jacobian of the evaluator with
     * respect to the variables, and optionally with respect to the parameters.
     *
     * @param pattern_x Sparsity pattern of the jacobian matrix w.r.t. x
     * @param pattern_p Sparsity pattern of the jacobian matrix w.r.t. p
     */
    void setJacobianSparsityPattern(
        const SparsityPattern &pattern_x,
        std::optional<const SparsityPattern> pattern_p = std::nullopt) {
        jacobian_x_sparsity_pattern_ = pattern_x;
        if (pattern_p.has_value()) {
            jacobian_p_sparsity_pattern_ = pattern_p;
        }
    }

    void setHessianSparsityPattern(
        const SparsityPattern &pattern_xx,
        std::optional<const SparsityPattern> pattern_xp = std::nullopt,
        std::optional<const SparsityPattern> pattern_pp = std::nullopt) {
        hessian_xx_sparsity_pattern_ = pattern_xx;
        if (pattern_xp.has_value()) {
            hessian_xp_sparsity_pattern_ = pattern_xp;
        }
        if (pattern_pp.has_value()) {
            hessian_xp_sparsity_pattern_ = pattern_pp;
        }
    }

    /**
     * @brief Dimension of the input variable vector, commonly denoted as x.
     *
     * @return const Index&
     */
    const Index &dim_input() const { return dim_input_; }

    /**
     * @brief Dimension of the tangent space for the input vector, typically
     * this is equal to dim_input().
     *
     * @return const Index&
     */
    const Index &dim_tangent_space() const { return dim_tangent_space_; }

    const Index &dim_output() const { return dim_output_; }

    const Index &dim_parameter() const { return dim_parameter_; }

    const std::string &description() const { return description_; }

    void setDescription(const std::string &description) {
        description_ = description;
    }

    const VectorXd &parameters() const { return parameters_; }

    void setParameters(const Eigen::Ref<const VectorXd> &p) {
        BOPT_ASSERT(p.size() == dim_parameter());
        parameters_ = p;
    }

   protected:
    EvaluatorBase(const Index &n_inputs, const Index &n_outputs,
                  const std::string &description = "")
        : dim_input_(n_inputs),
          dim_tangent_space_(n_inputs),
          dim_output_(n_outputs),
          jacobian_x_sparsity_pattern_(std::nullopt),
          jacobian_p_sparsity_pattern_(std::nullopt),
          hessian_xx_sparsity_pattern_(std::nullopt),
          hessian_xp_sparsity_pattern_(std::nullopt),
          hessian_pp_sparsity_pattern_(std::nullopt),
          parameters_(VectorXd::Zero(0)),
          description_(description) {}

    /**
     * @brief Sets the dimension of the evaluator output.
     *
     * @param dim Dimension of the vector
     */
    void setOutputDimension(const Index &dim) { dim_output_ = dim; }

    /**
     * @brief Sets the dimension of the evaluator parameter vector.
     *
     * @param dim Dimension of the vector
     */
    void setParameterDimension(const Index &dim) { dim_parameter_ = dim; }

    /**
     * @brief Sets the dimension of the tangent space for the input vector.
     *
     * @param dim Dimension of the vector
     */
    void setTangentSpaceDimension(const Index &dim) {
        dim_tangent_space_ = dim;
    }

    /**
     * @brief Implementation of the evaluator
     *
     * @param x
     * @param out
     */
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
                              Eigen::Ref<MatrixXd> out) {}

    virtual void jacobianImpl(const Eigen::Ref<const VectorXd> &x,
                              const Eigen::Ref<const VectorXd> &p,
                              Eigen::Ref<MatrixXd> out) {}
    /**
     * @brief Implements sparse evaluation of the hessian of the expression $\f
     * \lambda^T y \f$. If hessian_sparsity_pattern() is nullopt, expected
     * output should be a dense lower-triangular matrix. If
     * hessian_sparsity_pattern().has_value() is true, returns a vector of the
     * non-zero components of the matrix, with locations specified by the
     * sparsity pattern given by hessian_sparsity_pattern().
     *
     * @param x
     * @param lambda
     * @param out
     *
     * @note Only compute the lower-triangular component of the hessian.
     */
    virtual void hessianImpl(const Eigen::Ref<const VectorXd> &x,
                             const Eigen::Ref<const VectorXd> &lambda,
                             Eigen::Ref<MatrixXd> hessian) {}

    virtual void hessianImpl(const Eigen::Ref<const VectorXd> &x,
                             const Eigen::Ref<const VectorXd> &p,
                             const Eigen::Ref<const VectorXd> &lambda,
                             Eigen::Ref<MatrixXd> hessian) {}

   private:
    Index dim_input_;
    Index dim_tangent_space_;
    Index dim_parameter_;
    Index dim_output_;

    // Sparsity pattern for the evaluator jacobian with respect to x
    std::optional<SparsityPattern> jacobian_x_sparsity_pattern_;
    // Sparsity pattern for the evaluator jacobian with respect to p
    std::optional<SparsityPattern> jacobian_p_sparsity_pattern_;

    // Sparsity pattern for the evaluator lower-triangular hessian with respect
    // to x and x
    std::optional<SparsityPattern> hessian_xx_sparsity_pattern_;
    // Sparsity pattern for the evaluator lower-triangular hessian with respect
    // to x and p
    std::optional<SparsityPattern> hessian_xp_sparsity_pattern_;
    // Sparsity pattern for the evaluator lower-triangular hessian with respect
    // to p and p
    std::optional<SparsityPattern> hessian_pp_sparsity_pattern_;

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