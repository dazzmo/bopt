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
     * @brief Evaluates the expression y = fₚ(x) using variables x and
     * parameters p (set through \ref EvaluatorBase::setParameters()).
     *
     * @param x The input vector, composed as \f( x = [v p] \f) of size (num_var
     * + num_par x 1)
     * @param out
     */
    void eval(const Eigen::Ref<const VectorXd> &x, Eigen::Ref<VectorXd> y) {
        BOPT_ASSERT(x.rows() == dim_input());
        BOPT_ASSERT(y.rows() == dim_output());
        checkVector(x);
        evalImpl(x, y);
        checkVector(y);
    }

    /**
     * @brief Computes the jacobian of the epxression with respect to the vector
     * x. That is J(x) = [∂f/∂x]. If jacobian_x_sparsity_pattern().has_value()
     * == true, `jacobian` returns a vector of the nonzero elements of the
     * matrix.
     *
     * @param x
     * @param jacobian Either a dense matrix of size (n_output() x n_tangent())
     * or a vector of size jacobian_x_sparsity_pattern()->size()
     */
    void evalJacobian(const Eigen::Ref<const VectorXd> &x,
                      Eigen::Ref<MatrixXd> jacobian) {
        BOPT_ASSERT(x.rows() == dim_input());
        evalJacobianImpl(x, jacobian);
    }

    /**
     * @brief Computes the jacobian of the epxression with respect to the vector
     * [x p]. That is J = [∂f/∂x ∂f/∂p]. If jacobian_x_sparsity_pattern() and
     * jacobian_p_sparsity_pattern() are not empty, `jacobian` returns a vector
     * of the nonzero elements of the matrix.
     *
     * @param x
     * @param p
     * @param jacobian
     */
    void evalJacobian(const Eigen::Ref<const VectorXd> &x,
                      const Eigen::Ref<const VectorXd> &p,
                      Eigen::Ref<MatrixXd> jacobian) {
        evalJacobianImpl(x, p, jacobian);
    }

    /**
     * @brief Computes the hessian of the epxression λᵀf with respect to the
     * variables x
     *
     *  i.e.  ∂²(λᵀf)/∂x²
     *
     * @note If \ref hessian_xx_nz_only() == true, returns a vector of the
     * the nonzero elements of the matrix.
     *
     * @param x
     * @param lambda Multipliers for the vector product
     * @param jacobian
     */
    void evalHessian(const Eigen::Ref<const VectorXd> &x,
                     const Eigen::Ref<const VectorXd> &lambda,
                     Eigen::Ref<MatrixXd> out) {
        evalHessianImpl(x, lambda, out);
    }

    /**
     * @brief Computes the hessian of the epxression λᵀf with respect to the
     * variables x and parameters p
     *
     * i.e.  [∂²(λᵀf)/∂x² ∂²(λᵀf)/∂x∂p;
     *        ∂²(λᵀf)/∂p∂x ∂²(λᵀf)/∂p²]
     *
     * If hessian_xx_sparsity_pattern().has_value() == true, returns a vector of
     * the nonzero elements of the matrix.
     *
     * @param x
     * @param p
     * @param jacobian
     */
    void evalHessian(const Eigen::Ref<const VectorXd> &x,
                     const Eigen::Ref<const VectorXd> &p,
                     const Eigen::Ref<const VectorXd> &lambda,
                     Eigen::Ref<MatrixXd> out) {
        evalHessianImpl(x, p, lambda, out);
    }

    /**
     * @brief Optional sparsity pattern for the evaluator Jacobian ∂f/∂x.
     *
     * @return const std::optional<SparsityPattern>
     */
    const std::optional<SparsityPattern> jacobian_x_sparsity_pattern() const {
        return jacobian_x_sparsity_pattern_;
    }
    /**
     * @brief Optional sparsity pattern for the evaluator Jacobian ∂f/∂p.
     *
     * @return const std::optional<SparsityPattern>
     */
    const std::optional<SparsityPattern> jacobian_p_sparsity_pattern() const {
        return jacobian_p_sparsity_pattern_;
    }

    /**
     * @brief Optional sparsity pattern for the lower-triangular Hessian
     * ∂²(λᵀf)/∂x².
     *
     * @return const std::optional<SparsityPattern>
     */
    const std::optional<SparsityPattern> hessian_xx_sparsity_pattern() const {
        return hessian_xx_sparsity_pattern_;
    }

    /**
     * @brief Optional sparsity pattern for the lower-triangular Hessian
     * ∂²(λᵀf)/∂p∂x.
     *
     * @return const std::optional<SparsityPattern>
     */
    const std::optional<SparsityPattern> hessian_px_sparsity_pattern() const {
        return hessian_px_sparsity_pattern_;
    }

    /**
     * @brief Optional sparsity pattern for the lower-triangular Hessian
     * ∂²(λᵀf)/∂p².
     *
     * @return const std::optional<SparsityPattern>
     */
    const std::optional<SparsityPattern> hessian_pp_sparsity_pattern() const {
        return hessian_pp_sparsity_pattern_;
    }

    /**
     * @brief Whether the evalution of the jacobian ∂f/∂x returns only the
     * non-zero components.
     *
     * @return true
     * @return false
     */
    bool jacobian_x_nz_only() const { return jacobian_x_nz_only_; }

    /**
     * @brief Whether the evalution of the jacobian ∂f/∂p returns only the
     * non-zero components.
     *
     * @return true
     * @return false
     */
    bool jacobian_p_nz_only() const { return jacobian_p_nz_only_; }

    /**
     * @brief Whether the evalution of the hessian ∂²(λᵀf)/∂x² returns only the
     * non-zero components.
     *
     * @return true
     * @return false
     */
    bool hessian_xx_nz_only() const { return hessian_xx_nz_only_; }

    /**
     * @brief Whether the evalution of the hessian ∂²(λᵀf)/∂p∂x returns only the
     * non-zero components.
     *
     * @return true
     * @return false
     */
    bool hessian_px_nz_only() const { return hessian_px_nz_only_; }

    /**
     * @brief Whether the evalution of the hessian ∂²(λᵀf)/∂p² returns only the
     * non-zero components.
     *
     * @return true
     * @return false
     */
    bool hessian_pp_nz_only() const { return hessian_pp_nz_only_; }

    /**
     * @brief Set the sparsity patterns for the Jacobian ∂f/∂x of the evaluator
     * with respect to the variables, and optionally with respect to the
     * parameters (∂f/∂p).
     *
     * @param pattern_x Sparsity pattern of ∂f/∂x
     * @param pattern_p Sparsity pattern of ∂f/∂p
     */
    void setJacobianSparsityPattern(
        const SparsityPattern &pattern_x,
        std::optional<const SparsityPattern> pattern_p = std::nullopt) {
        jacobian_x_sparsity_pattern_ = pattern_x;
        if (pattern_p.has_value()) {
            jacobian_p_sparsity_pattern_ = pattern_p;
        }
    }

    /**
     * @brief Set the sparsity patterns for ∂²(λᵀf)/∂x²,∀λ, with respect to the
     * variables x, and optionally with respect to the parameters p.
     *
     * @param pattern_xx Sparsity pattern of the lower triangular component of
     * ∂²(λᵀf)/∂x²
     * @param pattern_px Sparsity pattern of the lower triangular component of
     * ∂²(λᵀf)/∂x∂p
     * @param pattern_pp Sparsity pattern of the lower triangular component of
     * ∂²(λᵀf)/∂p²
     */
    void setHessianSparsityPattern(
        const SparsityPattern &pattern_xx,
        std::optional<const SparsityPattern> pattern_px = std::nullopt,
        std::optional<const SparsityPattern> pattern_pp = std::nullopt) {
        hessian_xx_sparsity_pattern_ = pattern_xx;
        if (pattern_px.has_value()) {
            hessian_px_sparsity_pattern_ = pattern_px;
        }
        if (pattern_pp.has_value()) {
            hessian_px_sparsity_pattern_ = pattern_pp;
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

    /**
     * @brief Set the parameter vector p (dim_parameter() x 1).
     *
     * @param p
     */
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
          jacobian_p_nz_only_(false),
          jacobian_x_nz_only_(false),
          hessian_xx_nz_only_(false),
          hessian_px_nz_only_(false),
          hessian_pp_nz_only_(false),
          jacobian_x_sparsity_pattern_(std::nullopt),
          jacobian_p_sparsity_pattern_(std::nullopt),
          hessian_xx_sparsity_pattern_(std::nullopt),
          hessian_px_sparsity_pattern_(std::nullopt),
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
     * @brief Sets the dimension of the tangent space for the input variables.
     *
     * @param dim Dimension of the vector
     */
    void setTangentSpaceDimension(const Index &dim) {
        dim_tangent_space_ = dim;
    }

    /**
     * @brief Sets the dimension of the evaluator parameter vector.
     *
     * @param dim Dimension of the vector
     */
    void setParameterDimension(const Index &dim) { dim_parameter_ = dim; }

    /**
     * @brief Implementation of the evaluator
     *
     * @param x
     * @param out
     */
    virtual void evalImpl(const Eigen::Ref<const VectorXd> &x,
                          Eigen::Ref<VectorXd> out) = 0;

    /**
     * \copydoc EvaluatorBase::jacobian(const Eigen::Ref<const VectorXd>,
     * Eigen::Ref<MatrixXd>)
     *
     */
    virtual void evalJacobianImpl(const Eigen::Ref<const VectorXd> &x,
                                  Eigen::Ref<MatrixXd> out) {}

    /**
     * \copydoc EvaluatorBase::jacobian(const Eigen::Ref<const VectorXd>, const
     * Eigen::Ref<const VectorXd>, Eigen::Ref<MatrixXd>)
     *
     */
    virtual void evalJacobianImpl(const Eigen::Ref<const VectorXd> &x,
                                  const Eigen::Ref<const VectorXd> &p,
                                  Eigen::Ref<MatrixXd> out) {}
    /**
     * \copydoc EvaluatorBase::evalHessian(const Eigen::Ref<const VectorXd>,
     * const Eigen::Ref<const VectorXd>, Eigen::Ref<MatrixXd>)
     *
     */
    virtual void evalHessianImpl(const Eigen::Ref<const VectorXd> &x,
                                 const Eigen::Ref<const VectorXd> &lambda,
                                 Eigen::Ref<MatrixXd> hessian) {}

    /**
     * \copydoc EvaluatorBase::evalHessian(const Eigen::Ref<const VectorXd>,
     * const Eigen::Ref<const VectorXd>, Eigen::Ref<const VectorXd>,
     * Eigen::Ref<MatrixXd>)
     *
     */
    virtual void evalHessianImpl(const Eigen::Ref<const VectorXd> &x,
                                 const Eigen::Ref<const VectorXd> &p,
                                 const Eigen::Ref<const VectorXd> &lambda,
                                 Eigen::Ref<MatrixXd> hessian) {}

    /**
     * @brief Indicate whether the evaluation of the jacobians will return only
     * the non-zero elements. If false, evaluation expects the full jacobian to
     * be computed.
     *
     * @param jacobian_x
     * @param jacobian_p
     */
    void setJacobianNonZeroOnly(bool jacobian_x, bool jacobian_p = false) {
        jacobian_x_nz_only_ = jacobian_x;
        jacobian_p_nz_only_ = jacobian_p;
    }

    /**
     * @brief Indicate whether the evaluation of the jacobians will return only
     * the non-zero elements. If false, evaluation expects the full jacobian to
     * be computed.
     *
     * @param hessian_xx
     * @param hessian_px
     * @param hessian_pp
     */
    void setHessianNonZeroOnly(bool hessian_xx, bool hessian_px = false,
                               bool hessian_pp = false) {
        hessian_xx_nz_only_ = hessian_xx;
        hessian_px_nz_only_ = hessian_px;
        hessian_pp_nz_only_ = hessian_pp;
    }

   private:
    Index dim_input_;
    Index dim_tangent_space_;
    Index dim_parameter_;
    Index dim_output_;

    // Flag to indicate whether the output from jacobian() is a dense matrix or
    // a vector of non-zero entries
    bool jacobian_x_nz_only_;
    bool jacobian_p_nz_only_;

    bool hessian_xx_nz_only_;
    bool hessian_px_nz_only_;
    bool hessian_pp_nz_only_;

    // Sparsity pattern for the evaluator jacobian with respect to x
    std::optional<SparsityPattern> jacobian_x_sparsity_pattern_;
    // Sparsity pattern for the evaluator jacobian with respect to p
    std::optional<SparsityPattern> jacobian_p_sparsity_pattern_;

    // Sparsity pattern for the evaluator lower-triangular hessian with respect
    // to x and x
    std::optional<SparsityPattern> hessian_xx_sparsity_pattern_;
    // Sparsity pattern for the evaluator lower-triangular hessian with respect
    // to x and p
    std::optional<SparsityPattern> hessian_px_sparsity_pattern_;
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