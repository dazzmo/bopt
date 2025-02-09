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

// Forward declarations
template <typename Scalar>
struct EvaluatorBaseDataTpl;

using Index = Eigen::Index;

/**
 * @brief Evaluator class related the evaluation of a function y = fₚ(x)
 *
 * @tparam Scalar
 */
template <typename Scalar>
class EvaluatorBaseTpl {
   public:
    using EvaluatorData = EvaluatorBaseDataTpl<Scalar>;

    /**
     * @brief Evaluates the expression y = fₚ(x) using variables x and
     * parameters p (set through \ref EvaluatorBaseTpl::setParameters()).
     *
     * @param x The input vector (dim_input() x 1)
     * @param data
     */
    void eval(const Eigen::Ref<const VectorXd> &x, EvaluatorData &data) const {
        BOPT_ASSERT(x.rows() == dim_input());
        BOPT_ASSERT(data.y.rows() == dim_output());
        checkVector(x);
        evalImpl(x, data);
        checkVector(data.y);
    }

    /**
     * @brief Computes the jacobians of the expression f.
     *
     * @param x
     * @param data
     * @param compute_x Compute ∂y/∂x
     * @param compute_p Compute ∂y/∂p
     */
    void evalJacobians(const Eigen::Ref<const VectorXd> &x, EvaluatorData &data,
                       bool compute_x = true, bool compute_p = false) const {
        BOPT_ASSERT(x.rows() == dim_input());
        evalJacobiansImpl(x, data, compute_x, compute_p);
    }

    /**
     * @brief Computes the sparse jacobians of the expression f.
     *
     * @param x
     * @param data
     * @param compute_x Compute ∂f/∂x
     * @param compute_p Compute ∂f/∂p
     */
    void evalSparseJacobians(const Eigen::Ref<const VectorXd> &x,
                             EvaluatorData &data, bool compute_x = true,
                             bool compute_p = false) const {
        BOPT_ASSERT(x.rows() == dim_input());
        evalSparseJacobiansImpl(x, data, compute_x, compute_p);
    }

    /**
     * @brief Computes the lower-triangular hessians of the vector-product of
     * the expression λᵀf.
     *
     * @param x
     * @param lambda
     * @param data
     * @param compute_xx Compute ∂²(λᵀf)/∂x²
     * @param compute_xp Compute ∂²(λᵀf)/∂x∂p
     * @param compute_pp Compute ∂²(λᵀf)/∂p²
     */
    void evalHessians(const Eigen::Ref<const VectorXd> &x,
                      const Eigen::Ref<const VectorXd> &lambda,
                      EvaluatorData &data, bool compute_xx = true,
                      bool compute_xp = false, bool compute_pp = false) const {
        evalHessiansImpl(x, lambda, data, compute_xx, compute_xp, compute_pp);
    }

    /**
     * @brief Computes the sparse lower-triangular hessians of the
     * vector-product of the expression λᵀf.
     *
     * @param x
     * @param lambda
     * @param data
     * @param compute_xx Compute ∂²(λᵀf)/∂x²
     * @param compute_xp Compute ∂²(λᵀf)/∂x∂p
     * @param compute_pp Compute ∂²(λᵀf)/∂p²
     */
    void evalSparseHessians(const Eigen::Ref<const VectorXd> &x,
                            const Eigen::Ref<const VectorXd> &lambda,
                            EvaluatorData &data, bool compute_xx = true,
                            bool compute_xp = false,
                            bool compute_pp = false) const {
        evalSparseHessiansImpl(x, lambda, data, compute_xx, compute_xp,
                               compute_pp);
    }

    /**
     * @brief Provides the sparsity patterns for the sparse
     * Jacobians ∂f/∂x and ∂f/∂p (if applicable).
     *
     */
    virtual void setJacobianSparsityPatterns(EvaluatorData &data) const {}

    /**
     * @brief Provides the sparsity patterns for the lower-triangular Hessians
     * ∂²(λᵀf)/∂x², ∂²(λᵀf)/∂x∂p,  ∂²(λᵀf)/∂p² ∀λ (if applicable)
     *
     */
    virtual void setHessianSparsityPatterns(EvaluatorData &data) const {}

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

    /**
     * @brief Dimension of the output vector y.
     *
     * @return const Index&
     */
    const Index &dim_output() const { return dim_output_; }

    /**
     * @brief Dimension of the parameter vector p.
     *
     * @return const Index&
     */
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
    EvaluatorBaseTpl(const Index &n_inputs, const Index &n_outputs,
                     const std::string &description = "")
        : dim_input_(n_inputs),
          dim_tangent_space_(n_inputs),
          dim_output_(n_outputs),
          dim_parameter_(0),
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
                          EvaluatorData &data) const = 0;

    /**
     * \copydoc EvaluatorBaseTpl::evalJacobians(const Eigen::Ref<const
     * VectorXd>, EvaluatorData &)
     *
     */
    virtual void evalJacobiansImpl(const Eigen::Ref<const VectorXd> &x,
                                   EvaluatorData &data, bool compute_x,
                                   bool compute_p) const {}

    /**
     * \copydoc EvaluatorBaseTpl::evalSparseJacobians(const Eigen::Ref<const
     * VectorXd>, EvaluatorData &)
     *
     */
    virtual void evalSparseJacobiansImpl(const Eigen::Ref<const VectorXd> &x,
                                         EvaluatorData &data, bool compute_x,
                                         bool compute_p) const {}
    /**
     * \copydoc EvaluatorBaseTpl::evalHessians(const Eigen::Ref<const VectorXd>,
     * const Eigen::Ref<const VectorXd>, EvaluatorData &)
     *
     */
    virtual void evalHessiansImpl(const Eigen::Ref<const VectorXd> &x,
                                  const Eigen::Ref<const VectorXd> &lambda,
                                  EvaluatorData &data, bool compute_xx,
                                  bool compute_xp, bool compute_pp) const {}

    /**
     * \copydoc EvaluatorBaseTpl::evalSparseHessians(const Eigen::Ref<const
     * VectorXd>, const Eigen::Ref<const VectorXd>, EvaluatorData &)
     *
     */
    virtual void evalSparseHessiansImpl(
        const Eigen::Ref<const VectorXd> &x,
        const Eigen::Ref<const VectorXd> &lambda, EvaluatorData &data,
        bool compute_xx, bool compute_xp, bool compute_pp) const {}

   private:
    /// @brief Dimension of the input vector
    Index dim_input_;
    Index dim_tangent_space_;
    Index dim_parameter_;
    Index dim_output_;

    VectorXd parameters_;
    std::string description_;
};

typedef EvaluatorBaseTpl<double> EvaluatorBase;

template <typename Scalar>
std::ostream &operator<<(std::ostream &os, const EvaluatorBaseTpl<Scalar> &e) {
    os << "EvaluatorBase\n";
    os << "description: " << e.description() << '\n';
    os << "input dim: " << e.dim_input() << '\n';
    os << "output dim: " << e.dim_output();
    return os;
}

template <typename Scalar>
struct EvaluatorBaseDataTpl {
    EvaluatorBaseDataTpl(const EvaluatorBaseTpl<Scalar> &e)
        : y(VectorX<Scalar>::Zero(e.dim_output())),
          Jx(MatrixX<Scalar>::Zero(e.dim_output(), e.dim_tangent_space())),
          Jp(MatrixX<Scalar>::Zero(e.dim_output(), e.dim_parameter())),
          Hxx(MatrixX<Scalar>::Zero(e.dim_tangent_space(),
                                    e.dim_tangent_space())),
          Hxp(MatrixX<Scalar>::Zero(e.dim_tangent_space(), e.dim_parameter())),
          Hpp(MatrixX<Scalar>::Zero(e.dim_parameter(), e.dim_parameter())),
          Jx_s(e.dim_output(), e.dim_tangent_space()),
          Hxx_s(e.dim_input(), e.dim_input()),
          Hxp_s(e.dim_input(), e.dim_input()),
          Hpp_s(e.dim_input(), e.dim_input()) {
        e.setJacobianSparsityPatterns(*this);
        e.setHessianSparsityPatterns(*this);
    }

    /// Evaluator output vector y
    VectorX<Scalar> y;

    /// Dense matrix for ∂y/∂x
    MatrixX<Scalar> Jx;
    /// Dense matrix for ∂y/∂p
    MatrixX<Scalar> Jp;

    /// Sparse matrix for ∂y/∂x
    SparseMatrix<Scalar> Jx_s;
    /// Sparse matrix for ∂y/∂p
    SparseMatrix<Scalar> Jp_s;

    /// Dense matrix for lower-triangular matrix ∂²(λᵀy)/∂x²
    MatrixX<Scalar> Hxx;
    /// Dense matrix for lower-triangular matrix ∂²(λᵀy)/∂x∂p
    MatrixX<Scalar> Hxp;
    /// Dense matrix for lower-triangular matrix ∂²(λᵀy)/∂p²
    MatrixX<Scalar> Hpp;

    /// Sparse matrix for lower-triangular matrix ∂²(λᵀy)/∂x²
    SparseMatrix<Scalar> Hxx_s;
    /// Sparse matrix for lower-triangular matrix ∂²(λᵀy)/∂x∂p
    SparseMatrix<Scalar> Hxp_s;
    /// Sparse matrix for lower-triangular matrix ∂²(λᵀy)/∂p²
    SparseMatrix<Scalar> Hpp_s;
};

}  // namespace bopt