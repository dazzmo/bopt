#pragma once

#include "bopt/expressions/expression.hpp"
#include "bopt/logging.hpp"

namespace bopt {

/**
 * @brief A twice-differentiable expression of the form \f$y = f_p(x)\f$
 *
 * @tparam ValueType
 */
template <typename ValueType>
class differentiable_scalar_expression_tpl
    : public scalar_expression_tpl<ValueType> {
   public:
    using typename evaluator_tpl<ValueType>::value_t;
    using typename evaluator_tpl<ValueType>::dense_vector_t;
    using typename evaluator_tpl<ValueType>::sparse_vector_t;
    using typename evaluator_tpl<ValueType>::dense_matrix_t;
    using typename evaluator_tpl<ValueType>::sparse_matrix_t;

    using vector_buffer_t =
        dense_sparse_buffer_tpl<dense_vector_t, sparse_vector_t>;
    using matrix_buffer_t =
        dense_sparse_buffer_tpl<dense_matrix_t, sparse_matrix_t>;

    differentiable_scalar_expression_tpl()
        : scalar_expression_tpl<ValueType>(0, 0) {}

    differentiable_scalar_expression_tpl(const bopt_index &sz_in,
                                         const bopt_index &sz_p = 0)
        : scalar_expression_tpl<ValueType>(sz_in, sz_p) {
        buffer_gradient_.dense = dense_vector_t::Zero(cols_gradient());
        buffer_hessian_.dense =
            dense_matrix_t::Zero(rows_hessian(), cols_hessian());
    }

    ~differentiable_scalar_expression_tpl() = default;

    /**
     * @brief Evaluates the dense jacobian for the expression \f$c(x)\f$
     (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_gradient(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_vector_t> out) {
        DBGASSERT(this->check_vector(x) && "gradient input is invalid");
        return eval_gradient_impl(x, out);
    }

    /**
     * @brief Evaluates the sparse gradient for the expression \f$c(x)\f$
     (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_gradient(
        const Eigen::Ref<const dense_vector_t> &x, sparse_vector_t &out) {
        DBGASSERT(this->check_vector(x) && "gradient input is invalid");
        return eval_gradient_impl(x, out);
    }

    /**
     * @brief The number of columns within the gradient vector
     *
     * @return bopt_index
     */
    virtual bopt_index cols_gradient() const { return this->sz_in(); }

    /**
     * @brief Populates a sparse matrix with the sparsity pattern of the
     * gradient
     *
     * @param gradient
     */
    virtual void sparsity_gradient(sparse_matrix_t &gradient) const {}

    /**
     * @brief Evaluates the lower trianguar dense hessian for the expression
     * \f$c(x)\f$ (i.e. \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_hessian(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_matrix_t> out) {
        DBGASSERT(this->check_vector(x) && "hessian input is invalid");
        return eval_hessian_impl(x, out);
    }

    /**
     * @brief Evaluates the lower triangular sparse hessian for the expression
     * \f$c(x)\f$ (i.e. \f$ \frac{\partial^2 c}{\partial x^2}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_hessian(
        const Eigen::Ref<const dense_vector_t> &x, sparse_matrix_t &out) {
        DBGASSERT(this->check_vector(x) && "hessian input is invalid");
        return eval_hessian_impl(x, out);
    }

    /**
     * @brief The number of rows within the expression hessian
     *
     * @return bopt_index
     */
    virtual bopt_index rows_hessian() const { return this->sz_in(); }

    /**
     * @brief The number of columns within the expression hessian
     *
     * @return bopt_index
     */
    virtual bopt_index cols_hessian() const { return this->sz_in(); }

    /**
     * @brief Populates a sparse matrix with the sparsity pattern of the
     * hessian
     *
     * @param hessian
     */
    virtual void sparsity_hessian(sparse_matrix_t &hessian) const {}

    vector_buffer_t &buffer_gradient() { return buffer_gradient_; }
    matrix_buffer_t &buffer_hessian() { return buffer_hessian_; }

   protected:
    virtual evaluator::return_status eval_gradient_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_vector_t> out) {
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_gradient_impl(
        const Eigen::Ref<const dense_vector_t> &x, sparse_vector_t &out) {
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_hessian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_matrix_t> out) {
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_hessian_impl(
        const Eigen::Ref<const dense_vector_t> &x, sparse_matrix_t &out) {
        return evaluator::return_status::NotImplemented;
    }

   private:
    vector_buffer_t buffer_gradient_;
    matrix_buffer_t buffer_hessian_;
};

/**
 * @brief A twice-differentiable expression of the form \f$y = f_p(x)\f$
 *
 * @tparam ValueType
 */
template <typename ValueType>
class differentiable_vector_expression_tpl
    : public vector_expression_tpl<ValueType> {
   public:
    using typename evaluator_tpl<ValueType>::value_t;
    using typename evaluator_tpl<ValueType>::dense_vector_t;
    using typename evaluator_tpl<ValueType>::sparse_vector_t;
    using typename evaluator_tpl<ValueType>::dense_matrix_t;
    using typename evaluator_tpl<ValueType>::sparse_matrix_t;

    using vector_buffer_t =
        dense_sparse_buffer_tpl<dense_vector_t, sparse_vector_t>;
    using matrix_buffer_t =
        dense_sparse_buffer_tpl<dense_matrix_t, sparse_matrix_t>;

    differentiable_vector_expression_tpl() = default;
    ~differentiable_vector_expression_tpl() = default;

    differentiable_vector_expression_tpl(const bopt_index &sz_in,
                                         const bopt_index &sz_out,
                                         const bopt_index &sz_p = 0)
        : vector_expression_tpl<ValueType>(sz_in, sz_out, sz_p) {
        buffer_jacobian_.dense =
            dense_matrix_t::Zero(rows_jacobian(), cols_jacobian());
        buffer_hessian_.dense =
            dense_matrix_t::Zero(rows_hessian(), cols_hessian());
    }

    /**
     * @brief Evaluates the dense jacobian for the expression \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_jacobian(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_matrix_t> out) {
        DBGASSERT(this->check_vector(x) && "Jacobian input is invalid");
        return eval_jacobian_impl(x, out);
    }

    /**
     * @brief Evaluates the sparse jacobian for the expression \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_jacobian(
        const Eigen::Ref<const dense_vector_t> &x, sparse_matrix_t &out) {
        DBGASSERT(this->check_vector(x) && "Jacobian input is invalid");
        return eval_jacobian_impl(x, out);
    }

    /**
     * @brief The number of rows within the expression jacobian
     *
     * @return bopt_index
     */
    virtual bopt_index rows_jacobian() const { return this->sz_out(); }

    /**
     * @brief The number of columns within the expression jacobian
     *
     * @return bopt_index
     */
    virtual bopt_index cols_jacobian() const { return this->sz_in(); }

    /**
     * @brief Populates a sparse matrix with the sparsity pattern of the
     * jacobian
     *
     * @param jacobian
     */
    virtual void sparsity_jacobian(sparse_matrix_t &jacobian) const {}

    /**
     * @brief Evaluates the lower triangular component of the dense hessian for
     * the expression \f$c(x)\f$ (i.e. \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_hessian(
        const Eigen::Ref<const dense_vector_t> &x,
        const Eigen::Ref<const dense_vector_t> &lambda,
        Eigen::Ref<dense_matrix_t> out) {
        DBGASSERT(this->check_vector(x) && "hessian input is invalid");
        return eval_hessian_impl(x, lambda, out);
    }

    /**
     * @brief Evaluates the sparse lower triangular component of the hessian for
     * the expression \f$c(x)\f$ (i.e.  \f$ \frac{\partial^2 c}{\partial
     * x^2}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_hessian(
        const Eigen::Ref<const dense_vector_t> &x,
        const Eigen::Ref<const dense_vector_t> &lambda, sparse_matrix_t &out) {
        DBGASSERT(this->check_vector(x) && "hessian input is invalid");
        return eval_hessian_impl(x, lambda, out);
    }

    /**
     * @brief The number of rows within the expression hessian
     *
     * @return bopt_index
     */
    virtual bopt_index rows_hessian() const { return this->sz_in(); }

    /**
     * @brief The number of columns within the expression hessian
     *
     * @return bopt_index
     */
    virtual bopt_index cols_hessian() const { return this->sz_in(); }

    /**
     * @brief Populates a sparse matrix with the sparsity pattern of the
     * hessian
     *
     * @param hessian
     */
    virtual void sparsity_hessian(sparse_matrix_t &hessian) const {}

    // Buffers for evaluation
    matrix_buffer_t &buffer_jacobian() { return buffer_jacobian_; }
    matrix_buffer_t &buffer_hessian() { return buffer_hessian_; }

   protected:
    virtual evaluator::return_status eval_jacobian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_matrix_t> out) {
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_jacobian_impl(
        const Eigen::Ref<const dense_vector_t> &x, sparse_matrix_t &out) {
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_hessian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        const Eigen::Ref<const dense_vector_t> &lambda,
        Eigen::Ref<dense_matrix_t> out) {
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_hessian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        const Eigen::Ref<const dense_vector_t> &lambda, sparse_matrix_t &out) {
        return evaluator::return_status::NotImplemented;
    }

   private:
    matrix_buffer_t buffer_jacobian_;
    matrix_buffer_t buffer_hessian_;
};

}  // namespace bopt
