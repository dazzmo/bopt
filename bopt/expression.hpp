#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <memory>

#include "bopt/bounds.hpp"
#include "bopt/evaluator.hpp"
#include "bopt/logging.hpp"

namespace bopt {

template <typename DenseType, typename SparseType>
struct dense_sparse_buffer_tpl {
    DenseType dense;
    SparseType sparse;
};

template <typename ValueType>
class expression_base_tpl {
   public:
    using dense_vector_t = Eigen::VectorX<ValueType>;

    /**
     * @brief Construct a new expression base tpl object
     *
     * @param np Number of parameters in the expression \f$ f_p(x) = y \f$
     */
    expression_base_tpl(const bopt_index &np) : np_(np) {
        p_ = dense_vector_t::Zero(np);
    }

    const dense_vector_t &parameters() const { return p_; }
    void set_parameters(const Eigen::Ref<dense_vector_t> &p) { p_ = p; }

   protected:
   private:
    bopt_index np_;
    dense_vector_t p_;
};

/**
 * @brief A twice-differentiable expression of the form \f$y = f(x)\f$
 *
 * @tparam ValueType
 */
template <typename ValueType>
class expression_scalar_tpl : public expression_base_tpl<ValueType>,
                              public evaluator_tpl<ValueType> {
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

    expression_scalar_tpl()
        : expression_base_tpl<ValueType>(0), evaluator_tpl<ValueType>(0, 1) {}

    expression_scalar_tpl(const bopt_index &sz_in, const bopt_index &np = 0)
        : expression_base_tpl<ValueType>(np),
          evaluator_tpl<ValueType>(sz_in, 1) {
        buffer_gradient_.dense = dense_vector_t::Zero(cols_gradient());
        buffer_hessian_.dense =
            dense_matrix_t::Zero(rows_hessian(), cols_hessian());
    }

    ~expression_scalar_tpl() = default;

    /**
     * @brief Evaluates an expression of the form `out` = f(x)
     *
     * @param arg
     * @param ret
     * @return return_status
     */
    evaluator::return_status eval(const Eigen::Ref<const dense_vector_t> &x,
                                  value_t &out) {
        DBGASSERT(this->check_input(x) && "eval input is invalid");
        return eval_impl(x, out);
    }

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
        DBGASSERT(this->check_input(x) && "gradient input is invalid");
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
        DBGASSERT(this->check_input(x) && "gradient input is invalid");
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
     * @brief Evaluates the dense jacobian for the expression \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_hessian(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_matrix_t> out) {
        DBGASSERT(this->check_input(x) && "hessian input is invalid");
        return eval_hessian_impl(x, out);
    }

    /**
     * @brief Evaluates the sparse hessian for the expression \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial^2 c}{\partial x^2}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_hessian(
        const Eigen::Ref<const dense_vector_t> &x, sparse_matrix_t &out) {
        DBGASSERT(this->check_input(x) && "hessian input is invalid");
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

    // Buffers for evaluation
    ValueType &buffer() { return buffer_; }
    vector_buffer_t &buffer_gradient() { return buffer_gradient_; }
    matrix_buffer_t &buffer_hessian() { return buffer_hessian_; }

   protected:
    virtual evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x, value_t &out) = 0;

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
    ValueType buffer_;
    vector_buffer_t buffer_gradient_;
    matrix_buffer_t buffer_hessian_;
};

template <typename ValueType>
class expression_tpl : public expression_base_tpl<ValueType>,
                       public evaluator_tpl<ValueType> {
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

    expression_tpl() = default;
    ~expression_tpl() = default;

    expression_tpl(const bopt_index &sz_in, const bopt_index &sz_out,
                   const bopt_index &np = 0)
        : expression_base_tpl<ValueType>(np),
          evaluator_tpl<ValueType>(sz_in, sz_out) {
        buffer_jacobian_.dense =
            dense_matrix_t::Zero(rows_jacobian(), cols_jacobian());
        buffer_hessian_.dense =
            dense_matrix_t::Zero(rows_hessian(), cols_hessian());
    }

    evaluator::return_status eval(const Eigen::Ref<const dense_vector_t> &x,
                                  Eigen::Ref<dense_vector_t> out) {
        DBGASSERT(this->check_input(x) && "eval input is invalid");
        return eval_impl(x, out);
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
        DBGASSERT(this->check_input(x) && "Jacobian input is invalid");
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
        DBGASSERT(this->check_input(x) && "Jacobian input is invalid");
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
     * @brief Evaluates the dense jacobian for the expression \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_hessian(
        const Eigen::Ref<const dense_vector_t> &x,
        const Eigen::Ref<const dense_vector_t> &lambda,
        Eigen::Ref<dense_matrix_t> out) {
        DBGASSERT(this->check_input(x) && "hessian input is invalid");
        return eval_hessian_impl(x, lambda, out);
    }

    /**
     * @brief Evaluates the sparse hessian for the expression \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial^2 c}{\partial x^2}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_hessian(
        const Eigen::Ref<const dense_vector_t> &x,
        const Eigen::Ref<const dense_vector_t> &lambda, sparse_matrix_t &out) {
        DBGASSERT(this->check_input(x) && "hessian input is invalid");
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
    virtual evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_vector_t> out) = 0;

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

template <typename ValueType>
class linear_expression_tpl : public evaluator_tpl<ValueType> {
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

    linear_expression_tpl() = default;
    ~linear_expression_tpl() = default;

    linear_expression_tpl(const bopt_index &sz_in, const bopt_index &sz_out)
        : evaluator_tpl<ValueType>(sz_in, sz_out) {
        buffer_A_.dense = dense_matrix_t::Zero(rows_A(), cols_A());
        buffer_b_.dense = dense_vector_t::Zero(rows_b());
    }

    /**
     * @brief Evaluates the dense jacobian for the expression \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_A(Eigen::Ref<dense_matrix_t> out) {
        return eval_A_impl(out);
    }

    /**
     * @brief Evaluates the sparse coefficient matrix for the linear expression
     * \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_A(sparse_matrix_t &out) {
        return eval_A_impl(out);
    }

    /**
     * @brief The number of rows within the coefficient matrix A
     *
     * @return bopt_index
     */
    virtual bopt_index rows_A() const { return this->sz_out(); }

    /**
     * @brief The number of columns within the coefficient matrix A
     * @return bopt_index
     */
    virtual bopt_index cols_A() const { return this->sz_in(); }

    /**
     * @brief Populates a sparse matrix with the sparsity pattern of the
     * coefficient matrix A
     *
     * @param A
     */
    virtual void sparsity_A(sparse_matrix_t &A) const {}

    /**
     * @brief Evaluates the dense jacobian for the expression \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_b(Eigen::Ref<dense_vector_t> out) {
        return eval_b_impl(out);
    }

    /**
     * @brief Evaluates the sparse coefficient matrix for the linear expression
     * \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_b(sparse_vector_t &out) {
        return eval_b_impl(out);
    }

    /**
     * @brief The number of rows within the coefficient vector b
     *
     * @return bopt_index
     */
    virtual bopt_index rows_b() const { return this->sz_out(); }

    /**
     * @brief Populates a sparse matrix with the sparsity pattern of the
     * coefficient matrix A
     *
     * @param A
     */
    virtual void sparsity_b(sparse_vector_t &b) const {}

    matrix_buffer_t &buffer_A() { return buffer_A_; }
    vector_buffer_t &buffer_b() { return buffer_b_; }

   protected:
    virtual evaluator::return_status eval_A_impl(
        Eigen::Ref<dense_matrix_t> out) {
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_A_impl(sparse_matrix_t &out) {
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_b_impl(
        Eigen::Ref<dense_vector_t> out) {
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_b_impl(sparse_vector_t &out) {
        return evaluator::return_status::NotImplemented;
    }

   private:
    matrix_buffer_t buffer_A_;
    vector_buffer_t buffer_b_;
};

template <typename ValueType>
class linear_scalar_expression_tpl : public evaluator_tpl<ValueType> {
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

    linear_scalar_expression_tpl() : evaluator_tpl<ValueType>(0, 1) {}

    linear_scalar_expression_tpl(const bopt_index &sz_in)
        : evaluator_tpl<ValueType>(sz_in, 1) {
        // Create buffers for dense evaluation
        buffer_a_.dense = dense_vector_t::Zero(rows_a());
    }

    /**
     * @brief Evaluates the dense jacobian for the expression \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_a(Eigen::Ref<dense_vector_t> out) {
        return eval_a_impl(out);
    }

    /**
     * @brief Evaluates the sparse coefficient matrix for the linear expression
     * \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_a(sparse_vector_t &out) {
        return eval_a_impl(out);
    }

    /**
     * @brief The number of rows within the coefficient vector a for the
     * expression a^T x + b
     *
     * @return bopt_index
     */
    virtual bopt_index rows_a() const { return this->sz_in(); }

    /**
     * @brief Populates a sparse matrix with the sparsity pattern of the
     * coefficient matrix A
     *
     * @param A
     */
    virtual void sparsity_a(sparse_vector_t &a) const {}

    /**
     * @brief Evaluates the dense jacobian for the expression \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_b(ValueType &out) { return eval_b_impl(out); }

    vector_buffer_t &buffer_a() { return buffer_a_; }

   protected:
    virtual evaluator::return_status eval_a_impl(
        Eigen::Ref<dense_vector_t> out) {
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_a_impl(sparse_vector_t &out) {
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_b_impl(ValueType &out) {
        return evaluator::return_status::NotImplemented;
    }

   private:
    vector_buffer_t buffer_a_;
};

template <typename ValueType>
class quadratic_scalar_expression_tpl : public evaluator_tpl<ValueType> {
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

    quadratic_scalar_expression_tpl() : evaluator_tpl<ValueType>(0, 1) {}

    quadratic_scalar_expression_tpl(const bopt_index &sz_in)
        : evaluator_tpl<ValueType>(sz_in, 1) {
        // Create buffers for dense evaluation
        buffer_A_.dense = dense_matrix_t::Zero(rows_A(), cols_A());
        buffer_b_.dense = dense_vector_t::Zero(rows_b());
    }

    /**
     * @brief Evaluates the dense jacobian for the expression \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_A(Eigen::Ref<dense_matrix_t> out) {
        return eval_A_impl(out);
    }

    /**
     * @brief Evaluates the sparse coefficient matrix for the quadratic
     * expression
     * \f$c(x)\f$ (i.e. \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_A(sparse_matrix_t &out) {
        return eval_A_impl(out);
    }

    /**
     * @brief The number of rows within the coefficient matrix A
     *
     * @return bopt_index
     */
    virtual bopt_index rows_A() const { return this->sz_in(); }

    /**
     * @brief The number of columns within the coefficient matrix A
     *
     * @return bopt_index
     */
    virtual bopt_index cols_A() const { return this->sz_in(); }

    /**
     * @brief Populates a sparse matrix with the sparsity pattern of the
     * coefficient matrix A
     *
     * @param A
     */
    virtual void sparsity_A(sparse_matrix_t &A) const {}

    /**
     * @brief Evaluates the dense jacobian for the expression \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_b(Eigen::Ref<dense_vector_t> out) {
        return eval_b_impl(out);
    }

    /**
     * @brief Evaluates the dense jacobian for the expression \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_b(sparse_vector_t &out) {
        return eval_b_impl(out);
    }

    /**
     * @brief The number of rows within the coefficient vector b
     *
     * @return bopt_index
     */
    virtual bopt_index rows_b() const { return this->sz_in(); }

    /**
     * @brief Populates a sparse matrix with the sparsity pattern of the
     * coefficient vector b
     *
     * @param A
     */
    virtual void sparsity_b(sparse_vector_t &b) const {}

    evaluator::return_status eval_c(value_t &out) { return eval_c_impl(out); }

    matrix_buffer_t &buffer_A() { return buffer_A_; }
    vector_buffer_t &buffer_b() { return buffer_b_; }

   protected:
    virtual evaluator::return_status eval_A_impl(
        Eigen::Ref<dense_matrix_t> out) {
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_A_impl(sparse_matrix_t &out) {
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_b_impl(
        Eigen::Ref<dense_vector_t> out) {
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_b_impl(sparse_vector_t &out) {
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_c_impl(value_t &out) {
        return evaluator::return_status::NotImplemented;
    }

   private:
    matrix_buffer_t buffer_A_;
    vector_buffer_t buffer_b_;
};

}  // namespace bopt
