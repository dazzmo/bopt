#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <memory>

#include "bopt/bounds.hpp"
#include "bopt/evaluator.hpp"
#include "bopt/logging.hpp"

namespace bopt {

/**
 * @brief A twice-differentiable expression of the form \f$y = f(x)\f$
 *
 * @tparam ValueType
 */
template <typename ValueType>
class expression_scalar_tpl : public virtual evaluator_tpl<ValueType> {
   public:
    using typename evaluator_tpl<ValueType>::value_t;
    using typename evaluator_tpl<ValueType>::dense_vector_t;
    using typename evaluator_tpl<ValueType>::sparse_vector_t;
    using typename evaluator_tpl<ValueType>::dense_matrix_t;
    using typename evaluator_tpl<ValueType>::sparse_matrix_t;

    using id_type = bopt_index;
    using string_type = std::string;

    expression_scalar_tpl() = default;
    ~expression_scalar_tpl() = default;

    /**
     * @brief Evaluates an expression of the form `out` = f(x)
     *
     * @param arg
     * @param ret
     * @return return_status
     */
    evaluator::return_status eval(const dense_vector_t &x, value_t &out) {
        DBGASSERT(check_input(x) && "eval input is invalid");
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
    evaluator::return_status eval_gradient(const dense_vector_t &x,
                                           Eigen::Ref<dense_vector_t> out) {
        DBGASSERT(check_input(x) && "gradient input is invalid");
        return eval_gradient_impl(x, out);
    }

    /**
     * @brief Evaluates the sparse gradient for the expression \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_gradient(const dense_vector_t &x,
                                           Eigen::Ref<sparse_vector_t> out) {
        DBGASSERT(check_input(x) && "gradient input is invalid");
        return eval_gradient_impl(x, out);
    }

    /**
     * @brief The number of rows within the expression gradient
     *
     * @return bopt_index
     */
    virtual bopt_index gradient_rows() const = 0;

    /**
     * @brief The number of columns within the expression gradient
     *
     * @return bopt_index
     */
    virtual bopt_index gradient_cols() const = 0;

    /**
     * @brief Populates a sparse matrix with the sparsity pattern of the
     * gradient
     *
     * @param gradient
     */
    virtual void gradient_sparsity(sparse_matrix_t &gradient) const {}

    /**
     * @brief Evaluates the dense jacobian for the expression \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_hessian(const dense_vector_t &x,
                                          Eigen::Ref<dense_matrix_t> out) {
        DBGASSERT(check_input(x) && "hessian input is invalid");
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
    evaluator::return_status eval_hessian(const dense_vector_t &x,
                                          Eigen::Ref<sparse_matrix_t> out) {
        DBGASSERT(check_input(x) && "hessian input is invalid");
        return eval_hessian_impl(x, out);
    }

    /**
     * @brief The number of rows within the expression hessian
     *
     * @return bopt_index
     */
    virtual bopt_index hessian_rows() const = 0;

    /**
     * @brief The number of columns within the expression hessian
     *
     * @return bopt_index
     */
    virtual bopt_index hessian_cols() const = 0;

    /**
     * @brief Populates a sparse matrix with the sparsity pattern of the
     * hessian
     *
     * @param hessian
     */
    virtual void hessian_sparsity(sparse_matrix_t &hessian) const {}

   protected:
    virtual evaluator::return_status eval_impl(const dense_vector_t &x,
                                               value_t &out) = 0;

    virtual evaluator::return_status eval_gradient_impl(
        const dense_vector_t &x, Eigen::Ref<dense_vector_t> out) {
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_gradient_impl(
        const dense_vector_t &x, Eigen::Ref<sparse_vector_t> out) {
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_hessian_impl(
        const dense_vector_t &x, Eigen::Ref<dense_matrix_t> out) {
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_hessian_impl(
        const dense_vector_t &x, Eigen::Ref<sparse_matrix_t> out) {
        return evaluator::return_status::NotImplemented;
    }

   private:
};

template <typename ValueType>
class expression_tpl : public virtual evaluator_tpl<ValueType> {
   public:
    using typename evaluator_tpl<ValueType>::value_t;
    using typename evaluator_tpl<ValueType>::dense_vector_t;
    using typename evaluator_tpl<ValueType>::sparse_vector_t;
    using typename evaluator_tpl<ValueType>::dense_matrix_t;
    using typename evaluator_tpl<ValueType>::sparse_matrix_t;

    typedef bopt_index id_type;
    typedef std::string string_type;

    expression_tpl() = default;
    ~expression_tpl() = default;

    evaluator::return_status eval(const dense_vector_t &x,
                                  Eigen::Ref<dense_vector_t> out) {
        DBGASSERT(check_input(x) && "eval input is invalid");
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
    evaluator::return_status eval_jacobian(const dense_vector_t &x,
                                           Eigen::Ref<dense_matrix_t> out) {
        DBGASSERT(check_input(x) && "Jacobian input is invalid");
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
    evaluator::return_status eval_jacobian(const dense_vector_t &x,
                                           Eigen::Ref<sparse_matrix_t> out) {
        DBGASSERT(check_input(x) && "Jacobian input is invalid");
        return eval_jacobian_impl(x, out);
    }

    /**
     * @brief The number of rows within the expression jacobian
     *
     * @return bopt_index
     */
    virtual bopt_index jacobian_rows() const = 0;

    /**
     * @brief The number of columns within the expression jacobian
     *
     * @return bopt_index
     */
    virtual bopt_index jacobian_cols() const = 0;

    /**
     * @brief Populates a sparse matrix with the sparsity pattern of the
     * jacobian
     *
     * @param jacobian
     */
    virtual void jacobian_sparsity(sparse_matrix_t &jacobian) const {}

    /**
     * @brief Evaluates the dense jacobian for the expression \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_hessian(const dense_vector_t &x,
                                          const dense_vector_t &lambda,
                                          Eigen::Ref<dense_matrix_t> out) {
        DBGASSERT(check_input(x) && "hessian input is invalid");
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
    evaluator::return_status eval_hessian(const dense_vector_t &x,
                                          const dense_vector_t &lambda,
                                          Eigen::Ref<sparse_matrix_t> out) {
        DBGASSERT(check_input(x) && "hessian input is invalid");
        return eval_hessian_impl(x, lambda, out);
    }

    /**
     * @brief The number of rows within the expression hessian
     *
     * @return bopt_index
     */
    virtual bopt_index hessian_rows() const = 0;

    /**
     * @brief The number of columns within the expression hessian
     *
     * @return bopt_index
     */
    virtual bopt_index hessian_cols() const = 0;

    /**
     * @brief Populates a sparse matrix with the sparsity pattern of the
     * hessian
     *
     * @param hessian
     */
    virtual void hessian_sparsity(sparse_matrix_t &hessian) const {}

   protected:
    evaluator::return_status eval_impl(const dense_vector_t &x,
                                       Eigen::Ref<dense_vector_t> out) = 0;

    evaluator::return_status eval_jacobian_impl(
        const dense_vector_t &x, Eigen::Ref<dense_matrix_t> out) {
        return evaluator::return_status::NotImplemented;
    }

    evaluator::return_status eval_jacobian_impl(
        const dense_vector_t &x, Eigen::Ref<sparse_matrix_t> out) {
        return evaluator::return_status::NotImplemented;
    }

    evaluator::return_status eval_hessian_impl(const dense_vector_t &x,
                                               const dense_vector_t &lamba,
                                               Eigen::Ref<dense_matrix_t> out) {
        return evaluator::return_status::NotImplemented;
    }

    evaluator::return_status eval_hessian_impl(
        const dense_vector_t &x, const dense_vector_t &lamba,
        Eigen::Ref<sparse_matrix_t> out) {
        return evaluator::return_status::NotImplemented;
    }

   private:
};

template <typename ValueType>
class linear_expression_tpl : public expression_tpl<ValueType> {
   public:
    using typename expression_tpl<ValueType>::value_t;
    using typename expression_tpl<ValueType>::dense_vector_t;
    using typename expression_tpl<ValueType>::sparse_vector_t;
    using typename expression_tpl<ValueType>::dense_matrix_t;
    using typename expression_tpl<ValueType>::sparse_matrix_t;

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
    evaluator::return_status eval_A(Eigen::Ref<sparse_matrix_t> out) {
        return eval_A_impl(out);
    }

    evaluator::return_status eval_jacobian_impl(
        const dense_vector_t &x, Eigen::Ref<dense_matrix_t> out) override {
        return eval_A(out);
    }

    evaluator::return_status eval_jacobian_impl(
        const dense_vector_t &x, Eigen::Ref<sparse_matrix_t> out) override {
        return eval_A(out);
    }

    // evaluator::return_status eval_hessian_impl(
    //     const dense_vector_t &x, Eigen::Ref<dense_matrix> out) override {
    //     return eval_A(out);
    // }

    // evaluator::return_status eval_hessian_impl(
    //     const dense_vector_t &x, Eigen::Ref<sparse_matrix_t> out) override {
    //     return eval_A(out);
    // }

    /**
     * @brief The number of rows within the coefficient matrix A
     *
     * @return bopt_index
     */
    virtual bopt_index A_rows() const = 0;

    /**
     * @brief The number of columns within the coefficient matrix A
     * @return bopt_index
     */
    virtual bopt_index A_cols() const = 0;

    /**
     * @brief Populates a sparse matrix with the sparsity pattern of the
     * coefficient matrix A
     *
     * @param A
     */
    virtual void A_sparsity(sparse_matrix_t &A) const {}

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
    evaluator::return_status eval_b(Eigen::Ref<sparse_vector_t> out) {
        return eval_b_impl(out);
    }

    /**
     * @brief The number of rows within the coefficient vector b
     *
     * @return bopt_index
     */
    virtual bopt_index b_rows() const = 0;

    /**
     * @brief Populates a sparse matrix with the sparsity pattern of the
     * coefficient matrix A
     *
     * @param A
     */
    virtual void b_sparsity(sparse_vector_t &b) const {}

   protected:
    evaluator::return_status eval_A_impl(Eigen::Ref<dense_matrix_t> out) {
        return evaluator::return_status::NotImplemented;
    }

    evaluator::return_status eval_A_impl(Eigen::Ref<sparse_matrix_t> out) {
        return evaluator::return_status::NotImplemented;
    }

    evaluator::return_status eval_b_impl(Eigen::Ref<dense_vector_t> out) {
        return evaluator::return_status::NotImplemented;
    }

    evaluator::return_status eval_b_impl(Eigen::Ref<sparse_vector_t> out) {
        return evaluator::return_status::NotImplemented;
    }

   private:
};

template <typename ValueType>
class linear_scalar_expression_tpl : public expression_scalar_tpl<ValueType> {
   public:
    using typename expression_scalar_tpl<ValueType>::value_t;
    using typename expression_scalar_tpl<ValueType>::dense_vector_t;
    using typename expression_scalar_tpl<ValueType>::sparse_vector_t;
    using typename expression_scalar_tpl<ValueType>::dense_matrix_t;
    using typename expression_scalar_tpl<ValueType>::sparse_matrix_t;

    /**
     * @brief Evaluates the dense jacobian for the expression \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_a(Eigen::Ref<dense_matrix_t> out) {
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
    evaluator::return_status eval_a(Eigen::Ref<sparse_matrix_t> out) {
        return eval_a_impl(out);
    }

    evaluator::return_status eval_gradient_impl(
        const dense_vector_t &x, Eigen::Ref<dense_vector_t> out) override {
        return eval_a(out);
    }

    evaluator::return_status eval_gradient_impl(
        const dense_vector_t &x, Eigen::Ref<sparse_vector_t> out) override {
        return eval_a(out);
    }

    // evaluator::return_status eval_hessian_impl(
    //     const dense_vector_t &x, Eigen::Ref<dense_matrix> out) override {
    //     return eval_a(out);
    // }

    // evaluator::return_status eval_hessian_impl(
    //     const dense_vector_t &x, Eigen::Ref<sparse_matrix_t> out) override {
    //     return eval_a(out);
    // }

    /**
     * @brief The number of rows within the coefficient matrix A
     *
     * @return bopt_index
     */
    virtual bopt_index a_rows() const = 0;

    /**
     * @brief Populates a sparse matrix with the sparsity pattern of the
     * coefficient matrix A
     *
     * @param A
     */
    virtual void a_sparsity(sparse_vector_t &a) const {}

    /**
     * @brief Evaluates the dense jacobian for the expression \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_b(ValueType &out) { return eval_b_impl(out); }

   protected:
    evaluator::return_status eval_a_impl(Eigen::Ref<dense_vector_t> out) {
        return evaluator::return_status::NotImplemented;
    }

    evaluator::return_status eval_a_impl(Eigen::Ref<sparse_vector_t> out) {
        return evaluator::return_status::NotImplemented;
    }

    evaluator::return_status eval_b_impl(ValueType &out) {
        return evaluator::return_status::NotImplemented;
    }

   private:
};

template <typename ValueType>
class quadratic_scalar_expression_tpl
    : public expression_scalar_tpl<ValueType> {
   public:
    using typename expression_scalar_tpl<ValueType>::value_t;
    using typename expression_scalar_tpl<ValueType>::dense_vector_t;
    using typename expression_scalar_tpl<ValueType>::sparse_vector_t;
    using typename expression_scalar_tpl<ValueType>::dense_matrix_t;
    using typename expression_scalar_tpl<ValueType>::sparse_matrix_t;

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
    evaluator::return_status eval_A(Eigen::Ref<sparse_matrix_t> out) {
        return eval_A_impl(out);
    }

    evaluator::return_status eval_gradient(
        const dense_vector_t &x, Eigen::Ref<dense_vector_t> out) override {
        dense_matrix_t A;
        dense_vector_t b;
        eval_A(A);
        eval_b(b);
        out = 2.0 * A * x + b;
        return evaluator::return_status::Success;
    }

    evaluator::return_status eval_gradient_impl(
        const dense_vector_t &x, Eigen::Ref<sparse_vector_t> out) override {
        return eval_A(out);
    }

    // evaluator::return_status eval_hessian_impl(
    //     const dense_vector_t &x, Eigen::Ref<dense_matrix> out) override {
    //     return eval_a(out);
    // }

    // evaluator::return_status eval_hessian_impl(
    //     const dense_vector_t &x, Eigen::Ref<sparse_matrix_t> out) override {
    //     return eval_a(out);
    // }

    /**
     * @brief The number of rows within the coefficient matrix A
     *
     * @return bopt_index
     */
    virtual bopt_index a_rows() const = 0;

    /**
     * @brief Populates a sparse matrix with the sparsity pattern of the
     * coefficient matrix A
     *
     * @param A
     */
    virtual void a_sparsity(sparse_vector_t &a) const {}

    /**
     * @brief Evaluates the dense jacobian for the expression \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_b(ValueType &out) { return eval_b_impl(out); }

   protected:
    evaluator::return_status eval_a_impl(Eigen::Ref<dense_vector_t> out) {
        return evaluator::return_status::NotImplemented;
    }

    evaluator::return_status eval_a_impl(Eigen::Ref<sparse_vector_t> out) {
        return evaluator::return_status::NotImplemented;
    }

    evaluator::return_status eval_b_impl(ValueType &out) {
        return evaluator::return_status::NotImplemented;
    }

   private:
};

}  // namespace bopt
