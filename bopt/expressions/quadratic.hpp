#pragma once

#include "bopt/evaluator.hpp"
#include "bopt/logging.hpp"

namespace bopt {

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

    quadratic_scalar_expression_tpl(const bopt_index &sz_in,
                                    const bopt_index &sz_p = 0)
        : evaluator_tpl<ValueType>(sz_in, 1, sz_p) {
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
    ValueType &buffer_c() { return buffer_c_; }

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
    ValueType buffer_c_;
};

}  // namespace bopt
