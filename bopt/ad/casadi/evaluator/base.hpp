#pragma once

#include "bopt/ad/casadi/utils.hpp"
#include "bopt/evaluator/base.hpp"
#include "bopt/logging.hpp"

namespace bopt {
namespace casadi {

namespace evaluator {

using bopt::evaluator::return_status;

/**
 * @brief Generic expression evaluator, designed to compute the expression,
 * jacobian and hessian of the expression.
 *
 * @tparam T
 */
class scalar : public bopt::evaluator::scalar {
   public:
    using base_t = bopt::evaluator::scalar;

    using typename base_t::dense_vector_t;
    using typename base_t::value_t;

    scalar(const sym_t &expression, const sym_vector_t &x,
           const sym_vector_t &p, bool codegen = false);

   protected:
    return_status eval_impl(const Eigen::Ref<const dense_vector_t> &x,
                            value_t &out) override;

   private:
    function_t fun_;
};

/**
 * @brief Generic vector expression evaluator.
 *
 * @tparam
 */
class vector : public bopt::evaluator::vector {
   public:
    using base_t = bopt::evaluator::vector;

    using typename base_t::dense_matrix_t;
    using typename base_t::dense_vector_t;
    using typename base_t::sparse_matrix_t;
    using typename base_t::sparse_vector_t;
    using typename base_t::value_t;

    vector(const sym_t &expression, const sym_vector_t &x,
           const sym_vector_t &p, bool codegen = false);

   protected:
    evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_vector_t> out) override;

   private:
    function_t fun_;
};

/**
 * @brief Generic vector expression evaluator.
 *
 * @tparam
 */
class matrix : public bopt::evaluator::matrix {
   public:
    using base_t = bopt::evaluator::matrix;

    using typename base_t::dense_matrix_t;
    using typename base_t::dense_vector_t;
    using typename base_t::sparse_matrix_t;
    using typename base_t::sparse_vector_t;
    using typename base_t::value_t;

    matrix(const sym_t &expression, const sym_vector_t &x,
           const sym_vector_t &p, bool codegen = false);

   protected:
    evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_matrix_t> out) override;

    evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        sparse_matrix_t &out) override;

    void get_sparsity_impl(sparse_matrix_t &out) const override;

   private:
    function_t fun_;
};

}  // namespace evaluator

}  // namespace casadi

}  // namespace bopt