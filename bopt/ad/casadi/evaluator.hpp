#pragma once

#include <casadi/casadi.hpp>
#include <cassert>
#include <filesystem>

#include "bopt/ad/casadi/utils.hpp"
#include "bopt/evaluator.hpp"
#include "bopt/logging.hpp"

namespace bopt {
namespace casadi {

/**
 * @brief Generic expression evaluator, designed to compute the expression,
 * jacobian and hessian of the expression.
 *
 * @tparam T
 */
class scalar_evaluator : public bopt::scalar_evaluator_tpl<double> {
   public:
    using base_t = bopt::scalar_evaluator_tpl<double>;

    using typename base_t::dense_vector_t;
    using typename base_t::value_t;

    scalar_evaluator(const sym_t &expression, const sym_vector_t &x,
                     const sym_vector_t &p, bool codegen = false);

   protected:
    evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x, value_t &out) override;

   private:
    function_t fun_;
};

/**
 * @brief Generic vector expression evaluator.
 *
 * @tparam
 */
class vector_evaluator : public bopt::vector_evaluator_tpl<double> {
   public:
    using base_t = bopt::vector_evaluator_tpl<double>;

    using typename base_t::dense_matrix_t;
    using typename base_t::dense_vector_t;
    using typename base_t::sparse_matrix_t;
    using typename base_t::sparse_vector_t;
    using typename base_t::value_t;

    vector_evaluator(const sym_t &expression, const sym_vector_t &x,
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
class matrix_evaluator : public bopt::matrix_evaluator_tpl<double> {
   public:
    using base_t = bopt::matrix_evaluator_tpl<double>;

    using typename base_t::dense_matrix_t;
    using typename base_t::dense_vector_t;
    using typename base_t::sparse_matrix_t;
    using typename base_t::sparse_vector_t;
    using typename base_t::value_t;

    matrix_evaluator(const sym_t &expression, const sym_vector_t &x,
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

class linear_vector_evaluator
    : public bopt::linear_vector_evaluator_tpl<double> {
   public:
    using base = bopt::linear_vector_evaluator_tpl<double>;

    using typename base::dense_matrix_t;
    using typename base::dense_vector_t;
    using typename base::sparse_matrix_t;
    using typename base::sparse_vector_t;
    using typename base::value_t;

    linear_vector_evaluator(const sym_t &expression, const sym_vector_t &x,
                            const sym_vector_t &p, bool densify = false,
                            bool codegen = false);

   protected:
    evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_vector_t> out) override;

    void get_A_sparsity_impl(sparse_matrix_t &out) const override;

    void get_b_sparsity_impl(sparse_vector_t &out) const override;

    evaluator::return_status eval_A_impl(
        Eigen::Ref<dense_matrix_t> out) override;

    evaluator::return_status eval_A_impl(sparse_matrix_t &out) override;

    evaluator::return_status eval_b_impl(
        Eigen::Ref<dense_vector_t> out) override;

    evaluator::return_status eval_b_impl(sparse_vector_t &out) override;

   private:
    function_t f_;
    function_t A_;
    function_t b_;
};

class linear_scalar_evaluator
    : public bopt::linear_scalar_evaluator_tpl<double> {
   public:
    using base = bopt::linear_scalar_evaluator_tpl<double>;

    using typename base::dense_matrix_t;
    using typename base::dense_vector_t;
    using typename base::sparse_matrix_t;
    using typename base::sparse_vector_t;
    using typename base::value_t;

    linear_scalar_evaluator(const sym_t &expression, const sym_vector_t &x,
                            const sym_vector_t &p, bool densify = false,
                            bool codegen = false);

   protected:
    evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x, value_t &out) override;

    void get_a_sparsity_impl(sparse_vector_t &out) const override;

    evaluator::return_status eval_a_impl(
        Eigen::Ref<dense_vector_t> out) override;

    evaluator::return_status eval_a_impl(sparse_vector_t &out) override;

    evaluator::return_status eval_b_impl(value_t &out) override;

   private:
    function_t f_;
    function_t a_;
    function_t b_;
};

class quadratic_scalar_evaluator
    : public bopt::quadratic_scalar_evaluator_tpl<double> {
   public:
    using base = bopt::quadratic_scalar_evaluator_tpl<double>;

    using typename base::dense_matrix_t;
    using typename base::dense_vector_t;
    using typename base::sparse_matrix_t;
    using typename base::sparse_vector_t;
    using typename base::value_t;

    quadratic_scalar_evaluator(const sym_t &expression, const sym_vector_t &x,
                               const sym_vector_t &p, bool densify = false,
                               bool codegen = false);

   protected:
    evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x, value_t &out) override;

    void get_A_sparsity_impl(sparse_matrix_t &out) const override;

    void get_b_sparsity_impl(sparse_vector_t &out) const override;

    evaluator::return_status eval_A_impl(
        Eigen::Ref<dense_matrix_t> out) override;

    evaluator::return_status eval_A_impl(sparse_matrix_t &out) override;

    evaluator::return_status eval_b_impl(
        Eigen::Ref<dense_vector_t> out) override;

    evaluator::return_status eval_b_impl(sparse_vector_t &out) override;

    evaluator::return_status eval_c_impl(value_t &out) override;

   private:
    function_t f_;
    function_t A_;
    function_t b_;
    function_t c_;
};

class differentiable_scalar_evaluator
    : public bopt::differentiable_scalar_evaluator_tpl<double> {
   public:
    using base_t = bopt::differentiable_scalar_evaluator_tpl<double>;

    using typename base_t::dense_matrix_t;
    using typename base_t::dense_vector_t;
    using typename base_t::sparse_matrix_t;
    using typename base_t::sparse_vector_t;
    using typename base_t::value_t;

    differentiable_scalar_evaluator(const sym_t &expression,
                                    const sym_vector_t &x,
                                    const sym_vector_t &p, bool densify = false,
                                    bool codegen = false);

   protected:
    evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x, value_t &out) override;

    evaluator::return_status eval_gradient_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_vector_t> out) override;

    evaluator::return_status eval_gradient_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        sparse_vector_t &out) override;

    void get_gradient_sparsity_impl(sparse_vector_t &out) const override;

    evaluator::return_status eval_hessian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        const Eigen::Ref<const dense_vector_t> &lambda,
        Eigen::Ref<dense_matrix_t> out) override;

    evaluator::return_status eval_hessian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        const Eigen::Ref<const dense_vector_t> &lambda,
        sparse_matrix_t &out) override;

    void get_hessian_sparsity_impl(sparse_matrix_t &out) const override;

   private:
    function_t fun_;
    function_t grd_;
    function_t hes_;
};

class differentiable_vector_evaluator
    : public bopt::differentiable_vector_evaluator_tpl<double> {
   public:
    using base_t = bopt::differentiable_vector_evaluator_tpl<double>;

    using typename base_t::dense_matrix_t;
    using typename base_t::dense_vector_t;
    using typename base_t::sparse_matrix_t;
    using typename base_t::sparse_vector_t;
    using typename base_t::value_t;

    differentiable_vector_evaluator(const sym_t &expression,
                                    const sym_vector_t &x,
                                    const sym_vector_t &p, bool densify = false,
                                    bool codegen = false);

   protected:
    evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_vector_t> out) override;

    evaluator::return_status eval_jacobian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_matrix_t> out) override;

    evaluator::return_status eval_jacobian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        sparse_matrix_t &out) override;

    void get_jacobian_sparsity_impl(sparse_matrix_t &out) const override;

    evaluator::return_status eval_hessian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        const Eigen::Ref<const dense_vector_t> &lambda,
        Eigen::Ref<dense_matrix_t> out) override;

    evaluator::return_status eval_hessian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        const Eigen::Ref<const dense_vector_t> &lambda,
        sparse_matrix_t &out) override;

    void get_hessian_sparsity_impl(sparse_matrix_t &out) const override;

   private:
    function_t fun_;
    function_t jac_;
    function_t hes_;
};

}  // namespace casadi
}  // namespace bopt
