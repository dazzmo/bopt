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

    using typename base_t::value_t;
    using typename base_t::dense_vector_t;

    scalar_evaluator(const sym_t &expression, const sym_vector_t &x,
                     const sym_vector_t &p, bool codegen = false);

   protected:
    evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x, value_t &out) override;

   private:
    function_t fun_;
};

// /**
//  * @brief Generic vector expression evaluator.
//  *
//  * @tparam
//  */
// class vector_evaluator : public bopt::vector_evaluator_tpl<double> {
//    public:
//     using base = bopt::vector_evaluator_tpl<double>;

//     using typename base::dense_matrix_t;
//     using typename base::dense_vector_t;
//     using typename base::sparse_matrix_t;
//     using typename base::sparse_vector_t;
//     using typename base::value_t;

//     vector_evaluator(const sym_t &expression, const sym_vector_t &x,
//                      const sym_vector_t &p, bool codegen = false);

//    protected:
//     evaluator::return_status eval_impl(
//         const Eigen::Ref<const dense_vector_t> &x,
//         Eigen::Ref<dense_vector_t> out) override;

//    private:
//     function_t fun_;
// };

// class gradient_evaluator : public bopt::gradient_evaluator_tpl<double> {
//    public:
//     using base = bopt::gradient_evaluator_tpl<double>;

//     using typename base::dense_matrix_t;
//     using typename base::dense_vector_t;
//     using typename base::sparse_matrix_t;
//     using typename base::sparse_vector_t;
//     using typename base::value_t;

//     gradient_evaluator(const sym_t &expression, const sym_vector_t &x,
//                        const sym_vector_t &p, bool densify = false,
//                        bool codegen = false);

//     void sparsity_gradient(sparse_vector_t &out) const override;

//    protected:
//     evaluator::return_status eval_gradient_impl(
//         const Eigen::Ref<const dense_vector_t> &x,
//         Eigen::Ref<dense_vector_t> out) override;

//     evaluator::return_status eval_gradient_impl(
//         const Eigen::Ref<const dense_vector_t> &x,
//         sparse_vector_t &out) override;

//    private:
//     function_t fun_;
// };

// class jacobian_evaluator : public bopt::jacobian_evaluator_tpl<double> {
//    public:
//     using base = bopt::jacobian_evaluator_tpl<double>;

//     using typename base::dense_matrix_t;
//     using typename base::dense_vector_t;
//     using typename base::sparse_matrix_t;
//     using typename base::sparse_vector_t;
//     using typename base::value_t;

//     jacobian_evaluator(const sym_t &expression, const sym_vector_t &x,
//                        const sym_vector_t &p, bool densify = false,
//                        bool codegen = false);

//     void sparsity_jacobian(sparse_matrix_t &out) const override;

//    protected:
//     evaluator::return_status eval_jacobian_impl(
//         const Eigen::Ref<const dense_vector_t> &x,
//         Eigen::Ref<dense_matrix_t> out) override;

//     evaluator::return_status eval_jacobian_impl(
//         const Eigen::Ref<const dense_vector_t> &x,
//         sparse_matrix_t &out) override;

//    private:
//     function_t fun_;
// };

// class hessian_evaluator : public bopt::hessian_evaluator_tpl<double> {
//    public:
//     using base = bopt::hessian_evaluator_tpl<double>;

//     using typename base::dense_matrix_t;
//     using typename base::dense_vector_t;
//     using typename base::sparse_matrix_t;
//     using typename base::sparse_vector_t;
//     using typename base::value_t;

//     hessian_evaluator(const sym_t &expression, const sym_vector_t &x,
//                       const sym_vector_t &p, bool densify = false,
//                       bool codegen = false);

//     void sparsity_hessian(sparse_matrix_t &out) const override;

//    protected:
//     evaluator::return_status eval_hessian_impl(
//         const Eigen::Ref<const dense_vector_t> &x,
//         const Eigen::Ref<const dense_vector_t> &lambda,
//         Eigen::Ref<dense_matrix_t> out) override;

//     evaluator::return_status eval_hessian_impl(
//         const Eigen::Ref<const dense_vector_t> &x,
//         const Eigen::Ref<const dense_vector_t> &lambda,
//         sparse_matrix_t &out) override;

//    private:
//     function_t fun_;
// };

// class differentiable_scalar_evaluator
//     : public bopt::differentiable_scalar_evaluator_tpl<double> {
//    public:
//     using base = bopt::differentiable_scalar_evaluator_tpl<double>;

//     using typename base::dense_matrix_t;
//     using typename base::dense_vector_t;
//     using typename base::sparse_matrix_t;
//     using typename base::sparse_vector_t;
//     using typename base::value_t;

//     differentiable_scalar_evaluator(const sym_t &expression,
//                                     const sym_vector_t &x,
//                                     const sym_vector_t &p, bool densify = false,
//                                     bool codegen = false);
// };

// class differentiable_vector_evaluator
//     : public bopt::differentiable_vector_evaluator_tpl<double> {
//    public:
//     using base = bopt::differentiable_vector_evaluator_tpl<double>;

//     using typename base::dense_matrix_t;
//     using typename base::dense_vector_t;
//     using typename base::sparse_matrix_t;
//     using typename base::sparse_vector_t;
//     using typename base::value_t;

//     differentiable_vector_evaluator(const sym_t &expression,
//                                     const sym_vector_t &x,
//                                     const sym_vector_t &p, bool densify = false,
//                                     bool codegen = false);
// };

// /**
//  * @brief Generic expression evaluator, designed to compute the expression,
//  * jacobian and hessian of the expression.
//  *
//  * @tparam T
//  */
// class linear_scalar_evaluator
//     : public bopt::linear_scalar_evaluator_tpl<double> {
//    public:
//     using base = bopt::linear_scalar_evaluator_tpl<double>;

//     using typename base::dense_matrix_t;
//     using typename base::dense_vector_t;
//     using typename base::sparse_matrix_t;
//     using typename base::sparse_vector_t;
//     using typename base::value_t;

//     linear_scalar_evaluator(const sym_t &expression, const sym_vector_t &x,
//                             const sym_vector_t &p, bool densify = false,
//                             bool codegen = false);

//     void sparsity_a(sparse_vector_t &out) const override;

//    protected:
//     evaluator::return_status eval_a_impl(
//         Eigen::Ref<dense_vector_t> out) override;

//     evaluator::return_status eval_a_impl(sparse_vector_t &out) override;

//     evaluator::return_status eval_b_impl(value_t &out) override;

//    private:
//     function_t a_;
//     function_t b_;
// };

// /**
//  * @brief Generic expression evaluator, designed to compute the expression,
//  * jacobian and hessian of the expression.
//  *
//  * @tparam T
//  */
// class linear_vector_evaluator
//     : public bopt::linear_vector_evaluator_tpl<double> {
//    public:
//     using base = bopt::linear_vector_evaluator_tpl<double>;

//     using typename base::dense_matrix_t;
//     using typename base::dense_vector_t;
//     using typename base::sparse_matrix_t;
//     using typename base::sparse_vector_t;
//     using typename base::value_t;

//     linear_vector_evaluator(const sym_t &expression, const sym_vector_t &x,
//                             const sym_vector_t &p, bool densify = false,
//                             bool codegen = false);

//     void sparsity_A(sparse_matrix_t &out) const override;
//     void sparsity_b(sparse_vector_t &out) const override;

//    protected:
//     evaluator::return_status eval_A_impl(
//         Eigen::Ref<dense_matrix_t> out) override;
//     evaluator::return_status eval_b_impl(
//         Eigen::Ref<dense_vector_t> out) override;

//    private:
//     function_t A_;
//     function_t b_;
// };

// /**
//  * @brief Generic expression evaluator, designed to compute the expression,
//  * jacobian and hessian of the expression.
//  *
//  * @tparam T
//  */
// class quadratic_scalar_evaluator
//     : public bopt::quadratic_scalar_evaluator_tpl<double> {
//    public:
//     using base = bopt::quadratic_scalar_evaluator_tpl<double>;

//     using typename base::dense_matrix_t;
//     using typename base::dense_vector_t;
//     using typename base::sparse_matrix_t;
//     using typename base::sparse_vector_t;
//     using typename base::value_t;

//     quadratic_scalar_evaluator(const sym_t &expression, const sym_vector_t &x,
//                                const sym_vector_t &p, bool densify = false,
//                                bool codegen = false);

//     void sparsity_A(sparse_matrix_t &out) const override;
//     void sparsity_b(sparse_vector_t &out) const override;

//    protected:
//     evaluator::return_status eval_A_impl(
//         Eigen::Ref<dense_matrix_t> out) override;

//     evaluator::return_status eval_A_impl(sparse_matrix_t &out) override;

//     evaluator::return_status eval_b_impl(
//         Eigen::Ref<dense_vector_t> out) override;

//     evaluator::return_status eval_b_impl(sparse_vector_t &out) override;

//     evaluator::return_status eval_c_impl(value_t &out) override;

//    private:
//     function_t A_;
//     function_t b_;
//     function_t c_;
// };

}  // namespace casadi
}  // namespace bopt
