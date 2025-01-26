#pragma once

#include "bopt/ad/casadi/utils.hpp"
#include "bopt/evaluator/differentiable.hpp"
#include "bopt/logging.hpp"

namespace bopt {
namespace casadi {

namespace evaluator {
namespace differentiable {

using bopt::evaluator::return_status;

class scalar : public bopt::evaluator::differentiable::scalar {
   public:
    using base_t = bopt::evaluator::differentiable::scalar;

    using typename base_t::dense_matrix_t;
    using typename base_t::dense_vector_t;
    using typename base_t::sparse_matrix_t;
    using typename base_t::sparse_vector_t;
    using typename base_t::value_t;

    scalar(const sym_t &expression, const sym_vector_t &x,
           const sym_vector_t &p, bool densify = false, bool codegen = false);

   protected:
    return_status eval_impl(const Eigen::Ref<const dense_vector_t> &x,
                            value_t &out) override;

    return_status eval_gradient_impl(const Eigen::Ref<const dense_vector_t> &x,
                                     Eigen::Ref<dense_vector_t> out) override;

    return_status eval_gradient_impl(const Eigen::Ref<const dense_vector_t> &x,
                                     sparse_vector_t &out) override;

    void get_gradient_sparsity_impl(sparse_vector_t &out) const override;

    return_status eval_hessian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        const Eigen::Ref<const dense_vector_t> &lambda,
        Eigen::Ref<dense_matrix_t> out) override;

    return_status eval_hessian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        const Eigen::Ref<const dense_vector_t> &lambda,
        sparse_matrix_t &out) override;

    void get_hessian_sparsity_impl(sparse_matrix_t &out) const override;

   private:
    function_t fun_;
    function_t grd_;
    function_t hes_;
};

class vector
    : public bopt::evaluator::differentiable::vector {
   public:
    using base_t = bopt::evaluator::differentiable::vector;

    using typename base_t::dense_matrix_t;
    using typename base_t::dense_vector_t;
    using typename base_t::sparse_matrix_t;
    using typename base_t::sparse_vector_t;
    using typename base_t::value_t;

    vector(const sym_t &expression,
                                    const sym_vector_t &x,
                                    const sym_vector_t &p, bool densify = false,
                                    bool codegen = false);

   protected:
    return_status eval_impl(const Eigen::Ref<const dense_vector_t> &x,
                            Eigen::Ref<dense_vector_t> out) override;

    return_status eval_jacobian_impl(const Eigen::Ref<const dense_vector_t> &x,
                                     Eigen::Ref<dense_matrix_t> out) override;

    return_status eval_jacobian_impl(const Eigen::Ref<const dense_vector_t> &x,
                                     sparse_matrix_t &out) override;

    void get_jacobian_sparsity_impl(sparse_matrix_t &out) const override;

    return_status eval_hessian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        const Eigen::Ref<const dense_vector_t> &lambda,
        Eigen::Ref<dense_matrix_t> out) override;

    return_status eval_hessian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        const Eigen::Ref<const dense_vector_t> &lambda,
        sparse_matrix_t &out) override;

    void get_hessian_sparsity_impl(sparse_matrix_t &out) const override;

   private:
    function_t fun_;
    function_t jac_;
    function_t hes_;
};

}  // namespace differentiable
}  // namespace evaluator
}  // namespace casadi
}  // namespace bopt