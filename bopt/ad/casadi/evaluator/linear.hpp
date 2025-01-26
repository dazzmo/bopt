#pragma once

#include "bopt/ad/casadi/utils.hpp"
#include "bopt/evaluator/linear.hpp"
#include "bopt/logging.hpp"

namespace bopt {
namespace casadi {

namespace evaluator {
namespace linear {

using bopt::evaluator::return_status;

class scalar : public bopt::evaluator::linear::scalar {
   public:
    using base_t = bopt::evaluator::linear::scalar;

    using typename base_t::dense_vector_t;
    using typename base_t::value_t;

    scalar(const sym_t &expression, const sym_vector_t &x,
           const sym_vector_t &p, bool densify = true, bool codegen = false);

   protected:
    return_status eval_impl(const Eigen::Ref<const dense_vector_t> &x,
                            value_t &out) override;

    return_status eval_a_impl(Eigen::Ref<dense_vector_t> out) override;
    return_status eval_a_impl(sparse_vector_t &out) override;
    void get_a_sparsity_impl(sparse_vector_t &out) const override;

    return_status eval_b_impl(value_t &out) override;

   private:
    function_t f_;
    function_t a_;
    function_t b_;
};

/**
 * @brief Generic vector expression evaluator.
 *
 * @tparam
 */
class vector : public bopt::evaluator::linear::vector {
   public:
    using base_t = bopt::evaluator::linear::vector;

    using typename base_t::dense_matrix_t;
    using typename base_t::dense_vector_t;
    using typename base_t::sparse_matrix_t;
    using typename base_t::sparse_vector_t;
    using typename base_t::value_t;

    vector(const sym_t &expression, const sym_vector_t &x,
           const sym_vector_t &p, bool densify = true, bool codegen = false);

   protected:
    return_status eval_impl(const Eigen::Ref<const dense_vector_t> &x,
                            Eigen::Ref<dense_vector_t> out) override;

    return_status eval_A_impl(Eigen::Ref<dense_matrix_t> out) override {
        return return_status::NotImplemented;
    }

    return_status eval_A_impl(sparse_matrix_t &out) override;

    void get_A_sparsity_impl(sparse_matrix_t &out) const override;

    return_status eval_b_impl(Eigen::Ref<dense_vector_t> out) override;

    return_status eval_b_impl(sparse_vector_t &out) override;

    void get_b_sparsity_impl(sparse_vector_t &out) const override;

   private:
    function_t f_;
    function_t A_;
    function_t b_;
};

}  // namespace linear
}  // namespace evaluator
}  // namespace casadi
}  // namespace bopt