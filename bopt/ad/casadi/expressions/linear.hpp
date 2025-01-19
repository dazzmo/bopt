#pragma once

#include <casadi/casadi.hpp>
#include <cassert>
#include <filesystem>

#include "bopt/ad/casadi/utils.hpp"
#include "bopt/expressions/linear.hpp"
#include "bopt/logging.hpp"

namespace bopt {
namespace casadi {

/**
 * @brief Generic expression evaluator, designed to compute the expression,
 * jacobian and hessian of the expression.
 *
 * @tparam T
 */
class linear_scalar_expression
    : public bopt::linear_scalar_expression_tpl<double> {
   public:
    using base = bopt::linear_scalar_expression_tpl<double>;

    using typename base::dense_matrix_t;
    using typename base::dense_vector_t;
    using typename base::sparse_matrix_t;
    using typename base::sparse_vector_t;
    using typename base::value_t;

    linear_scalar_expression(const sym_t &expression, const sym_vector_t &x,
                             const sym_vector_t &p, bool densify = false,
                             bool codegen = false)
        : bopt::linear_scalar_expression_tpl<double>(x.size1(), p.size1()) {
        // Compute coefficients
        sym_t a, b;
        sym_t::linear_coeff(expression, x, a, b, true);

        std::vector<sym_vector_t> in = {};
        in.push_back(p);

        a_ = create_function("a", in, {a}, densify, codegen);
        b_ = create_function("b", in, {b}, densify, codegen);
    }

    void sparsity_a(sparse_vector_t &out) const override {
        set_eigen_sparsity(out, a_.sparsity_out(0));
    }

   protected:
    evaluator::return_status eval_a_impl(
        Eigen::Ref<dense_vector_t> out) override {
        a_({this->parameters().data()}, {out.data()});
        return evaluator::return_status::Success;
    }

    evaluator::return_status eval_a_impl(sparse_vector_t &out) override {
        a_({this->parameters().data()}, {out.valuePtr()});
        return evaluator::return_status::Success;
    }

    evaluator::return_status eval_b_impl(value_t &out) override {
        b_({this->parameters().data()}, {&out});
        return evaluator::return_status::Success;
    }

   private:
    function_t a_;
    function_t b_;
};

/**
 * @brief Generic expression evaluator, designed to compute the expression,
 * jacobian and hessian of the expression.
 *
 * @tparam T
 */
class linear_vector_expression
    : public bopt::linear_vector_expression_tpl<double> {
   public:
    using base = bopt::linear_vector_expression_tpl<double>;

    using typename base::dense_matrix_t;
    using typename base::dense_vector_t;
    using typename base::sparse_matrix_t;
    using typename base::sparse_vector_t;
    using typename base::value_t;

    linear_vector_expression(const sym_t &expression, const sym_vector_t &x,
                             const sym_vector_t &p, bool densify = false,
                             bool codegen = false)
        : bopt::linear_vector_expression_tpl<double>(
              x.size1(), expression.size1(), p.size1()) {
        // Compute coefficients
        sym_t A, b;
        sym_t::linear_coeff(expression, x, A, b, true);

        std::vector<sym_vector_t> in = {};
        in.push_back(p);

        A_ = create_function("A", in, {A}, densify, codegen);
        b_ = create_function("b", in, {b}, densify, codegen);
    }

    void sparsity_A(sparse_matrix_t &out) const override {
        set_eigen_sparsity(out, A_.sparsity_out(0));
    }

    void sparsity_b(sparse_vector_t &out) const override {
        set_eigen_sparsity(out, b_.sparsity_out(0));
    }

   protected:
    evaluator::return_status eval_A_impl(
        Eigen::Ref<dense_matrix_t> out) override {
        A_({this->parameters().data()}, {out.data()});
        return evaluator::return_status::Success;
    }

    evaluator::return_status eval_b_impl(
        Eigen::Ref<dense_vector_t> out) override {
        b_({this->parameters().data()}, {out.data()});
        return evaluator::return_status::Success;
    }

   private:
    function_t A_;
    function_t b_;
};

}  // namespace casadi
}  // namespace bopt
