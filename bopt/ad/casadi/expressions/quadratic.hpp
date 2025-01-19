#pragma once

#include <casadi/casadi.hpp>
#include <cassert>
#include <filesystem>

#include "bopt/ad/casadi/utils.hpp"
#include "bopt/expressions/quadratic.hpp"
#include "bopt/logging.hpp"

namespace bopt {
namespace casadi {

/**
 * @brief Generic expression evaluator, designed to compute the expression,
 * jacobian and hessian of the expression.
 *
 * @tparam T
 */
class quadratic_scalar_expression
    : public bopt::quadratic_scalar_expression_tpl<double> {
   public:
    using base = bopt::quadratic_scalar_expression_tpl<double>;

    using typename base::dense_matrix_t;
    using typename base::dense_vector_t;
    using typename base::sparse_matrix_t;
    using typename base::sparse_vector_t;
    using typename base::value_t;

    quadratic_scalar_expression(const sym_t &expression, const sym_vector_t &x,
                                const sym_vector_t &p, bool densify = false,
                                bool codegen = false)
        : bopt::quadratic_scalar_expression_tpl<double>(x.size1(), p.size1()) {
        // Compute coefficients
        sym_t A, b, c;
        sym_t::quadratic_coeff(expression, x, A, b, c, true);

        std::vector<sym_vector_t> in = {};
        in.push_back(p);

        A_ = create_function("A", in, {A}, densify, codegen);
        b_ = create_function("b", in, {b}, densify, codegen);
        c_ = create_function("c", in, {c}, densify, codegen);
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

    evaluator::return_status eval_A_impl(sparse_matrix_t &out) override {
        A_({this->parameters().data()}, {out.valuePtr()});
        return evaluator::return_status::Success;
    }

    evaluator::return_status eval_b_impl(
        Eigen::Ref<dense_vector_t> out) override {
        b_({this->parameters().data()}, {out.data()});
        return evaluator::return_status::Success;
    }

    evaluator::return_status eval_b_impl(sparse_vector_t &out) override {
        b_({this->parameters().data()}, {out.valuePtr()});
        return evaluator::return_status::Success;
    }

    evaluator::return_status eval_c_impl(value_t &out) override {
        c_({this->parameters().data()}, {&out});
        return evaluator::return_status::Success;
    }

   private:
    function_t A_;
    function_t b_;
    function_t c_;
};

}  // namespace casadi
}  // namespace bopt
