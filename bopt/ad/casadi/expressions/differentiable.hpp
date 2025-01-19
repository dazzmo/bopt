#pragma once

#include <casadi/casadi.hpp>

#include "bopt/ad/casadi/utils.hpp"
#include "bopt/expressions/differentiable.hpp"
#include "bopt/logging.hpp"

namespace bopt {
namespace casadi {

class differentiable_scalar_expression
    : public bopt::differentiable_scalar_expression_tpl<double> {
   public:
    using base = bopt::differentiable_scalar_expression_tpl<double>;

    using typename base::dense_matrix_t;
    using typename base::dense_vector_t;
    using typename base::sparse_matrix_t;
    using typename base::sparse_vector_t;
    using typename base::value_t;

    differentiable_scalar_expression(const sym_t &expression,
                                     const sym_vector_t &x,
                                     const sym_vector_t &p,
                                     bool densify = false, bool codegen = false)
        : bopt::differentiable_scalar_expression_tpl<double>(x.size1(),
                                                             p.size1()) {
        DBGASSERT(expression.size1() == 1 && "Expression must be scalar!");
        std::vector<sym_vector_t> in = {};
        in.push_back(x);
        in.push_back(p);

        sym_t gradient = sym_t::gradient(expression, x),
              hessian = sym_t::hessian(expression, x);

        fun_ = create_function("f", in, {gradient}, densify, codegen);
        grd_ = create_function("fgrd", in, {gradient}, densify, codegen);
        hes_ = create_function("fhes", in, {hessian}, densify, codegen);
    }

    void sparsity_gradient(sparse_matrix_t &out) const override {
        set_eigen_sparsity(out, grd_.sparsity_out(0));
    }

    void sparsity_hessian(sparse_matrix_t &out) const override {
        set_eigen_sparsity(out, hes_.sparsity_out(0));
    }

   protected:
    evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x, value_t &out) override {
        fun_({x.data(), this->parameters().data()}, {&out});
        return evaluator::return_status::Success;
    }

    evaluator::return_status eval_gradient_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_vector_t> out) override {
        grd_({x.data(), this->parameters().data()}, {out.data()});
        return evaluator::return_status::Success;
    }

    evaluator::return_status eval_gradient_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        sparse_vector_t &out) override {
        grd_({x.data(), this->parameters().data()}, {out.valuePtr()});
        return evaluator::return_status::Success;
    }

    evaluator::return_status eval_hessian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_matrix_t> out) override {
        hes_({x.data(), this->parameters().data()}, {out.data()});
        return evaluator::return_status::Success;
    }

    evaluator::return_status eval_hessian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        sparse_matrix_t &out) override {
        hes_({x.data(), this->parameters().data()}, {out.valuePtr()});
        return evaluator::return_status::Success;
    }

   private:
    function_t fun_;
    function_t grd_;
    function_t hes_;
};

class differentiable_vector_expression
    : public bopt::differentiable_vector_expression_tpl<double> {
   public:
    using base = bopt::differentiable_vector_expression_tpl<double>;

    using typename base::dense_matrix_t;
    using typename base::dense_vector_t;
    using typename base::sparse_matrix_t;
    using typename base::sparse_vector_t;
    using typename base::value_t;

    differentiable_vector_expression(const sym_t &expression,
                                     const sym_vector_t &x,
                                     const sym_vector_t &p,
                                     bool densify = false, bool codegen = false)
        : bopt::differentiable_vector_expression_tpl<double>(
              x.size1(), expression.size1(), p.size1()) {
        std::vector<sym_vector_t> in = {};
        in.push_back(x);
        in.push_back(p);

        // Create lagrange multipliers
        sym_t l = sym_t::sym("l", expression.size1());

        sym_t jacobian = sym_t::jacobian(expression, x),
              hessian =
                  sym_t::tril(sym_t::hessian(sym_t::dot(expression, l), x));

        fun_ = create_function("f", in, {expression}, densify, codegen);
        jac_ = create_function("fjac", in, {jacobian}, densify, codegen);
        in.push_back(l);
        hes_ = create_function("fhes", in, {hessian}, densify, codegen);
    }

    void sparsity_jacobian(sparse_matrix_t &out) const override {
        set_eigen_sparsity(out, jac_.sparsity_out(0));
    }

    void sparsity_hessian(sparse_matrix_t &out) const override {
        set_eigen_sparsity(out, hes_.sparsity_out(0));
    }

   protected:
    evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_vector_t> out) override {
        fun_({x.data(), this->parameters().data()}, {out.data()});
        return evaluator::return_status::Success;
    }

    evaluator::return_status eval_jacobian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_matrix_t> out) override {
        jac_({x.data(), this->parameters().data()}, {out.data()});
        return evaluator::return_status::Success;
    }

    evaluator::return_status eval_jacobian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        sparse_matrix_t &out) override {
        jac_({x.data(), this->parameters().data()}, {out.valuePtr()});
        return evaluator::return_status::Success;
    }

    evaluator::return_status eval_hessian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        const Eigen::Ref<const dense_vector_t> &lam,
        Eigen::Ref<dense_matrix_t> out) override {
        hes_({x.data(), this->parameters().data(), lam.data()}, {out.data()});
        return evaluator::return_status::Success;
    }

    evaluator::return_status eval_hessian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        const Eigen::Ref<const dense_vector_t> &lam,
        sparse_matrix_t &out) override {
        hes_({x.data(), this->parameters().data(), lam.data()},
             {out.valuePtr()});
        return evaluator::return_status::Success;
    }

   private:
    function_t fun_;
    function_t jac_;
    function_t hes_;
};

}  // namespace casadi
}  // namespace bopt
