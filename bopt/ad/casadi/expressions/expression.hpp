#pragma once

#include <casadi/casadi.hpp>
#include <cassert>
#include <filesystem>


#include "bopt/ad/casadi/utils.hpp"
#include "bopt/expressions/expression.hpp"
#include "bopt/logging.hpp"

namespace bopt {
namespace casadi {

/**
 * @brief Generic expression evaluator, designed to compute the expression,
 * jacobian and hessian of the expression.
 *
 * @tparam T
 */
class scalar_expression : public bopt::scalar_expression_tpl<double> {
   public:
    using base = bopt::scalar_expression_tpl<double>;

    using typename base::dense_matrix_t;
    using typename base::dense_vector_t;
    using typename base::sparse_matrix_t;
    using typename base::sparse_vector_t;
    using typename base::value_t;

    scalar_expression(const sym_t &expression, const sym_vector_t &x,
                      const sym_vector_t &p, bool codegen = false)
        : bopt::scalar_expression_tpl<double>(x.size1(), p.size1()) {
        DBGASSERT(expression.size1() == 1 && "Expression is not scalar!");
        std::vector<sym_vector_t> in = {};
        in.push_back(x);
        in.push_back(p);

        fun_ = create_function("f", in, {expression}, true, codegen);
    }

   protected:
    evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x, value_t &out) override {
        fun_({x.data(), this->parameters().data()}, {&out});
        return evaluator::return_status::Success;
    }

   private:
    function_t fun_;
};

/**
 * @brief Generic expression evaluator, designed to compute the expression,
 * jacobian and hessian of the expression.
 *
 * @tparam T
 */
class vector_expression : public bopt::vector_expression_tpl<double> {
   public:
    using base = bopt::vector_expression_tpl<double>;

    using typename base::dense_matrix_t;
    using typename base::dense_vector_t;
    using typename base::sparse_matrix_t;
    using typename base::sparse_vector_t;
    using typename base::value_t;

    vector_expression(const sym_t &expression, const sym_vector_t &x,
                      const sym_vector_t &p, bool codegen = false)
        : bopt::vector_expression_tpl<double>(x.size1(), expression.size1(),
                                              p.size1()) {
        std::vector<sym_vector_t> in = {};
        in.push_back(x);
        in.push_back(p);

        fun_ = create_function("f", in, {expression}, true, codegen);
    }

   protected:
    evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_vector_t> out) override {
        fun_({x.data(), this->parameters().data()}, {out.data()});
        return evaluator::return_status::Success;
    }

   private:
    function_t fun_;
};

}  // namespace casadi
}  // namespace bopt
