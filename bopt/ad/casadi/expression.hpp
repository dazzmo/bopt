#pragma once

#include <casadi/casadi.hpp>
#include <cassert>
#include <filesystem>

#include "bopt/ad/casadi/codegen.hpp"
#include "bopt/constraints.hpp"
#include "bopt/logging.hpp"

namespace bopt {
namespace casadi {

/**
 * @brief Generic expression evaluator, designed to compute the expression,
 * jacobian and hessian of the expression.
 *
 * @tparam T
 */
template <typename ValueType>
class expression_tpl : public bopt::expression_tpl<ValueType> {
   public:
    typedef ::casadi::SX sym_t;
    typedef ::casadi::SX sym_vector_t;
    typedef ::casadi::Function function_t;

    using typename evaluator_tpl<ValueType>::value_t;
    using typename evaluator_tpl<ValueType>::dense_vector_t;
    using typename evaluator_tpl<ValueType>::sparse_vector_t;
    using typename evaluator_tpl<ValueType>::dense_matrix_t;
    using typename evaluator_tpl<ValueType>::sparse_matrix_t;

    expression_tpl(const sym_t &expression, const sym_vector_t &x,
                   const sym_vector_t &p, bool codegen = false)
        : bopt::expression_tpl<ValueType>(x.size1(), expression.size1()) {
        std::vector<sym_vector_t> in = {};
        in.push_back(x);
        in.push_back(p);

        // Create lagrange multipliers
        sym_t l = sym_t::sym("l", expression.size1());

        // Create functions
        function_t f("f", in, {expression});
        // Jacobian
        function_t fjac("fjac", in, {sym_t::jacobian(expression, x)});
        // Hessian
        in.push_back(l);
        function_t fhes("fhes", in,
                        {sym_t::hessian(sym_t::dot(expression, l), x)});

        // Perform code generation to evaluate these expressions
        function_t h("h", in, {sym_t::hessian(sym_t::dot(expression, l), x)});

        // Load the code generated functions
        if (codegen) {
            fun_ = std::make_unique<function_t>(bopt::casadi::codegen(f));
            jac_ = std::make_unique<function_t>(bopt::casadi::codegen(fjac));
            hes_ = std::make_unique<function_t>(bopt::casadi::codegen(fhes));
        } else {
            fun_ = std::make_unique<function_t>(f);
            jac_ = std::make_unique<function_t>(fjac);
            hes_ = std::make_unique<function_t>(fhes);
        }
    }

   protected:
    evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_vector_t> out) override {
        (*fun_)({x.data(), this->parameters().data()}, {out.data()});
        return evaluator::return_status::Success;
    }

   private:
    std::unique_ptr<function_t> fun_;
    std::unique_ptr<function_t> jac_;
    std::unique_ptr<function_t> hes_;
};

class constraint_example {
    // Create example
    // constraint create(use this evaluator)

   private:
    std::shared_ptr<expression_tpl<double>> expression_;
};

}  // namespace casadi
}  // namespace bopt
