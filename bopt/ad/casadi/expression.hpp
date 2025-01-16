#pragma once
#include <dlfcn.h>

#include <casadi/casadi.hpp>
#include <cassert>
#include <filesystem>

#include "bopt/ad/casadi/codegen.hpp"
#include "bopt/ad/casadi/evaluator.hpp"
#include "bopt/constraints.hpp"
#include "bopt/dlib_handler.hpp"
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

        // Check if already generated
        f.generate("./f_" +
                   std::to_string(std::hash<std::string>()(f.serialize())));

        // Compile the C-code to a shared library
        std::string compile_command = "gcc -fPIC -shared -O3 f.c -o f.so";
        int flag = system(compile_command.c_str());
        DBGASSERT(flag == 0 && "Compilation failed");

        // Load the code generated functions
        fun_ = std::make_unique<function_t>(::casadi::external("./f"));
    }

    /**
     * @brief Construct a new expression evaluator with known inputs (no
     * jacobian or hessian computed)
     *
     * @param expression
     * @param in
     */
    expression_evaluator(const sym_t &expression, const sym_vector_t &in)
        : Base() {
        function_t f("f", in, {expression});
        // Perform code generation to evaluate these expressions
        auto f_handle = codegen(f);
        f_eval_ = std::make_unique<casadi_evaluator_t>(f_handle, "f");
    }

   public:
    evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_vector_t> out) override {
        (*fun_)({x.data(), p.data()}, {out.data()});
        return evaluator::return_status::Success;
    }

   private:
    std::unique_ptr<function_t> fun_;
    std::unique_ptr<function_t> jac_;
    std::unique_ptr<function_t> hes_;
};

}  // namespace casadi
}  // namespace bopt
