#pragma once
#include <dlfcn.h>

#include <casadi/casadi.hpp>
#include <cassert>
#include <filesystem>

#include "bopt/ad/casadi/evaluator.hpp"
#include "bopt/ad/casadi/codegen.hpp"
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
template <typename T>
class expression_evaluator : public bopt::evaluator<T> {
   public:
    typedef ::casadi::SX sym_t;
    typedef std::vector<sym_t> sym_vector_t;
    typedef ::casadi::Function function_t;

    typedef bopt::evaluator<T> Base;
    typedef typename Base::value_type value_type;
    typedef typename Base::index_type index_type;
    typedef typename Base::integer_type integer_type;
    typedef typename Base::out_info_t out_info_t;

    typedef bopt::evaluator<value_type> evaluator_t;
    typedef bopt::casadi::evaluator<value_type> casadi_evaluator_t;

    expression_evaluator(const sym_t &expression, const sym_t &x,
                         const sym_vector_t &p)
        : Base() {
        sym_vector_t in = {};
        in.push_back(x);
        in.insert(in.end(), p.begin(), p.end());

        function_t f("f", in, {expression});
        // Perform code generation to evaluate these expressions
        auto f_handle = codegen(f);
        f_eval_ = std::make_unique<casadi_evaluator_t>(f_handle, "f");

        function_t j("j", in, {sym_t::jacobian(expression, x)});
        auto j_handle = codegen(j);
        j_eval_ = std::make_unique<casadi_evaluator_t>(j_handle, "j");

        sym_t l = sym_t::sym("l", expression.size1());
        in.push_back(l);
        function_t h("h", in, {sym_t::hessian(sym_t::dot(expression, l), x)});
        auto h_handle = codegen(h);
        h_eval_ = std::make_unique<casadi_evaluator_t>(h_handle, "h");
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
    integer_type operator()(const value_type **arg, value_type *res) override {
        return (*f_eval_)(arg, res);
    }

    integer_type info(out_info_t &info) override { return f_eval_->info(info); }

    integer_type jac(const value_type **arg, value_type *res) {
        return (*j_eval_)(arg, res);
    }

    integer_type hes(const value_type **arg, value_type *res) {
        return (*h_eval_)(arg, res);
    }

    integer_type jac_info(out_info_t &info) { return j_eval_->info(info); }

    integer_type hes_info(out_info_t &info) { return h_eval_->info(info); }

   private:
    std::unique_ptr<casadi_evaluator_t> f_eval_;
    std::unique_ptr<casadi_evaluator_t> j_eval_;
    std::unique_ptr<casadi_evaluator_t> h_eval_;
};

template <typename T>
class linear_expression_evaluator : public expression_evaluator<T> {
   public:
    typedef expression_evaluator<T> Base;

    typedef typename Base::sym_t sym_t;
    typedef typename Base::sym_vector_t sym_vector_t;
    typedef typename Base::function_t function_t;

    typedef typename Base::value_type value_type;
    typedef typename Base::index_type index_type;
    typedef typename Base::integer_type integer_type;
    typedef typename Base::out_info_t out_info_t;

    typedef typename Base::evaluator_t evaluator_t;
    typedef typename Base::casadi_evaluator_t casadi_evaluator_t;

    linear_expression_evaluator(const sym_t &expression, const sym_t &x,
                                const sym_vector_t &p)
        : Base(expression, x, p) {
        // Compute linear expression coefficients
        sym_t A, b;
        try {
            sym_t::linear_coeff(expression, x, A, b, true);
        } catch (std::exception &e) {
            throw std::runtime_error(
                "Expression provided is not linear in specified variable "
                "x!");
        }

        sym_vector_t in = {};
        in.insert(in.end(), p.begin(), p.end());

        function_t f_A("f_A", in, {A});
        function_t f_b("f_b", in, {b});
        // Perform code generation to evaluate these expressions
        auto A_handle = codegen(f_A);
        A_eval_ = std::make_unique<casadi_evaluator_t>(A_handle, "f_A");

        auto b_handle = codegen(f_b);
        b_eval_ = std::make_unique<casadi_evaluator_t>(b_handle, "f_b");
    }

   public:
    integer_type A(const value_type **arg, value_type *res) {
        return (*A_eval_)(arg, res);
    }

    integer_type A_info(out_info_t &info) { return A_eval_->info(info); }

    integer_type b(const value_type **arg, value_type *res) {
        return (*b_eval_)(arg, res);
    }

    integer_type b_info(out_info_t &info) { return b_eval_->info(info); }

   private:
    std::unique_ptr<casadi_evaluator_t> A_eval_;
    std::unique_ptr<casadi_evaluator_t> b_eval_;
};

template <typename T>
class quadratic_expression_evaluator : public expression_evaluator<T> {
   public:
    typedef expression_evaluator<T> Base;

    typedef typename Base::sym_t sym_t;
    typedef typename std::vector<sym_t> sym_vector_t;
    typedef typename Base::function_t function_t;

    typedef typename Base::value_type value_type;
    typedef typename Base::index_type index_type;
    typedef typename Base::integer_type integer_type;
    typedef typename Base::out_info_t out_info_t;

    typedef typename Base::evaluator_t evaluator_t;
    typedef typename Base::casadi_evaluator_t casadi_evaluator_t;

    quadratic_expression_evaluator(const sym_t &expression, const sym_t &x,
                                   const sym_vector_t &p)
        : Base(expression, x, p) {
        // Compute linear expression coefficients
        sym_t A, b, c;
        try {
            sym_t::quadratic_coeff(expression, x, A, b, c, true);
        } catch (std::exception &e) {
            throw std::runtime_error(
                "Expression provided is not quadratic in specified variable "
                "x!");
        }
        sym_t::quadratic_coeff(expression, x, A, b, c, true);

        sym_vector_t in = {};
        in.insert(in.end(), p.begin(), p.end());

        function_t f_A("f_A", in, {A});
        function_t f_b("f_b", in, {b});
        function_t f_c("f_c", in, {c});
        // Perform code generation to evaluate these expressions
        auto A_handle = codegen(f_A);
        A_eval_ = std::make_unique<casadi_evaluator_t>(A_handle, "f_A");

        auto b_handle = codegen(f_b);
        b_eval_ = std::make_unique<casadi_evaluator_t>(b_handle, "f_b");

        auto c_handle = codegen(f_c);
        c_eval_ = std::make_unique<casadi_evaluator_t>(c_handle, "f_c");
    }

   public:
    integer_type A(const value_type **arg, value_type *res) {
        LOG(INFO) << "expression A start";
        (*A_eval_)(arg, res);
        LOG(INFO) << "expression A done";
        return integer_type(0);
    }

    integer_type A_info(out_info_t &info) {
        A_eval_->info(info);
        return integer_type(0);
    }

    integer_type b(const value_type **arg, value_type *res) {
        (*b_eval_)(arg, res);
        return integer_type(0);
    }

    integer_type b_info(out_info_t &info) {
        b_eval_->info(info);
        return integer_type(0);
    }

    integer_type c(const value_type **arg, value_type *res) {
        (*c_eval_)(arg, res);
        return integer_type(0);
    }

    integer_type c_info(out_info_t &info) {
        c_eval_->info(info);
        return integer_type(0);
    }

   private:
    std::unique_ptr<casadi_evaluator_t> A_eval_;
    std::unique_ptr<casadi_evaluator_t> b_eval_;
    std::unique_ptr<casadi_evaluator_t> c_eval_;
};

}  // namespace casadi
}  // namespace bopt
