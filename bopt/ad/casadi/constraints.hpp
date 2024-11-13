// Utilities to create constraints for the BOPT interface

#include <dlfcn.h>

#include <casadi/casadi.hpp>
#include <cassert>
#include <filesystem>

#include "bopt/ad/casadi/evaluator.hpp"
#include "bopt/constraints.hpp"
#include "bopt/dlib_handler.hpp"
#include "bopt/logging.hpp"

namespace bopt {
namespace casadi {

template <typename T>
class constraint : public bopt::constraint<T> {
   public:
    typedef ::casadi::SX sym_t;
    typedef ::casadi::Function function_t;

    typedef bopt::constraint<T> Base;
    typedef typename Base::value_type value_type;
    typedef typename Base::index_type index_type;
    typedef typename Base::integer_type integer_type;
    typedef typename Base::out_info_t out_info_t;

    typedef bopt::casadi::evaluator<value_type> casadi_evaluator_t;

    constraint(const sym_t &expression, const sym_t &x, const sym_t &p,
               const bound_type::type &type = bound_type::Equality)
        : Base(expression.size1()) {
        function_t f("f", {x, p}, {expression});
        // Perform code generation to evaluate these expressions
        auto f_handle = codegen(f);
        f_eval_ = std::make_unique<casadi_evaluator_t>(f_handle, "f");

        function_t j("j", {x, p}, {sym_t::jacobian(expression, x)});
        auto j_handle = codegen(j);
        j_eval_ = std::make_unique<casadi_evaluator_t>(j_handle, "j");

        sym_t l = sym_t::sym("l", expression.size1());
        function_t h("h", {x, p, l},
                     {sym_t::hessian(sym_t::dot(expression, l), x)});
        auto h_handle = codegen(h);
        h_eval_ = std::make_unique<casadi_evaluator_t>(h_handle, "h");
    }

   public:
    integer_type operator()(const value_type **arg, value_type *res) override {
        return (*f_eval_)(arg, res);
    }

    integer_type jac(const value_type **arg, value_type *res) override {
        (*j_eval_)(arg, res);
        return integer_type(0);
    }

    integer_type hes(const value_type **arg, value_type *res) override {
        (*h_eval_)(arg, res);
        return integer_type(0);
    }

    integer_type jac_info(out_info_t &info) override {
        j_eval_->info(info);
        return integer_type(0);
    }

    integer_type hes_info(out_info_t &info) override {
        h_eval_->info(info);
        return integer_type(0);
    }

   private:
    std::unique_ptr<casadi_evaluator_t> f_eval_;
    std::unique_ptr<casadi_evaluator_t> j_eval_;
    std::unique_ptr<casadi_evaluator_t> h_eval_;
};

}  // namespace casadi
}  // namespace bopt
