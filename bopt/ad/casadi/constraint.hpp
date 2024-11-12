// Utilities to create constraints for the BOPT interface

#include <dlfcn.h>

#include <casadi/casadi.hpp>
#include <cassert>
#include <filesystem>

#include "bopt/constraint.hpp"
#include "bopt/dlib_handler.hpp"

namespace bopt {
namespace casadi {

typedef casadi::SX sym;

template <typename T>
class Constraint : public bopt::Constraint<T> {

    Constraint(const sym &expression, const sym &x, const sym &p,
               const bound_type::type &type = bound_type::Equality)
        : Base(expression.size1()) {
        // Perform code generation to evaluate these expressions
        auto f_handle = codegen(::casadi::Function(), "name");
        f_eval_ = std::make_unique<bopt::casadi::Evaluator>(f_handle);

        auto j_handle = codegen(::casadi::Function(), "name");
        j_eval_ = std::make_unique<bopt::casadi::Evaluator>(j_handle);

        auto h_handle = codegen(::casadi::Function(), "name");
        h_eval_ = std::make_unique<bopt::casadi::Evaluator>(h_handle);
    }

   public:
    index_type operator()(const value_type **arg, value_type *res) override {
        return f_eval_->(arg, res);
    }

    index_type jac(const value_type **arg, value_type *res) { return 0; }

    index_type hes(const value_type **arg, const value_type **lam,
                   value_type *res) {
        return 0;
    }

    index_type jac_info(index_type *n, index_type *m, index_type *nnz,
                        index_type *indices, index_type *indptr) {
        return 0;
    }

    index_type hes_info(index_type *n, index_type *m, index_type *nnz,
                        index_type *indices, index_type *indptr) {
        return 0;
    }

   private:
    std::unique_ptr<bopt::casadi::Evaluator> f_eval_;
    std::unique_ptr<bopt::casadi::Evaluator> j_eval_;
    std::unique_ptr<bopt::casadi::Evaluator> h_eval_;
};

}  // namespace casadi
}  // namespace bopt
