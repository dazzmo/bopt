#include "bopt/ad/casadi/expression.hpp"
#include "bopt/constraints.hpp"

namespace bopt {
namespace casadi {

template <typename T>
class constraint : public bopt::constraint<T> {
    typedef bopt::constraint<T> Base;

    typedef ::casadi::SX sym_t;

    typedef typename Base::value_type value_type;
    typedef typename Base::index_type index_type;
    typedef typename Base::integer_type integer_type;
    typedef typename Base::out_info_t out_info_t;

   public:
    constraint(const sym_t &expression, const sym_t &x, const sym_t &p,
               const bound_type::type &bound_type = bound_type::Unbounded)
        : bopt::constraint<T>(expression.size1(), bound_type) {
        expression_evaluator_ =
            std::make_unique<expression_evaluator<T>>(expression, x, p);
    }

    integer_type operator()(const value_type **arg, value_type *res) override {
        return (*expression_evaluator_)(arg, res);
    }

    integer_type info(out_info_t &info) override {
        return expression_evaluator_->info(info);
    }

    integer_type jac(const value_type **arg, value_type *res) override {
        return expression_evaluator_->jac(arg, res);
    }

    integer_type hes(const value_type **arg, value_type *res) override {
        return expression_evaluator_->hes(arg, res);
    }

    integer_type jac_info(out_info_t &info) override {
        return expression_evaluator_->jac_info(info);
    }

    integer_type hes_info(out_info_t &info) override {
        return expression_evaluator_->hes_info(info);
    }

   private:
    std::unique_ptr<expression_evaluator<T>> expression_evaluator_;
};

}  // namespace casadi
}  // namespace bopt