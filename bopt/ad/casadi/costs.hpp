#include "bopt/ad/casadi/expression.hpp"
#include "bopt/constraints.hpp"

namespace bopt {
namespace casadi {

template <typename T>
class cost : public bopt::cost<T> {
    typedef bopt::cost<T> Base;

    typedef ::casadi::SX sym_t;
    typedef std::vector<sym_t> sym_vector_t;

    typedef typename Base::value_type value_type;
    typedef typename Base::index_type index_type;
    typedef typename Base::integer_type integer_type;
    typedef typename Base::out_info_t out_info_t;

   public:
    cost() = default;

    cost(const sym_t &expression, const sym_t &x, const sym_vector_t &p)
        : bopt::cost<T>() {
        expression_evaluator_ =
            std::make_unique<expression_evaluator<T>>(expression, x, p);
    }

    static inline std::shared_ptr<cost> create(const sym_t &expression,
                                               const sym_t &x, const sym_vector_t &p) {
        return std::make_shared<cost>(expression, x, p);
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

    integer_type jac_info(out_info_t &info) override {
        return expression_evaluator_->jac_info(info);
    }

    integer_type hes(const value_type **arg, value_type *res) override {
        return expression_evaluator_->hes(arg, res);
    }

    integer_type hes_info(out_info_t &info) override {
        return expression_evaluator_->hes_info(info);
    }

   protected:
   private:
    std::unique_ptr<expression_evaluator<T>> expression_evaluator_;
};

template <typename T>
class linear_cost : public bopt::linear_cost<T> {
   public:
    typedef bopt::linear_cost<T> Base;
    typedef ::casadi::SX sym_t;
    typedef std::vector<sym_t> sym_vector_t;

    typedef typename Base::value_type value_type;
    typedef typename Base::index_type index_type;
    typedef typename Base::integer_type integer_type;
    typedef typename Base::out_info_t out_info_t;

    linear_cost(const sym_t &expression, const sym_t &x, const sym_vector_t &p)
        : bopt::linear_cost<T>() {
        expression_evaluator_ =
            std::make_unique<linear_expression_evaluator<T>>(expression, x, p);
    }

    static inline std::shared_ptr<linear_cost> create(const sym_t &expression,
                                                      const sym_t &x,
                                                      const sym_vector_t &p) {
        return std::make_shared<linear_cost>(expression, x, p);
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

    integer_type jac_info(out_info_t &info) override {
        return expression_evaluator_->jac_info(info);
    }

    integer_type hes(const value_type **arg, value_type *res) override {
        return expression_evaluator_->hes(arg, res);
    }

    integer_type hes_info(out_info_t &info) override {
        return expression_evaluator_->hes_info(info);
    }

    integer_type a(const value_type **arg, value_type *res) override {
        return expression_evaluator_->A(arg, res);
    }

    integer_type a_info(out_info_t &info) override {
        return expression_evaluator_->A_info(info);
    }

    integer_type b(const value_type **arg, value_type *res) override {
        return expression_evaluator_->b(arg, res);
    }

    integer_type b_info(out_info_t &info) override {
        return expression_evaluator_->b_info(info);
    }

   protected:
   private:
    std::unique_ptr<linear_expression_evaluator<T>> expression_evaluator_;
};

template <typename T>
class quadratic_cost : public bopt::quadratic_cost<T> {
   public:
    typedef bopt::quadratic_cost<T> Base;
    typedef ::casadi::SX sym_t;
    typedef std::vector<sym_t> sym_vector_t;

    typedef typename Base::value_type value_type;
    typedef typename Base::index_type index_type;
    typedef typename Base::integer_type integer_type;
    typedef typename Base::out_info_t out_info_t;

    quadratic_cost(const sym_t &expression, const sym_t &x, const sym_vector_t &p)
        : bopt::quadratic_cost<T>() {
        expression_evaluator_ =
            std::make_unique<quadratic_expression_evaluator<T>>(expression, x,
                                                                p);
    }

    static inline std::shared_ptr<quadratic_cost> create(
        const sym_t &expression, const sym_t &x, const sym_vector_t &p) {
        return std::make_shared<quadratic_cost>(expression, x, p);
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

    integer_type jac_info(out_info_t &info) override {
        return expression_evaluator_->jac_info(info);
    }

    integer_type hes(const value_type **arg, value_type *res) override {
        return expression_evaluator_->hes(arg, res);
    }

    integer_type hes_info(out_info_t &info) override {
        return expression_evaluator_->hes_info(info);
    }

    integer_type A(const value_type **arg, value_type *res) override {
        return expression_evaluator_->A(arg, res);
    }

    integer_type A_info(out_info_t &info) override {
        return expression_evaluator_->A_info(info);
    }

    integer_type b(const value_type **arg, value_type *res) override {
        return expression_evaluator_->b(arg, res);
    }

    integer_type b_info(out_info_t &info) override {
        return expression_evaluator_->b_info(info);
    }

    integer_type c(const value_type **arg, value_type *res) override {
        return expression_evaluator_->c(arg, res);
    }

    integer_type c_info(out_info_t &info) override {
        return expression_evaluator_->c_info(info);
    }

   protected:
   private:
    std::unique_ptr<quadratic_expression_evaluator<T>> expression_evaluator_;
};

}  // namespace casadi
}  // namespace bopt