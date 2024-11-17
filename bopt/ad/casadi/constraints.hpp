#include "bopt/ad/casadi/expression.hpp"
#include "bopt/constraints.hpp"

namespace bopt {
namespace casadi {

template <typename T>
class constraint : public bopt::constraint<T> {
    typedef bopt::constraint<T> Base;

    typedef ::casadi::SX sym_t;
    typedef std::vector<sym_t> sym_vector_t;

    typedef typename Base::value_type value_type;
    typedef typename Base::index_type index_type;
    typedef typename Base::integer_type integer_type;
    typedef typename Base::out_info_t out_info_t;

   public:
    constraint() = default;

    constraint(const sym_t &expression, const sym_t &x, const sym_vector_t &p,
               const bound_type::type &bound_type = bound_type::Unbounded)
        : bopt::constraint<T>(expression.size1(), bound_type) {
        expression_evaluator_ =
            std::make_unique<expression_evaluator<T>>(expression, x, p);
    }

    static inline std::shared_ptr<constraint> create(
        const sym_t &expression, const sym_t &x, const sym_vector_t &p,
        const bound_type::type &bound_type = bound_type::Unbounded) {
        return std::make_shared<constraint>(expression, x, p, bound_type);
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
class linear_constraint : public bopt::linear_constraint<T> {
   public:
    typedef bopt::linear_constraint<T> Base;
    typedef ::casadi::SX sym_t;
    typedef std::vector<sym_t> sym_vector_t;

    typedef typename Base::value_type value_type;
    typedef typename Base::index_type index_type;
    typedef typename Base::integer_type integer_type;
    typedef typename Base::out_info_t out_info_t;

    linear_constraint(
        const sym_t &expression, const sym_t &x, const sym_vector_t &p,
        const bound_type::type &bound_type = bound_type::Unbounded)
        : bopt::linear_constraint<T>(expression.size1(), bound_type) {
        expression_evaluator_ =
            std::make_unique<linear_expression_evaluator<T>>(expression, x, p);
    }

    static inline std::shared_ptr<linear_constraint> create(
        const sym_t &expression, const sym_t &x, const sym_vector_t &p,
        const bound_type::type &bound_type = bound_type::Unbounded) {
        return std::make_shared<linear_constraint>(expression, x, p,
                                                   bound_type);
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

   protected:
   private:
    std::unique_ptr<linear_expression_evaluator<T>> expression_evaluator_;
};

// template <typename T>
// class bounding_box_constraint : public bopt::bounding_box_constraint<T> {
//    public:
//     typedef bopt::linear_constraint<T> Base;
//     typedef ::casadi::SX sym_t;
//     typedef std::vector<sym_t> sym_vector_t;

//     typedef typename Base::value_type value_type;
//     typedef typename Base::index_type index_type;
//     typedef typename Base::integer_type integer_type;
//     typedef typename Base::out_info_t out_info_t;

//     bounding_box_constraint(const sym_t &lb, const sym_t &ub,
//                             const sym_vector_t &p)
//         : bopt::bounding_box_constraint<T>(expression.size1(),
//                                            bound_type::Unbounded) {
//         lb_expression_evaluator_ =
//             std::make_unique<expression_evaluator<T>>(lb, p);

//         ub_expression_evaluator_ =
//             std::make_unique<expression_evaluator<T>>(ub, p);
//     }

//     static inline std::shared_ptr<bounding_box_constraint> create(
//         const sym_t &lb, const sym_t &ub, const sym_vector_t &p) {
//         return std::make_shared<bounding_box_constraint>(lb, ub, p);
//     }

//     integer_type update_bounds(const value_type **arg) override {
//         std::vector<value_type> lb();
//         std::vector<value_type> ub();

//         (*lb_expression_evaluator_)(arg, bounds)

//         return integer_type(0);
//     }

//    protected:
//    private:
//     std::unique_ptr<expression_evaluator<T>> lb_expression_evaluator_;
//     std::unique_ptr<expression_evaluator<T>> ub_expression_evaluator_;
// };

}  // namespace casadi
}  // namespace bopt