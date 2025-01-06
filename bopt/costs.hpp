#pragma once

#include <memory>

#include "bopt/expression.hpp"
#include "bopt/logging.hpp"

namespace bopt {

template <typename ValueType>
class cost_tpl : public virtual expression_scalar_tpl<ValueType> {
   public:
    using typename expression_scalar_tpl<ValueType>::value_t;
    using typename expression_scalar_tpl<ValueType>::dense_vector_t;
    using typename expression_scalar_tpl<ValueType>::sparse_vector_t;
    using typename expression_scalar_tpl<ValueType>::dense_matrix_t;
    using typename expression_scalar_tpl<ValueType>::sparse_matrix_t;

    typedef std::string string_t;

    typedef std::shared_ptr<cost_tpl> shared_ptr;
    typedef std::unique_ptr<cost_tpl> unique_ptr;

    cost_tpl() = default;
    ~cost_tpl() = default;

    cost_tpl(const bopt_index &sz_in)
        : expression_scalar_tpl<ValueType>(sz_in), name_("") {}

    const string_t &name() const { return name_; }
    void name(const string_t &name) { name_ = name; }

   public:
    string_t name_;
};

/**
 * @brief Linear cost of the form \f$ a^T x + b \f$
 *
 * @tparam ValueType
 * @tparam IntegerType
 * @tparam IndexType
 * @tparam MatrixType
 */
template <typename ValueType>
class linear_cost_tpl : public cost_tpl<ValueType>,
                        public linear_scalar_expression_tpl<ValueType> {
   public:
   private:
};

/**
 * @brief Linear cost of the form \f$ x^T A x + b^T x + c \f$
 *
 * @tparam ValueType
 * @tparam IntegerType
 * @tparam IndexType
 * @tparam MatrixType
 */
template <typename ValueType>
class quadratic_cost_tpl : public cost_tpl<ValueType>,
                           public quadratic_scalar_expression_tpl<ValueType> {
   public:
    using typename cost_tpl<ValueType>::value_t;
    using typename cost_tpl<ValueType>::dense_vector_t;
    using typename cost_tpl<ValueType>::sparse_vector_t;
    using typename cost_tpl<ValueType>::dense_matrix_t;
    using typename cost_tpl<ValueType>::sparse_matrix_t;

   private:
};

template <typename ValueType>
class least_squares_cost_tpl : public quadratic_cost_tpl<ValueType> {
   public:
    using typename quadratic_cost_tpl<ValueType>::value_t;
    using typename quadratic_cost_tpl<ValueType>::dense_vector_t;
    using typename quadratic_cost_tpl<ValueType>::sparse_vector_t;
    using typename quadratic_cost_tpl<ValueType>::dense_matrix_t;
    using typename quadratic_cost_tpl<ValueType>::sparse_matrix_t;

    least_squares_cost_tpl(
        std::shared_ptr<linear_scalar_expression_tpl<ValueType>> &expression)
        : expression_(nullptr) {
        expression_ = expression;
    }

    least_squares_cost_tpl(std::shared_ptr<linear_cost_tpl<ValueType>> &cost)
        : expression_(nullptr) {
        expression_ = cost;
    }

   protected:
    evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x, double &out) {
        expression_->eval(x, out);
        out = out * out;
        return evaluator::return_status::Success;
    }

   private:
    std::shared_ptr<linear_scalar_expression_tpl<ValueType>> expression_;
};

}  // namespace bopt
