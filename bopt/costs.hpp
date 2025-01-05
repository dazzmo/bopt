#pragma once

#include <Eigen/Core>
#include <memory>

#include "bopt/bounds.hpp"
#include "bopt/evaluator.hpp"
#include "bopt/logging.hpp"

namespace bopt {

template <typename ValueType, typename GradientVectorType,
          typename HessianMatrixType>
class cost_base_tpl : public virtual expression_scalar_tpl<ValueType> {
   public:
    typedef std::shared_ptr<cost_base_tpl> shared_ptr;
    typedef std::unique_ptr<cost_base_tpl> unique_ptr;

    typedef std::string string_type;

    cost_base_tpl() = default;
    ~cost_base_tpl() = default;

    cost_base_tpl(const index_type &sz_in)
        : evaluator_tpl<ValueType, IntegerType, IndexType>(sz_in, 1),
          name_("") {}

    const string_type &name() const { return name_; }
    void name(const string_type &name) { name_ = name; }

   public:
    string_type name_;
};

/**
 * @brief Linear cost of the form \f$ a^T x + b \f$
 *
 * @tparam ValueType
 * @tparam IntegerType
 * @tparam IndexType
 * @tparam MatrixType
 */
template <typename ValueType, typename VectorType>
class linear_cost_tpl : public cost_base_tpl<ValueType>,
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
template <typename ValueType, typename VectorType, typename MatrixType>
class quadratic_cost_tpl
    : public cost_base_tpl<ValueType, VectorType, MatrixType> {
   public:
    BOPT_ADD_EVALUATOR_FUNCTIONS_NO_ARGUMENTS(A, dense_matrix_t,
                                              sparse_matrix_t)
    BOPT_ADD_EVALUATOR_FUNCTIONS_NO_ARGUMENTS(b, dense_vector_t,
                                              sparse_vector_t)

    virtual evaluator::return_status eval_c(value_type &out) {
        return evaluator::return_status::NotImplemented;
    }

   private:
};

template <typename ValueType>
class least_squares_cost_tpl : public quadratic_cost_tpl<ValueType> {
   public:
    least_squares_cost_tpl(
        std::shared_ptr<linear_expression_tpl<ValueType>> &expression) {}

   protected:
    evaluator::return_status eval_A_impl(
        Eigen::Ref<dense_matrix_t> out) override {
        dense_matrix_t &A = expression_->buffer.A.dense;
        expression_->eval_A(A);
        out = A.transpose() * A;
        return evaluator::return_status::Success;
    }

    evaluator::return_status eval_b_impl(
        Eigen::Ref<dense_vector_t> out) override {
        dense_matrix_t &A = expression_->buffer.A.dense;
        dense_vector_t &b = expression_->buffer.b.dense;
        expression_->eval_A(A);
        expression_->eval_b(b);
        out = 2.0 * A.transpose() * b;
        return evaluator::return_status::Success;
    }

   private:
    std::shared_ptr<linear_expression_tpl<ValueType>> expression_;
};

}  // namespace bopt
