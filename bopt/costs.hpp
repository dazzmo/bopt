#pragma once

#include <memory>

#include "bopt/evaluator.hpp"
#include "bopt/logging.hpp"

namespace bopt {

template <typename ValueType>
class cost_tpl : public differentiable_scalar_evaluator_tpl<ValueType> {
   public:
    using base = differentiable_scalar_evaluator_tpl<ValueType>;
    using typename base::dense_matrix_t;
    using typename base::dense_vector_t;
    using typename base::sparse_matrix_t;
    using typename base::sparse_vector_t;
    using typename base::value_t;

    typedef std::string string_t;

    using vector_buffer_t =
        dense_sparse_buffer_tpl<dense_vector_t, sparse_vector_t>;
    using matrix_buffer_t =
        dense_sparse_buffer_tpl<dense_matrix_t, sparse_matrix_t>;

    cost_tpl() = default;
    ~cost_tpl() = default;

    cost_tpl(const bopt_index &sz_in, const bopt_index &sz_p = 0)
        : base(sz_in, sz_p), name_("") {}

    const string_t &name() const { return name_; }
    void set_name(const string_t &name) { name_ = name; }

    value_t &buffer() { return buffer_; }
    vector_buffer_t &buffer_gradient() { return buffer_gradient_; }
    matrix_buffer_t &buffer_hessian() { return buffer_hessian_; }

   protected:
   private:
    string_t name_;
    value_t buffer_;
    vector_buffer_t buffer_gradient_;
    matrix_buffer_t buffer_hessian_;
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
                        public linear_scalar_evaluator_tpl<ValueType> {
   public:
    using typename cost_tpl<ValueType>::value_t;
    using typename cost_tpl<ValueType>::dense_vector_t;
    using typename cost_tpl<ValueType>::sparse_vector_t;
    using typename cost_tpl<ValueType>::dense_matrix_t;
    using typename cost_tpl<ValueType>::sparse_matrix_t;

    using typename cost_tpl<ValueType>::matrix_buffer_t;
    using typename cost_tpl<ValueType>::vector_buffer_t;

    linear_cost_tpl(const bopt_index &sz_in, const bopt_index &sz_p = 0)
        : cost_tpl<ValueType>(sz_in, sz_p),
          linear_scalar_evaluator_tpl<ValueType>(sz_in, sz_p) {}

    const bopt_index &sz_in() const { return cost_tpl<ValueType>::sz_in(); }

    const bopt_index &sz_out() const { return cost_tpl<ValueType>::sz_out(); }

    vector_buffer_t &buffer_a() { return buffer_a_; }

   protected:
    // Overrides
    evaluator::return_status eval_gradient_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_vector_t> out) override {
        return this->eval_a(out);
    }

    evaluator::return_status eval_gradient_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        sparse_vector_t &out) override {
        return this->eval_a(out);
    }

    // evaluator::return_status eval_hessian_impl(
    //     const Eigen::Ref<const dense_vector_t> &x,
    //     Eigen::Ref<dense_matrix_t> out) override {
    //     out.setZero();
    //     return evaluator::return_status::Success;
    // }

    // evaluator::return_status eval_hessian_impl(
    //     const Eigen::Ref<const dense_vector_t> &x,
    //     sparse_matrix_t &out) override {
    //     for (int k = 0; k < out.outerSize(); ++k)
    //         for (typename sparse_matrix_t::InnerIterator it(out, k); it;
    //         ++it)
    //             it.valueRef() = 0.0;
    //     return evaluator::return_status::Success;
    // }

   private:
    vector_buffer_t buffer_a_;
};

/**
 * @brief Quadratic cost of the form \f$ x^T A x + b^T x + c \f$
 *
 * @tparam ValueType
 * @tparam IntegerType
 * @tparam IndexType
 * @tparam MatrixType
 */
template <typename ValueType>
class quadratic_cost_tpl : public cost_tpl<ValueType>,
                           public quadratic_scalar_evaluator_tpl<ValueType> {
   public:
    using typename cost_tpl<ValueType>::value_t;
    using typename cost_tpl<ValueType>::dense_vector_t;
    using typename cost_tpl<ValueType>::sparse_vector_t;
    using typename cost_tpl<ValueType>::dense_matrix_t;
    using typename cost_tpl<ValueType>::sparse_matrix_t;

    using typename cost_tpl<ValueType>::matrix_buffer_t;
    using typename cost_tpl<ValueType>::vector_buffer_t;

    quadratic_cost_tpl(const bopt_index &sz_in, const bopt_index &sz_p = 0)
        : cost_tpl<ValueType>(sz_in, sz_p),
          quadratic_scalar_evaluator_tpl<ValueType>(sz_in, sz_p) {}

    const bopt_index &sz_in() const { return cost_tpl<ValueType>::sz_in(); }
    const bopt_index &sz_out() const { return cost_tpl<ValueType>::sz_out(); }

    matrix_buffer_t &buffer_A() { return buffer_A_; }
    vector_buffer_t &buffer_b() { return buffer_b_; }

   protected:
    // Overrides
    evaluator::return_status eval_gradient_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_vector_t> out) override {
        dense_matrix_t &A = this->buffer_A().dense;
        dense_vector_t &b = this->buffer_b().dense;

        this->eval_A(A);
        this->eval_b(b);

        out = ValueType(2.0) * A * x + b;
        return evaluator::return_status::Success;
    }

    evaluator::return_status eval_gradient_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        sparse_vector_t &out) override {
        sparse_matrix_t &A = this->buffer_A().sparse;
        sparse_vector_t &b = this->buffer_b().sparse;

        this->eval_A(A);
        this->eval_b(b);

        out = ValueType(2.0) * A * x + b;
        return evaluator::return_status::Success;
    }

    void get_hessian_sparsity_impl(sparse_matrix_t &out) const override {
        this->sparsity_A(out);
    }

    // evaluator::return_status eval_hessian_impl(
    //     const Eigen::Ref<const dense_vector_t> &x,
    //     Eigen::Ref<dense_matrix_t> out) override {
    //     return this->eval_A(out);
    // }

    // evaluator::return_status eval_hessian_impl(
    //     const Eigen::Ref<const dense_vector_t> &x,
    //     sparse_matrix_t &out) override {
    //     return this->eval_A(out);
    // }

   private:
    matrix_buffer_t buffer_A_;
    vector_buffer_t buffer_b_;
};

typedef quadratic_cost_tpl<double> quadratic_cost;

template <typename ValueType>
class least_squares_cost_tpl : public quadratic_cost_tpl<ValueType> {
   public:
    using typename quadratic_cost_tpl<ValueType>::value_t;
    using typename quadratic_cost_tpl<ValueType>::dense_vector_t;
    using typename quadratic_cost_tpl<ValueType>::sparse_vector_t;
    using typename quadratic_cost_tpl<ValueType>::dense_matrix_t;
    using typename quadratic_cost_tpl<ValueType>::sparse_matrix_t;

    least_squares_cost_tpl(
        std::shared_ptr<linear_scalar_evaluator_tpl<ValueType>> &expression)
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
    std::shared_ptr<linear_scalar_evaluator_tpl<ValueType>> expression_;
};

}  // namespace bopt
