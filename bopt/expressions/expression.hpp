#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <memory>

#include "bopt/bounds.hpp"
#include "bopt/evaluator.hpp"
#include "bopt/logging.hpp"

namespace bopt {

/**
 * @brief A scalar expression of the form \f$y = f_p(x) \in \mathbb R \f$
 *
 * @tparam ValueType
 */
template <typename ValueType>
class scalar_expression_tpl : public evaluator_tpl<ValueType> {
   public:
    using typename evaluator_tpl<ValueType>::value_t;
    using typename evaluator_tpl<ValueType>::dense_vector_t;

    scalar_expression_tpl() : evaluator_tpl<ValueType>(0, 1) {}

    scalar_expression_tpl(const bopt_index &sz_in, const bopt_index &sz_p = 0)
        : evaluator_tpl<ValueType>(sz_in, 1, sz_p) {}

    ~scalar_expression_tpl() = default;

    /**
     * @brief Evaluates an expression of the form `out` = f(x)
     *
     * @param arg
     * @param ret
     * @return return_status
     */
    evaluator::return_status eval(const Eigen::Ref<const dense_vector_t> &x,
                                  value_t &out) {
        DBGASSERT(this->check_vector(x) && "eval input is invalid");
        return eval_impl(x, out);
    }

    // Buffers for evaluation
    ValueType &buffer() { return buffer_; }

   protected:
    virtual evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x, value_t &out) = 0;

   private:
    ValueType buffer_;
};

/**
 * @brief A scalar expression of the form \f$y = f_p(x) \in \mathbb R^{n_{out}}
 * \f$
 *
 * @tparam ValueType
 */
template <typename ValueType>
class vector_expression_tpl : public evaluator_tpl<ValueType> {
   public:
    using typename evaluator_tpl<ValueType>::value_t;
    using typename evaluator_tpl<ValueType>::dense_vector_t;

    vector_expression_tpl() = default;
    ~vector_expression_tpl() = default;

    vector_expression_tpl(const bopt_index &sz_in, const bopt_index &sz_out,
                          const bopt_index &sz_p = 0)
        : evaluator_tpl<ValueType>(sz_in, sz_out, sz_p),
          buffer_(dense_vector_t::Zero(sz_out)) {}

    evaluator::return_status eval(const Eigen::Ref<const dense_vector_t> &x,
                                  Eigen::Ref<dense_vector_t> out) {
        DBGASSERT(this->check_vector(x) && "eval input is invalid");
        return eval_impl(x, out);
    }

    dense_vector_t &buffer() { return buffer_; }

   protected:
    virtual evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_vector_t> out) = 0;

   private:
    dense_vector_t buffer_;
};

}  // namespace bopt
