#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <memory>

#include "bopt/bounds.hpp"
#include "bopt/evaluator.hpp"
#include "bopt/expression.hpp"
#include "bopt/logging.hpp"

namespace bopt {

template <typename ValueType>
class constraint_base_tpl : public expression_tpl<ValueType> {
   public:
    constraint_base_tpl() = default;
    ~constraint_base_tpl() = default;

    constraint_base_tpl(const bopt_index &sz_in, const bopt_index &sz_out,
                        const bound_type &type = bound_type::Unbounded)
        : evaluator_tpl<ValueType>(sz_in, sz_out),
          name_(""),
          lower_bound(sz_out),
          upper_bound(sz_out) {
        set_bounds_to_type(type);
    }

    const string_type &name() const { return name_; }
    void name(const string_type &name) { name_ = name; }

    void set_bounds_to_type(const bounds_type &type) {
        set_bound_limits(type, lower_bound, upper_bound);
    }

    dense_vector_t lower_bound;
    dense_vector_t upper_bound;

   private:
    string_type name_;
};

// todo - print out everything about everything

template <typename ValueType>
class linear_constraint_tpl : public constraint_base_tpl<ValueType>,
                              public linear_expression_tpl<ValueType> {
   public:
   protected:
   private:
};

template <typename ValueType, typename MatrixType>
class bounding_box_constraint_tpl : public constraint_base_tpl<ValueType> {
   public:
    bounding_box_constraint_tpl() = default;

    bounding_box_constraint_tpl(const index_type &sz_in, const vector_type &lb,
                                const vector_type &ub)
        : constraint_base_tpl<ValueType, IntegerType, IndexType, MatrixType>(
              sz_in, 2 * sz_in),
          converted_(false) {
        DBGASSERT(lb.size() == ub.size() && lb.size() == sz &&
                  "Bound vector size mismatch");
    }

    /**
     * @brief Converts the constraint \f$ lb \le x \le ub \f$ to the stacked
     * inequality constraint \f$ [x - ub, -x + lb] \le 0 \f$
     *
     */
    void convert_to_constraint() {
        arg_lower_bound_ = lower_bound;
        arg_upper_bound_ = upper_bound;
        this->set_bounds_to_type(bound_type::Negative);
        converted_ = true;
    }

    integer_type eval(const value_type *arg, value_type *ret) override {
        DBGASSERT(converted_ &&
                  "Bounding box constraint not converted to generic constraint "
                  "format");

        for (index_type i = 0; i < this->sz_out(); ++i) {
            ret[i] = arg[i] - arg_upper_bound_[i];
            ret[this->sz_out() + i] = -arg[i] + arg_lower_bound_[i];
        }

        return integer_type(0);
    }

    integer_type eval_jac(const vector_type &arg, MatrixType &res) {
        // res.diagonal().array().setConstant(1.0);
        // res.diagonal().array().setConstant(1.0);
    }

   private:
    bool converted_;
    vector_type arg_lower_bound_;
    vector_type arg_upper_bound_;
};

}  // namespace bopt
