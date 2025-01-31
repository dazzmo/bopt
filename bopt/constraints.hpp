#pragma once

#include "bopt/bounds.hpp"
#include "bopt/evaluator.hpp"
#include "bopt/logging.hpp"

namespace bopt {

template <typename ValueType>
class constraint_tpl : public evaluator::differentiable::vector_tpl<ValueType> {
   public:
    using base_t = evaluator::differentiable::vector_tpl<ValueType>;
    using typename base_t::dense_matrix_t;
    using typename base_t::dense_vector_t;
    using typename base_t::sparse_matrix_t;
    using typename base_t::sparse_vector_t;
    using typename base_t::value_t;

    using matrix_buffer_t =
        evaluator::dense_sparse_buffer_tpl<dense_matrix_t, sparse_matrix_t>;

    typedef std::string string_t;

    constraint_tpl() = default;
    ~constraint_tpl() = default;

    constraint_tpl(const bopt_index &sz_in, const bopt_index &sz_out,
                   const bopt_index &sz_p, const bounds::type &type)
        : base_t(sz_in, sz_out, sz_p),
          name_(""),
          lower_bound_(sz_out),
          upper_bound_(sz_out) {
        set_bounds(type);

        // Set up dense buffers
        buffer_ = dense_vector_t::Zero(this->rows());
        buffer_jacobian_.dense = dense_matrix_t::Zero(
            this->sz_jacobian().first, this->sz_jacobian().second);
        buffer_hessian_.dense = dense_matrix_t::Zero(this->sz_hessian().first,
                                                     this->sz_hessian().second);
    }

    /**
     * @brief Construct a new constraint tpl object through an expression
     *
     * @param expression
     * @param type
     */
    constraint_tpl(const typename base_t::shared_ptr_t &ptr,
                   const bounds::type &type)
        : base_t(ptr),
          name_(""),
          lower_bound_(ptr->sz_out()),
          upper_bound_(ptr->sz_out()) {
        set_bounds(type);

        // Set up dense buffers
        buffer_ = dense_vector_t::Zero(this->rows());
        buffer_jacobian_.dense = dense_matrix_t::Zero(
            this->sz_jacobian().first, this->sz_jacobian().second);
        buffer_hessian_.dense = dense_matrix_t::Zero(this->sz_hessian().first,
                                                     this->sz_hessian().second);
    }

    /**
     * @brief Name of the constraint
     *
     * @return const string_t&
     */
    const string_t &name() const { return name_; }
    void set_name(const string_t &name) { name_ = name; }

    /**
     * @brief Set the bounds of the constraint by type
     *
     * @param type
     */
    void set_bounds(const bounds::type &type) {
        bounds::set_bound_limits<ValueType>(type, lower_bound_, upper_bound_);
    }

    const dense_vector_t &lower_bound() const { return lower_bound_; }
    void set_lower_bound(const Eigen::Ref<const dense_vector_t> &lower_bound) {
        // DBGASSERT(lower_bound.size() == this->rows() && "Incorrect bound
        // size");
        lower_bound_ = lower_bound;
    }

    const dense_vector_t &upper_bound() const { return upper_bound_; }
    void set_upper_bound(const Eigen::Ref<const dense_vector_t> &upper_bound) {
        // DBGASSERT(upper_bound.size() == this->rows() && "Incorrect bound
        // size");
        upper_bound_ = upper_bound;
    }

    /**
     * @brief Determines whether the constraint (evaluated with the @ref
     * constraint_tpl::buffer()) is within the specified bounds.
     *
     * @param epsilon
     * @return true
     * @return false
     */
    inline bool is_satisfied(
        const ValueType &epsilon =
            std::numeric_limits<ValueType>::epsilon()) const {
        return (this->buffer() - lower_bound_).minCoeff() >= epsilon &&
               (upper_bound_ - this->buffer()).minCoeff() >= epsilon;
    }

    // todo - margins

    dense_vector_t &buffer() { return buffer_; }
    matrix_buffer_t &buffer_jacobian() { return buffer_jacobian_; }
    matrix_buffer_t &buffer_hessian() { return buffer_hessian_; }

   protected:
   private:
    string_t name_;
    dense_vector_t lower_bound_;
    dense_vector_t upper_bound_;

    dense_vector_t buffer_;
    matrix_buffer_t buffer_jacobian_;
    matrix_buffer_t buffer_hessian_;
};

typedef constraint_tpl<double> constraint;

template <typename ValueType>
std::ostream &operator<<(std::ostream &out,
                         constraint_tpl<ValueType> const &constraint) {
    out << "constraint name: " << constraint.name() << '\n';
    // out << "buffer: " << constraint.buffer().transpose() << '\n';
    out << "lower bound: " << constraint.lower_bound().transpose() << '\n';
    out << "upper bound: " << constraint.upper_bound().transpose() << '\n';
    return out;
}

template <typename ValueType>
class linear_constraint_tpl : public constraint_tpl<ValueType>,
                              public evaluator::linear::vector_tpl<ValueType> {
   public:
    using base_t = constraint_tpl<ValueType>;

    using typename base_t::dense_matrix_t;
    using typename base_t::dense_vector_t;
    using typename base_t::sparse_matrix_t;
    using typename base_t::sparse_vector_t;
    using typename base_t::value_t;

    using typename base_t::matrix_buffer_t;

    using vector_buffer_t =
        evaluator::dense_sparse_buffer_tpl<dense_vector_t, sparse_vector_t>;

    linear_constraint_tpl(const bopt_index &sz_in, const bopt_index &sz_out,
                          const bopt_index &sz_p, const bounds::type &type)
        : constraint_tpl<ValueType>(sz_in, sz_out, sz_p, type),
          evaluator::linear::vector_tpl<ValueType>(sz_in, sz_out, sz_p) {
        // Initialise buffers
        this->buffer_A().dense =
            dense_matrix_t::Zero(this->sz_A().first, this->sz_A().second);
        this->buffer_b().dense = dense_vector_t::Zero(this->sz_b().first);
    }

    /**
     * @brief Construct a new constraint tpl object via a linear expression
     *
     * @param expression
     * @param type
     */
    linear_constraint_tpl(
        const typename evaluator::linear::vector_tpl<ValueType>::shared_ptr_t
            &ptr,
        const bounds::type &type)
        : base_t(ptr->sz_in(), ptr->rows(), ptr->sz_p(), type),
          evaluator::linear::vector_tpl<ValueType>(ptr) {}

    const bopt_index &sz_in() const { return base_t::sz_in(); }

    bopt_index sz_p() const { return base_t::sz_p(); }

    const bopt_index &sz_out() const { return base_t::sz_out(); }

    matrix_buffer_t &buffer_A() { return buffer_A_; }
    vector_buffer_t &buffer_b() { return buffer_b_; }

   protected:
    evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_vector_t> out) override {
        return evaluator::linear::vector_tpl<ValueType>::eval(x, out);
    }

    // Overrides given the structure
    evaluator::return_status eval_jacobian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_matrix_t> out) override {
        return this->eval_A(out);
    }

    evaluator::return_status eval_jacobian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        sparse_matrix_t &out) override {
        return this->eval_A(out);
    }

    evaluator::return_status eval_hessian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        const Eigen::Ref<const dense_vector_t> &lambda,
        Eigen::Ref<dense_matrix_t> out) override {
        out.setZero();
        return evaluator::return_status::Success;
    }

    evaluator::return_status eval_hessian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        const Eigen::Ref<const dense_vector_t> &lambda,
        sparse_matrix_t &out) override {
        for (int k = 0; k < out.outerSize(); ++k)
            for (Eigen::SparseMatrix<double>::InnerIterator it(out, k); it;
                 ++it)
                it.valueRef() = 0.0;
        return evaluator::return_status::Success;
    }

   private:
    matrix_buffer_t buffer_A_;
    vector_buffer_t buffer_b_;
};

typedef linear_constraint_tpl<double> linear_constraint;

/**
 * @brief Converts the constraint \f$ lb \le x \le ub \f$ to the stacked
 * inequality constraint \f$ [x - ub, -x + lb] \le 0 \f$
 *
 */
template <typename ValueType>
class bounding_box_constraint_tpl : public constraint_tpl<ValueType> {
   public:
    using typename constraint_tpl<ValueType>::value_t;
    using typename constraint_tpl<ValueType>::dense_vector_t;
    using typename constraint_tpl<ValueType>::sparse_vector_t;
    using typename constraint_tpl<ValueType>::dense_matrix_t;
    using typename constraint_tpl<ValueType>::sparse_matrix_t;

    bounding_box_constraint_tpl() = default;

    bounding_box_constraint_tpl(const bopt_index &sz_in, const bopt_index &sz_p,
                                const bounds::type &type)
        : constraint_tpl<ValueType>(sz_in, 2 * sz_in, sz_p),
          x_lower_bound_(dense_vector_t::Zero(sz_in)),
          x_upper_bound_(dense_vector_t::Zero(sz_in)),
          converted_(false) {}

    bounding_box_constraint_tpl(
        const bopt_index &sz_in, const bopt_index &sz_p,
        const Eigen::Ref<const dense_vector_t> &lower_bound,
        const Eigen::Ref<const dense_vector_t> &upper_bound)
        : constraint_tpl<ValueType>(sz_in, 2 * sz_in, sz_p),
          x_lower_bound_(lower_bound),
          x_upper_bound_(upper_bound),
          converted_(false) {
        // DBGASSERT(lower_bound.size() == sz_in && upper_bound.size() == sz_in
        // &&
        //   "Bound vector size mismatch");
    }

   protected:
    // evaluator::return_status eval_impl(
    //     const Eigen::Ref<const dense_vector_t> &x,
    //     Eigen::Ref<dense_vector_t> out) override {
    //     if (!converted_) {
    //         x_lower_bound_ = this->lower_bound();
    //         x_upper_bound_ = this->upper_bound();
    //         this->set_bounds(bounds::type::Negative);
    //         converted_ = true;
    //     }

    //     for (bopt_index i = 0; i < this->sz_out(); ++i) {
    //         out[i] = x[i] - x_upper_bound_[i];
    //         out[this->sz_out() + i] = -x[i] + x_lower_bound_[i];
    //     }

    //     return evaluator::return_status::Success;
    // }

    // evaluator::return_status eval_jacobian_impl(
    //     const Eigen::Ref<const dense_vector_t> &x,
    //     Eigen::Ref<dense_vector_t> out) {
    //     out.topRows(this->sz_in()).diagonal().array().setConstant(1.0);
    //     out.bottomRows(this->sz_in()).diagonal().array().setConstant(-1.0);
    //     return evaluator::return_status::Success;
    // }

   private:
    bool converted_;
    dense_vector_t x_lower_bound_;
    dense_vector_t x_upper_bound_;
};

}  // namespace bopt
