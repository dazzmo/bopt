#pragma once
#include "bopt/evaluator/base.hpp"

namespace bopt {

namespace evaluator {
namespace quadratic {

template <typename ValueType>
class scalar_tpl : public evaluator::scalar_tpl<ValueType> {
   public:
    using base_t = evaluator::scalar_tpl<ValueType>;

    using value_t = typename base_t::value_t;
    using dense_vector_t = typename base_t::dense_vector_t;
    using sparse_vector_t = typename base_t::sparse_vector_t;
    using dense_matrix_t = typename base_t::dense_matrix_t;
    using sparse_matrix_t = typename base_t::sparse_matrix_t;

    using parameter_data_t = typename base_t::parameter_data_t;

    using shared_ptr_t = std::shared_ptr<scalar_tpl<value_t>>;

    scalar_tpl() = default;
    scalar_tpl(const bopt_index &sz_in, const bopt_index &sz_p = 0)
        : base_t(sz_in, sz_p), ptr_(nullptr) {}

    scalar_tpl(const shared_ptr_t &ptr) : base_t(ptr), ptr_(ptr) {}

    /**
     * @brief Evaluates the dense jacobian for the expression \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return return_status
     */
    return_status eval_A(Eigen::Ref<dense_matrix_t> out) {
        return eval_A_impl(out);
    }

    /**
     * @brief Evaluates the sparse coefficient matrix for the quadratic
     * expression
     * \f$c(x)\f$ (i.e. \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return return_status
     */
    return_status eval_A(sparse_matrix_t &out) {
        return eval_A_impl(out);
    }

    /**
     * @brief The number of rows within the coefficient matrix A
     *
     * @return bopt_index
     */
    virtual out_size_t sz_A() const {
        if (ptr_) return ptr_->sz_A();
        return out_size_t(this->sz_out().first, this->sz_in());
    }

    /**
     * @brief Populates a sparse matrix with the sparsity pattern of the
     * coefficient matrix A
     *
     * @param out
     */
    void get_A_sparsity(sparse_matrix_t &out) const {
        if (ptr_)
            ptr_->get_A_sparsity(out);
        else
            get_A_sparsity_impl(out);
    }

    /**
     * @brief Evaluates the dense jacobian for the expression \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return return_status
     */
    return_status eval_b(Eigen::Ref<dense_vector_t> out) {
        return eval_b_impl(out);
    }

    /**
     * @brief Evaluates the dense jacobian for the expression \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return return_status
     */
    return_status eval_b(sparse_vector_t &out) {
        return eval_b_impl(out);
    }

    /**
     * @brief The number of rows within the coefficient vector b
     *
     * @return bopt_index
     */
    virtual out_size_t sz_b() const {
        if (ptr_) return ptr_->sz_b();
        return out_size_t(this->sz_in(), 1);
    }

    /**
     * @brief Populates a sparse vector with the sparsity pattern of the
     * coefficient vector b
     *
     * @param out
     */
    void get_b_sparsity(sparse_vector_t &out) const {
        if (ptr_)
            ptr_->get_b_sparsity(out);
        else
            get_b_sparsity_impl(out);
    }

    return_status eval_c(value_t &out) {
        if (ptr_) return ptr_->eval_c(out);
        return eval_c_impl(out);
    }

   protected:
    virtual return_status eval_A_impl(
        Eigen::Ref<dense_matrix_t> out) {
        return return_status::NotImplemented;
    }

    virtual return_status eval_A_impl(sparse_matrix_t &out) {
        return return_status::NotImplemented;
    }

    virtual void get_A_sparsity_impl(sparse_matrix_t &out) const {}

    virtual return_status eval_b_impl(
        Eigen::Ref<dense_vector_t> out) {
        return return_status::NotImplemented;
    }

    virtual return_status eval_b_impl(sparse_vector_t &out) {
        return return_status::NotImplemented;
    }

    virtual void get_b_sparsity_impl(sparse_vector_t &out) const {}

    virtual return_status eval_c_impl(value_t &out) {
        return return_status::NotImplemented;
    }

   private:
    shared_ptr_t ptr_;
};

typedef scalar_tpl<double> scalar;

}  // namespace quadratic
}  // namespace evaluator
}  // namespace bopt