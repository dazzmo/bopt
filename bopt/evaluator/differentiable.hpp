#pragma once
#include "bopt/evaluator/base.hpp"

namespace bopt {

namespace evaluator {
namespace differentiable {

/**
 * @brief A twice-differentiable expression of the form \f$y = f_p(x)\f$
 *
 * @tparam ValueType
 */
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

    ~scalar_tpl() = default;

    virtual out_size_t sz_gradient() const {
        if (ptr_) return ptr_->sz_gradient();
        return out_size_t(1, this->sz_in());
    }

    return_status eval_gradient(const Eigen::Ref<const dense_vector_t> &x,
                                Eigen::Ref<dense_vector_t> out) {
        if (ptr_) return ptr_->eval_gradient(x, out);
        return eval_gradient_impl(x, out);
    }

    return_status eval_gradient(const Eigen::Ref<const dense_vector_t> &x,
                                sparse_vector_t &out) {
        if (ptr_) return ptr_->eval_gradient(x, out);
        return eval_gradient_impl(x, out);
    }

    void get_gradient_sparsity(sparse_vector_t &out) const {
        if (ptr_)
            ptr_->get_gradient_sparsity(out);
        else
            get_gradient_sparsity_impl(out);
    }

    virtual out_size_t sz_hessian() {
        if (ptr_) return ptr_->sz_hessian();
        return out_size_t(this->sz_in(), this->sz_in());
    }

    return_status eval_hessian(const Eigen::Ref<const dense_vector_t> &x,
                               Eigen::Ref<dense_matrix_t> out) {
        if (ptr_) return ptr_->eval_hessian(x, out);
        return eval_hessian_impl(x, out);
    }

    return_status eval_hessian(const Eigen::Ref<const dense_vector_t> &x,
                               sparse_matrix_t &out) {
        if (ptr_) return ptr_->eval_hessian(x, out);
        return eval_hessian_impl(x, out);
    }

    void get_hessian_sparsity(sparse_matrix_t &out) const {
        if (ptr_)
            ptr_->get_hessian_sparsity(out);
        else
            get_hessian_sparsity_impl(out);
    }

   protected:
    virtual return_status eval_gradient_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_vector_t> out) {
        return return_status::NotImplemented;
    }

    virtual return_status eval_gradient_impl(
        const Eigen::Ref<const dense_vector_t> &x, sparse_vector_t &out) {
        return return_status::NotImplemented;
    }

    virtual void get_gradient_sparsity_impl(sparse_vector_t &out) const {}

    virtual return_status eval_hessian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_matrix_t> out) {
        return return_status::NotImplemented;
    }

    virtual return_status eval_hessian_impl(
        const Eigen::Ref<const dense_vector_t> &x, sparse_matrix_t &out) {
        return return_status::NotImplemented;
    }

    virtual void get_hessian_sparsity_impl(sparse_matrix_t &out) const {}

   private:
    shared_ptr_t ptr_;
};

typedef scalar_tpl<double> scalar;

/**
 * @brief A twice-differentiable expression of the form \f$y = f_p(x)\f$
 *
 * @tparam ValueType
 */
template <typename ValueType>
class vector_tpl : public evaluator::vector_tpl<ValueType> {
   public:
    using base_t = evaluator::vector_tpl<ValueType>;

    using value_t = typename base_t::value_t;
    using dense_vector_t = typename base_t::dense_vector_t;
    using sparse_vector_t = typename base_t::sparse_vector_t;
    using dense_matrix_t = typename base_t::dense_matrix_t;
    using sparse_matrix_t = typename base_t::sparse_matrix_t;

    using parameter_data_t = typename base_t::parameter_data_t;

    using shared_ptr_t = std::shared_ptr<vector_tpl<value_t>>;

    vector_tpl() = default;

    vector_tpl(const bopt_index &sz_in, const bopt_index &sz_out,
               const bopt_index &sz_p = 0)
        : base_t(sz_in, sz_out, sz_p), ptr_(nullptr) {}

    vector_tpl(const shared_ptr_t &ptr) : base_t(ptr), ptr_(ptr) {}

    ~vector_tpl() = default;

    virtual out_size_t sz_jacobian() const {
        if (ptr_) return ptr_->sz_jacobian();
        return out_size_t(this->sz_out().first, this->sz_in());
    }

    return_status eval_jacobian(const Eigen::Ref<const dense_vector_t> &x,
                                Eigen::Ref<dense_matrix_t> out) {
        if (ptr_) return ptr_->eval_jacobian(x, out);
        return eval_jacobian_impl(x, out);
    }

    return_status eval_jacobian(const Eigen::Ref<const dense_vector_t> &x,
                                sparse_matrix_t &out) {
        if (ptr_) return ptr_->eval_jacobian(x, out);
        return eval_jacobian_impl(x, out);
    }

    void get_jacobian_sparsity(sparse_matrix_t &out) const {
        if (ptr_)
            ptr_->get_jacobian_sparsity(out);
        else
            get_jacobian_sparsity_impl(out);
    }

    virtual out_size_t sz_hessian() {
        if (ptr_) return ptr_->sz_hessian();
        return out_size_t(this->sz_in(), this->sz_in());
    }

    return_status eval_hessian(const Eigen::Ref<const dense_vector_t> &x,
                               const Eigen::Ref<const dense_vector_t> &lambda,
                               Eigen::Ref<dense_matrix_t> out) {
        if (ptr_) return ptr_->eval_hessian(x, lambda, out);
        return eval_hessian_impl(x, lambda, out);
    }

    return_status eval_hessian(const Eigen::Ref<const dense_vector_t> &x,
                               const Eigen::Ref<const dense_vector_t> &lambda,
                               sparse_matrix_t &out) {
        if (ptr_) return ptr_->eval_hessian(x, lambda, out);
        return eval_hessian_impl(x, lambda, out);
    }

    void get_hessian_sparsity(sparse_matrix_t &out) const {
        if (ptr_)
            ptr_->get_hessian_sparsity(out);
        else
            get_hessian_sparsity_impl(out);
    }

   protected:
    virtual return_status eval_jacobian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_matrix_t> out) {
        return return_status::NotImplemented;
    }

    virtual return_status eval_jacobian_impl(
        const Eigen::Ref<const dense_vector_t> &x, sparse_matrix_t &out) {
        return return_status::NotImplemented;
    }

    virtual void get_jacobian_sparsity_impl(sparse_matrix_t &out) const {}

    virtual return_status eval_hessian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        const Eigen::Ref<const dense_vector_t> &lambda,
        Eigen::Ref<dense_matrix_t> out) {
        return return_status::NotImplemented;
    }

    virtual return_status eval_hessian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        const Eigen::Ref<const dense_vector_t> &lambda, sparse_matrix_t &out) {
        return return_status::NotImplemented;
    }

    virtual void get_hessian_sparsity_impl(sparse_matrix_t &out) const {}

   private:
    shared_ptr_t ptr_;
};

typedef vector_tpl<double> vector;

}  // namespace differentiable
}  // namespace evaluator
}  // namespace bopt