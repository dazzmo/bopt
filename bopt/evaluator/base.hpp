#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>

#include "bopt/common.hpp"
#include "bopt/logging.hpp"
#include "bopt/types.hpp"

namespace bopt {
namespace evaluator {

enum class return_status {
    Success = 0,      // Successful evaluation
    NotImplemented,   // Evaluator not implemented
    InvalidArgument,  // An invalid argument was provided
    InvalidOutput,    // An invalid output was provided
    Failure           // Unknown failure
};

template <typename ValueType>
inline bool check_vector(const Eigen::Ref<const Eigen::VectorX<ValueType>> &x) {
    return !x.hasNaN() || x.allFinite();
}

template <typename T>
using dense_vector_tpl = Eigen::VectorX<T>;
template <typename T>
using sparse_vector_tpl = Eigen::SparseVector<T>;

template <typename T>
using dense_matrix_tpl = Eigen::MatrixX<T>;
template <typename T>
using sparse_matrix_tpl = Eigen::SparseMatrix<T>;

typedef std::pair<bopt_index, bopt_index> out_size_t;

class input_output_traits {
   public:
    input_output_traits() = default;
    ~input_output_traits() = default;

    input_output_traits(const bopt_index &sz_in, const out_size_t &sz_out)
        : sz_in_(sz_in), sz_out_(sz_out) {};

    const bopt_index &sz_in() const { return sz_in_; }
    const out_size_t &sz_out() const { return sz_out_; }

   private:
    // Input size
    bopt_index sz_in_;
    // Output size
    out_size_t sz_out_;
};

template <typename ValueType>
class parameter_data_tpl {
   public:
    using dense_vector_t = dense_vector_tpl<ValueType>;

    parameter_data_tpl() = default;
    ~parameter_data_tpl() = default;

    parameter_data_tpl(const bopt_index &sz)
        : sz_(sz), p_(dense_vector_t::Zero(sz)) {}

    /**
     * @brief Number of parameters
     *
     * @return const bopt_index&
     */
    const bopt_index &size() const { return sz_; }

    const dense_vector_t &values() const { return p_; }
    dense_vector_t &values() { return p_; }

   private:
    // Number of parameters
    bopt_index sz_;
    // Parameter vector
    dense_vector_t p_;
};

/**
 * @brief Evaluator base class. Describes the input-output structure of an
 * evaluator, as well as contains parameter data for the evaluator.
 *
 * @tparam ValueType
 */
template <typename ValueType>
class base_tpl : public input_output_traits {
   public:
    using value_t = ValueType;
    using dense_vector_t = dense_vector_tpl<value_t>;
    using sparse_vector_t = sparse_vector_tpl<value_t>;
    using dense_matrix_t = dense_matrix_tpl<value_t>;
    using sparse_matrix_t = sparse_matrix_tpl<value_t>;

    using evaluator_t = base_tpl<ValueType>;
    using parameter_data_t = parameter_data_tpl<ValueType>;

    using shared_ptr_t = std::shared_ptr<base_tpl<ValueType>>;

    base_tpl(const bopt_index &sz_in, const out_size_t &sz_out,
             const bopt_index &sz_p = 0)
        : input_output_traits(sz_in, sz_out),
          parameter_data_(nullptr),
          ptr_(nullptr) {
        // Create parameter data if parameter count is non-zero
        if (sz_p) parameter_data_ = std::make_shared<parameter_data_t>(sz_p);
    }

    base_tpl(const shared_ptr_t &ptr)
        : input_output_traits(ptr->sz_in(), ptr->sz_out()),
          parameter_data_(nullptr),
          ptr_(ptr) {}

    /**
     * @brief Whether the evaluator has parameters that can be modified.
     *
     * @return true
     * @return false
     */
    bool has_parameters() const {
        if (ptr_) return ptr_->has_parameters();
        return parameter_data_ != nullptr;
    }

    /**
     * @brief Number of parameters within the evaluator
     * 
     * @return bopt_index 
     */
    bopt_index sz_p() const {
        if (ptr_) return ptr_->sz_p();
        if (parameter_data_) return parameter_data_->size();
        return 0;
    }

    /**
     * @brief Parameters used by the evaluator
     *
     * @return const std::shared_ptr<parameter_data_t>&
     */
    const dense_vector_t &parameters() const {
        if (ptr_) return ptr_->parameters();
        return parameter_data_->values();
    }

    dense_vector_t &parameters() {
        if (ptr_) return ptr_->parameters();
        return parameter_data_->values();
    }

    /**
     * @brief shared_ptr to the parameter data used by the evaluator
     *
     * @return const std::shared_ptr<parameter_data_t>&
     */
    const std::shared_ptr<parameter_data_t> &parameter_data() const {
        if (ptr_) return ptr_->parameter_data();
        return parameter_data_;
    }

    void set_parameter_data(
        const std::shared_ptr<parameter_data_t> &parameter_data) {
        if (ptr_) ptr_->set_parameter_data(parameter_data);
        parameter_data_ = parameter_data;
    }

   private:
    static_assert(std::is_arithmetic<ValueType>::value,
                  "ValueType for evaluator::base_tpl "
                  "must be a numerical type.");

    shared_ptr_t ptr_;
    std::shared_ptr<parameter_data_t> parameter_data_;
};

typedef base_tpl<double> base;

/**
 * @brief Evaluator class that computes an output given an input. Also provides
 * the option for a parameterised evaluator of the form \f$ y = f_p(x) \f$
 *
 */
template <typename ValueType>
class scalar_tpl : public base_tpl<ValueType> {
   public:
    using base_t = base_tpl<ValueType>;

    using value_t = typename base_t::value_t;
    using dense_vector_t = typename base_t::dense_vector_t;
    using sparse_vector_t = typename base_t::sparse_vector_t;
    using dense_matrix_t = typename base_t::dense_matrix_t;
    using sparse_matrix_t = typename base_t::sparse_matrix_t;
    using parameter_data_t = typename base_t::parameter_data_t;

    using shared_ptr_t = std::shared_ptr<scalar_tpl<value_t>>;

    scalar_tpl() = default;
    scalar_tpl(const bopt_index &sz_in, const bopt_index &sz_p = 0)
        : base_tpl<ValueType>(sz_in, out_size_t(1, 1), sz_p), ptr_(nullptr) {}

    /**
     * @brief Construct a new scalar evaluator tpl object using an existing
     * evaluator. Useful where evaluators are created through other means such
     * as code generation or symbolic algebra.
     *
     * @param evaluator shared_ptr
     */
    scalar_tpl(const shared_ptr_t &ptr) : base_tpl<ValueType>(ptr), ptr_(ptr) {}

    ~scalar_tpl() = default;

    return_status eval(const Eigen::Ref<const dense_vector_t> &x,
                       value_t &out) {
        if (ptr_) return ptr_->eval(x, out);
        return eval_impl(x, out);
    }

   protected:
    virtual return_status eval_impl(const Eigen::Ref<const dense_vector_t> &x,
                                    value_t &out) {
        return return_status::NotImplemented;
    }

   private:
    shared_ptr_t ptr_;
};

typedef scalar_tpl<double> scalar;

/**
 * @brief Evaluator class that computes an output given an input. Also provides
 * the option for a parameterised evaluator of the form \f$ y = f_p(x) \f$
 *
 */
template <typename ValueType>
class vector_tpl : public base_tpl<ValueType> {
   public:
    using base_t = base_tpl<ValueType>;

    using value_t = typename base_t::value_t;
    using dense_vector_t = typename base_t::dense_vector_t;
    using sparse_vector_t = typename base_t::sparse_vector_t;
    using dense_matrix_t = typename base_t::dense_matrix_t;
    using sparse_matrix_t = typename base_t::sparse_matrix_t;
    using parameter_data_t = typename base_t::parameter_data_t;

    using shared_ptr_t = std::shared_ptr<vector_tpl<value_t>>;

   public:
    vector_tpl() = default;

    vector_tpl(const bopt_index &sz_in, const bopt_index &sz_out,
               const bopt_index &sz_p = 0)
        : base_tpl<ValueType>(sz_in, out_size_t(sz_out, 1), sz_p) {}

    /**
     * @brief Construct a new vector_tpl object using an existing
     * evaluator. Useful where evaluators are created through other means such
     * as code generation or symbolic algebra.
     *
     * @param evaluator shared_ptr
     */
    vector_tpl(const shared_ptr_t &ptr) : base_tpl<ValueType>(ptr), ptr_(ptr) {}

    ~vector_tpl() = default;

    bopt_index rows() const { return this->sz_out().first; }

    return_status eval(const Eigen::Ref<const dense_vector_t> &x,
                       Eigen::Ref<dense_vector_t> out) {
        if (ptr_) return ptr_->eval(x, out);
        return eval_impl(x, out);
    }

    return_status eval(const Eigen::Ref<const dense_vector_t> &x,
                       sparse_vector_t &out) {
        if (ptr_) return ptr_->eval(x, out);
        return eval_impl(x, out);
    }

    void get_sparsity(sparse_vector_t &out) const {
        if (ptr_)
            ptr_->get_sparsity(out);
        else
            get_sparsity_impl(out);
    }

   protected:
    virtual return_status eval_impl(const Eigen::Ref<const dense_vector_t> &x,
                                    Eigen::Ref<dense_vector_t> out) {
        return return_status::NotImplemented;
    }

    virtual return_status eval_impl(const Eigen::Ref<const dense_vector_t> &x,
                                    sparse_vector_t &out) {
        return return_status::NotImplemented;
    }

    virtual void get_sparsity_impl(sparse_vector_t &out) const {}

   private:
    shared_ptr_t ptr_;
};

typedef vector_tpl<double> vector;

/**
 * @brief Evaluator class that computes an output given an input. Also provides
 * the option for a parameterised evaluator of the form \f$ y = f_p(x) \f$
 *
 */
template <typename ValueType>
class matrix_tpl : public base_tpl<ValueType> {
   public:
    using base_t = base_tpl<ValueType>;

    using value_t = typename base_t::value_t;
    using dense_vector_t = typename base_t::dense_vector_t;
    using sparse_vector_t = typename base_t::sparse_vector_t;
    using dense_matrix_t = typename base_t::dense_matrix_t;
    using sparse_matrix_t = typename base_t::sparse_matrix_t;
    using parameter_data_t = typename base_t::parameter_data_t;

    using shared_ptr_t = std::shared_ptr<matrix_tpl<value_t>>;

   public:
    matrix_tpl() = default;

    matrix_tpl(const bopt_index &sz_in, const bopt_index &sz_out,
               const bopt_index &sz_p = 0)
        : base_tpl<ValueType>(sz_in, out_size_t(sz_out, 1), sz_p) {}

    /**
     * @brief Construct a new matrix_tpl object using an existing
     * evaluator. Useful where evaluators are created through other means such
     * as code generation or symbolic algebra.
     *
     * @param evaluator shared_ptr
     */
    matrix_tpl(const shared_ptr_t &ptr) : base_tpl<ValueType>(ptr), ptr_(ptr) {}

    ~matrix_tpl() = default;

    bopt_index rows() const { return this->sz_out().first; }
    bopt_index cols() const { return this->sz_out().second; }

    return_status eval(const Eigen::Ref<const dense_vector_t> &x,
                       Eigen::Ref<dense_matrix_t> out) {
        if (ptr_) return ptr_->eval(x, out);
        return eval_impl(x, out);
    }

    return_status eval(const Eigen::Ref<const dense_vector_t> &x,
                       sparse_matrix_t &out) {
        if (ptr_) return ptr_->eval(x, out);
        return eval_impl(x, out);
    }

    void get_sparsity(sparse_matrix_t &out) const {
        if (ptr_)
            ptr_->get_sparsity(out);
        else
            get_sparsity_impl(out);
    }

   protected:
    virtual return_status eval_impl(const Eigen::Ref<const dense_vector_t> &x,
                                    Eigen::Ref<dense_matrix_t> out) {
        return return_status::NotImplemented;
    }

    virtual return_status eval_impl(const Eigen::Ref<const dense_vector_t> &x,
                                    sparse_matrix_t &out) {
        return return_status::NotImplemented;
    }

    virtual void get_sparsity_impl(sparse_matrix_t &out) const {}

   private:
    shared_ptr_t ptr_;
};

typedef matrix_tpl<double> matrix;

template <typename DenseType, typename SparseType>
struct dense_sparse_buffer_tpl {
    DenseType dense;
    SparseType sparse;
};

}  // namespace evaluator
}  // namespace bopt
