#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>

#include "bopt/common.hpp"
#include "bopt/logging.hpp"
#include "bopt/types.hpp"

namespace bopt {

struct evaluator {
    enum class return_status {
        Success = 0,      // Successful evaluation
        NotImplemented,   // Evaluator not implemented
        InvalidArgument,  // An invalid argument was provided
        InvalidOutput,    // An invalid output was provided
        Failure           // Unknown failure
    };
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
class evaluator_base_tpl : public input_output_traits {
   public:
    using value_t = ValueType;
    using dense_vector_t = dense_vector_tpl<value_t>;
    using sparse_vector_t = sparse_vector_tpl<value_t>;
    using dense_matrix_t = dense_matrix_tpl<value_t>;
    using sparse_matrix_t = sparse_matrix_tpl<value_t>;

    using evaluator_t = evaluator_base_tpl<ValueType>;
    using parameter_data_t = parameter_data_tpl<ValueType>;

    evaluator_base_tpl(const bopt_index &sz_in, const out_size_t &sz_out,
                       const bopt_index &sz_p = 0)
        : input_output_traits(sz_in, sz_out),
          parameter_data_(std::make_shared<parameter_data_t>(sz_p)) {}

    evaluator_base_tpl(const std::shared_ptr<evaluator_t> &ptr)
        : input_output_traits(ptr->sz_in(), ptr->sz_out()),
          parameter_data_(ptr->parameter_data()) {}

    /**
     * @brief Parameters used by the evaluator
     *
     * @return const std::shared_ptr<parameter_data_t>&
     */
    const dense_vector_t &parameters() const {
        return parameter_data_->values();
    }
    dense_vector_t &parameters() { return parameter_data_->values(); }

    const std::shared_ptr<parameter_data_t> &parameter_data() const {
        return parameter_data_;
    }

    /**
     * @brief Set data for the parameters for the evaluator to the set provided
     * by p
     *
     * @param p
     */
    void parameter_data(const std::shared_ptr<parameter_data_t> &p) {
        parameter_data_ = p;
    }

   private:
    static_assert(std::is_arithmetic<ValueType>::value,
                  "ValueType for evaluator_base_tpl "
                  "must be a numerical type.");

    std::shared_ptr<parameter_data_t> parameter_data_;
};

/**
 * @brief Evaluator class that computes an output given an input. Also provides
 * the option for a parameterised evaluator of the form \f$ y = f_p(x) \f$
 *
 */
template <typename ValueType>
class scalar_evaluator_tpl : public evaluator_base_tpl<ValueType> {
   public:
    using base_t = evaluator_base_tpl<ValueType>;

    using value_t = typename base_t::value_t;
    using dense_vector_t = typename base_t::dense_vector_t;
    using sparse_vector_t = typename base_t::sparse_vector_t;
    using dense_matrix_t = typename base_t::dense_matrix_t;
    using sparse_matrix_t = typename base_t::sparse_matrix_t;
    using parameter_data_t = typename base_t::parameter_data_t;

    using shared_ptr_t = std::shared_ptr<scalar_evaluator_tpl<value_t>>;

    scalar_evaluator_tpl() = default;
    scalar_evaluator_tpl(const bopt_index &sz_in, const bopt_index &sz_p = 0)
        : evaluator_base_tpl<ValueType>(sz_in, out_size_t(1, 1), sz_p),
          ptr_(nullptr) {}

    /**
     * @brief Construct a new scalar evaluator tpl object using an existing
     * evaluator. Useful where evaluators are created through other means such
     * as code generation or symbolic algebra.
     *
     * @param evaluator shared_ptr
     */
    scalar_evaluator_tpl(const shared_ptr_t &ptr)
        : evaluator_base_tpl<ValueType>(ptr), ptr_(ptr) {}

    ~scalar_evaluator_tpl() = default;

    evaluator::return_status eval(const Eigen::Ref<const dense_vector_t> &x,
                                  value_t &out) {
        DBGASSERT(check_vector(x) && "Invalid input");
        return eval_impl(x, out);
    }

   protected:
    virtual evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x, value_t &out) {
        if (ptr_) return ptr_->eval(x, out);
        return evaluator::return_status::NotImplemented;
    }

   private:
    shared_ptr_t ptr_;
};

typedef scalar_evaluator_tpl<double> scalar_evaluator;

/**
 * @brief Evaluator class that computes an output given an input. Also provides
 * the option for a parameterised evaluator of the form \f$ y = f_p(x) \f$
 *
 */
template <typename ValueType>
class vector_evaluator_tpl : public evaluator_base_tpl<ValueType> {
   public:
    using base_t = evaluator_base_tpl<ValueType>;

    using value_t = typename base_t::value_t;
    using dense_vector_t = typename base_t::dense_vector_t;
    using sparse_vector_t = typename base_t::sparse_vector_t;
    using dense_matrix_t = typename base_t::dense_matrix_t;
    using sparse_matrix_t = typename base_t::sparse_matrix_t;
    using parameter_data_t = typename base_t::parameter_data_t;

    using shared_ptr_t = std::shared_ptr<vector_evaluator_tpl<value_t>>;

   public:
    vector_evaluator_tpl() = default;

    vector_evaluator_tpl(const bopt_index &sz_in, const bopt_index &sz_out,
                         const bopt_index &sz_p = 0)
        : evaluator_base_tpl<ValueType>(sz_in, out_size_t(sz_out, 1), sz_p) {}

    /**
     * @brief Construct a new vector_evaluator_tpl object using an existing
     * evaluator. Useful where evaluators are created through other means such
     * as code generation or symbolic algebra.
     *
     * @param evaluator shared_ptr
     */
    vector_evaluator_tpl(const shared_ptr_t &ptr)
        : evaluator_base_tpl<ValueType>(ptr), ptr_(ptr) {}

    ~vector_evaluator_tpl() = default;

    evaluator::return_status eval(const Eigen::Ref<const dense_vector_t> &x,
                                  Eigen::Ref<dense_vector_t> out) {
        DBGASSERT(check_vector(x) && "Invalid input");
        return eval_impl(x, out);
    }

   protected:
    virtual evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_vector_t> out) {
        if (ptr_) return ptr_->eval(x, out);
        return evaluator::return_status::NotImplemented;
    }

   private:
    shared_ptr_t ptr_;
};

typedef vector_evaluator_tpl<double> vector_evaluator;

template <typename ValueType>
class gradient_evaluator_tpl : public evaluator_base_tpl<ValueType> {
   public:
    using base_t = evaluator_base_tpl<ValueType>;

    using value_t = typename base_t::value_t;
    using dense_vector_t = typename base_t::dense_vector_t;
    using sparse_vector_t = typename base_t::sparse_vector_t;
    using dense_matrix_t = typename base_t::dense_matrix_t;
    using sparse_matrix_t = typename base_t::sparse_matrix_t;

    using shared_ptr_t = std::shared_ptr<gradient_evaluator_tpl<value_t>>;

    gradient_evaluator_tpl() = default;
    ~gradient_evaluator_tpl() = default;

    gradient_evaluator_tpl(const bopt_index &sz_in, const bopt_index &sz_grd,
                           const bopt_index &sz_p = 0)
        : evaluator_base_tpl<ValueType>(sz_in, out_size_t(1, sz_grd), sz_p),
          ptr_(nullptr) {}

    gradient_evaluator_tpl(const shared_ptr_t &ptr)
        : evaluator_base_tpl<ValueType>(ptr), ptr_(ptr) {}

    /**
     * @brief Evaluates the dense gradient for the expression \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_gradient(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_vector_t> out) {
        DBGASSERT(check_vector(x) && "Gradient input is invalid");
        DBGASSERT(out.rows() == rows_gradient() &&
                  out.cols() == cols_gradient() &&
                  "Gradient out is incorrect size");
        return eval_gradient_impl(x, out);
    }

    /**
     * @brief Evaluates the sparse gradient for the expression \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_gradient(
        const Eigen::Ref<const dense_vector_t> &x, sparse_vector_t &out) {
        DBGASSERT(check_vector(x) && "Gradient input is invalid");
        DBGASSERT(out.rows() == rows_gradient() &&
                  out.cols() == cols_gradient() &&
                  "Gradient out is incorrect size");
        return eval_gradient_impl(x, out);
    }

    /**
     * @brief The number of rows within the expression gradient
     *
     * @return bopt_index
     */
    virtual bopt_index rows_gradient() const { return this->sz_out().first; }

    /**
     * @brief The number of columns within the expression gradient
     *
     * @return bopt_index
     */
    virtual bopt_index cols_gradient() const { return this->sz_out().second; }

    /**
     * @brief Populates a sparse matrix with the sparsity pattern of the
     * gradient
     *
     * @param gradient
     */
    virtual void sparsity_gradient(sparse_vector_t &out) const {
        if (ptr_) ptr_->sparsity_gradient(out);
    }

   protected:
    virtual evaluator::return_status eval_gradient_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_vector_t> out) {
        if (ptr_) return ptr_->eval_gradient(x, out);
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_gradient_impl(
        const Eigen::Ref<const dense_vector_t> &x, sparse_vector_t &out) {
        if (ptr_) return ptr_->eval_gradient(x, out);
        return evaluator::return_status::NotImplemented;
    }

   private:
    shared_ptr_t ptr_;
};

typedef gradient_evaluator_tpl<double> gradient_evaluator;

/**
 * @brief Evaluator class containing interface methods for evaluation of a
 * Jacobian.
 *
 * @tparam ValueType
 */
template <typename ValueType>
class jacobian_evaluator_tpl : public evaluator_base_tpl<ValueType> {
   public:
    using base_t = evaluator_base_tpl<ValueType>;

    using value_t = typename base_t::value_t;
    using dense_vector_t = typename base_t::dense_vector_t;
    using sparse_vector_t = typename base_t::sparse_vector_t;
    using dense_matrix_t = typename base_t::dense_matrix_t;
    using sparse_matrix_t = typename base_t::sparse_matrix_t;

    using shared_ptr_t = std::shared_ptr<jacobian_evaluator_tpl<value_t>>;

    jacobian_evaluator_tpl() = default;
    ~jacobian_evaluator_tpl() = default;

    jacobian_evaluator_tpl(const bopt_index &sz_in, const out_size_t &sz_out,
                           const bopt_index &sz_p = 0)
        : base_t(sz_in, sz_out, sz_p), ptr_(nullptr) {}

    jacobian_evaluator_tpl(const shared_ptr_t &ptr) : base_t(ptr), ptr_(ptr) {}

    /**
     * @brief Evaluates the dense jacobian for the expression \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_jacobian(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_matrix_t> out) {
        DBGASSERT(check_vector(x) && "Jacobian input is invalid");
        return eval_jacobian_impl(x, out);
    }

    /**
     * @brief Evaluates the sparse jacobian for the expression \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_jacobian(
        const Eigen::Ref<const dense_vector_t> &x, sparse_matrix_t &out) {
        DBGASSERT(check_vector(x) && "Jacobian input is invalid");
        return eval_jacobian_impl(x, out);
    }

    /**
     * @brief The number of rows within the expression jacobian
     *
     * @return bopt_index
     */
    virtual bopt_index rows_jacobian() const { return this->sz_out().first; }

    /**
     * @brief The number of columns within the expression jacobian
     *
     * @return bopt_index
     */
    virtual bopt_index cols_jacobian() const { return this->sz_out().second; }

    /**
     * @brief Populates a sparse matrix with the sparsity pattern of the
     * jacobian
     *
     * @param jacobian
     */
    virtual void sparsity_jacobian(sparse_matrix_t &out) const {
        if (ptr_) ptr_->sparsity_jacobian(out);
    }

   protected:
    virtual evaluator::return_status eval_jacobian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_matrix_t> out) {
        if (ptr_) return ptr_->eval_jacobian(x, out);
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_jacobian_impl(
        const Eigen::Ref<const dense_vector_t> &x, sparse_matrix_t &out) {
        if (ptr_) return ptr_->eval_jacobian(x, out);
        return evaluator::return_status::NotImplemented;
    }

   private:
    shared_ptr_t ptr_;
};

typedef jacobian_evaluator_tpl<double> jacobian_evaluator;

template <typename ValueType>
class hessian_evaluator_tpl : public evaluator_base_tpl<ValueType> {
   public:
    using base_t = evaluator_base_tpl<ValueType>;

    using value_t = typename base_t::value_t;
    using dense_vector_t = typename base_t::dense_vector_t;
    using sparse_vector_t = typename base_t::sparse_vector_t;
    using dense_matrix_t = typename base_t::dense_matrix_t;
    using sparse_matrix_t = typename base_t::sparse_matrix_t;

    using shared_ptr_t = std::shared_ptr<hessian_evaluator_tpl<value_t>>;

    hessian_evaluator_tpl() = default;
    ~hessian_evaluator_tpl() = default;

    hessian_evaluator_tpl(const bopt_index &sz_in, const out_size_t &sz_out,
                          const bopt_index &sz_p = 0)
        : base_t(sz_in, sz_out, sz_p), ptr_(nullptr) {}

    hessian_evaluator_tpl(const shared_ptr_t &ptr) : base_t(ptr), ptr_(ptr) {}

    /**
     * @brief Evaluates the dense hessian for the expression \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial^2 (\lambda ^T c)}{\partial x^2}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_hessian(
        const Eigen::Ref<const dense_vector_t> &x,
        const Eigen::Ref<const dense_vector_t> &lambda,
        Eigen::Ref<dense_matrix_t> out) {
        DBGASSERT(check_vector(x) && check_vector(lambda) &&
                  "Hessian input is invalid");
        return eval_hessian_impl(x, lambda, out);
    }

    /**
     * @brief Evaluates the sparse hessian for the expression \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial^2 (\lambda ^T c)}{\partial x^2}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_hessian(
        const Eigen::Ref<const dense_vector_t> &x,
        const Eigen::Ref<const dense_vector_t> &lambda, sparse_matrix_t &out) {
        DBGASSERT(check_vector(x) && check_vector(lambda) &&
                  "Hessian input is invalid");
        return eval_hessian_impl(x, lambda, out);
    }

    /**
     * @brief The number of rows within the expression hessian
     *
     * @return bopt_index
     */
    virtual bopt_index rows_hessian() const { return this->sz_out().first; }

    /**
     * @brief The number of columns within the expression hessian
     *
     * @return bopt_index
     */
    virtual bopt_index cols_hessian() const { return this->sz_out().second; }

    /**
     * @brief Populates a sparse matrix with the sparsity pattern of the
     * hessian
     *
     * @param hessian
     */
    virtual void sparsity_hessian(sparse_matrix_t &out) const {
        if (ptr_) ptr_->sparsity_hessian(out);
    }

   protected:
    virtual evaluator::return_status eval_hessian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        const Eigen::Ref<const dense_vector_t> &lambda,
        Eigen::Ref<dense_matrix_t> out) {
        if (ptr_) return ptr_->eval_hessian(x, lambda, out);
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_hessian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        const Eigen::Ref<const dense_vector_t> &lambda, sparse_matrix_t &out) {
        if (ptr_) return ptr_->eval_hessian(x, lambda, out);
        return evaluator::return_status::NotImplemented;
    }

   private:
    shared_ptr_t ptr_;
};

typedef hessian_evaluator_tpl<double> hessian_evaluator;

template <typename ValueType>
class linear_vector_evaluator_tpl : public vector_evaluator_tpl<ValueType> {
   public:
    using base_t = vector_evaluator_tpl<ValueType>;

    using value_t = typename base_t::value_t;
    using dense_vector_t = typename base_t::dense_vector_t;
    using sparse_vector_t = typename base_t::sparse_vector_t;
    using dense_matrix_t = typename base_t::dense_matrix_t;
    using sparse_matrix_t = typename base_t::sparse_matrix_t;

    using shared_ptr_t = std::shared_ptr<linear_vector_evaluator_tpl<value_t>>;

    linear_vector_evaluator_tpl() = default;
    ~linear_vector_evaluator_tpl() = default;

    linear_vector_evaluator_tpl(const bopt_index &sz_in,
                                const out_size_t &sz_out,
                                const bopt_index &sz_p = 0)
        : base_t(sz_in, sz_out, sz_p), ptr_(nullptr) {}

    linear_vector_evaluator_tpl(const shared_ptr_t &ptr)
        : base_t(ptr), ptr_(ptr) {}

    /**
     * @brief Evaluates the dense jacobian for the expression \f$ c(x) \f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_A(Eigen::Ref<dense_matrix_t> out) {
        return eval_A_impl(out);
    }

    /**
     * @brief Evaluates the sparse coefficient matrix for the linear expression
     * \f$ c(x)\f$ (i.e. \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_A(sparse_matrix_t &out) {
        return eval_A_impl(out);
    }

    /**
     * @brief The number of rows within the coefficient matrix A
     *
     * @return bopt_index
     */
    virtual bopt_index rows_A() const { return this->sz_out().first; }

    /**
     * @brief The number of columns within the coefficient matrix A
     * @return bopt_index
     */
    virtual bopt_index cols_A() const { return this->sz_in(); }

    /**
     * @brief Populates a sparse matrix with the sparsity pattern of the
     * coefficient matrix A
     *
     * @param A
     */
    virtual void sparsity_A(sparse_matrix_t &out) const {
        if (ptr_) ptr_->sparsity_A(out);
    }

    /**
     * @brief Evaluates the dense jacobian for the expression \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_b(Eigen::Ref<dense_vector_t> out) {
        return eval_b_impl(out);
    }

    /**
     * @brief Evaluates the sparse coefficient matrix for the linear expression
     * \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_b(sparse_vector_t &out) {
        return eval_b_impl(out);
    }

    /**
     * @brief The number of rows within the coefficient vector b
     *
     * @return bopt_index
     */
    virtual bopt_index rows_b() const { return this->sz_out().first; }

    /**
     * @brief Populates a sparse matrix with the sparsity pattern of the
     * coefficient matrix A
     *
     * @param A
     */
    virtual void sparsity_b(sparse_vector_t &out) const {
        if (ptr_) ptr_->sparsity_b(out);
    }

   protected:
    virtual evaluator::return_status eval_A_impl(
        Eigen::Ref<dense_matrix_t> out) {
        if (ptr_) return ptr_->eval_A(out);
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_A_impl(sparse_matrix_t &out) {
        if (ptr_) return ptr_->eval_A(out);
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_b_impl(
        Eigen::Ref<dense_vector_t> out) {
        if (ptr_) return ptr_->eval_b(out);
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_b_impl(sparse_vector_t &out) {
        if (ptr_) return ptr_->eval_b(out);
        return evaluator::return_status::NotImplemented;
    }

   private:
    shared_ptr_t ptr_;
};

template <typename ValueType>
class linear_scalar_evaluator_tpl : public scalar_evaluator_tpl<ValueType> {
   public:
    using base_t = scalar_evaluator_tpl<ValueType>;

    using value_t = typename base_t::value_t;
    using dense_vector_t = typename base_t::dense_vector_t;
    using sparse_vector_t = typename base_t::sparse_vector_t;
    using dense_matrix_t = typename base_t::dense_matrix_t;
    using sparse_matrix_t = typename base_t::sparse_matrix_t;

    using shared_ptr_t = std::shared_ptr<linear_scalar_evaluator_tpl<value_t>>;

    linear_scalar_evaluator_tpl() = default;

    linear_scalar_evaluator_tpl(const bopt_index &sz_in,
                                const bopt_index &sz_p = 0)
        : base_t(sz_in, out_size_t(1, 1), sz_p), ptr_(nullptr) {}

    linear_scalar_evaluator_tpl(const shared_ptr_t &ptr)
        : base_t(ptr), ptr_(ptr) {}

    /**
     * @brief Evaluates the dense jacobian for the expression \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_a(Eigen::Ref<dense_vector_t> out) {
        return eval_a_impl(out);
    }

    /**
     * @brief Evaluates the sparse coefficient matrix for the linear expression
     * \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_a(sparse_vector_t &out) {
        return eval_a_impl(out);
    }

    /**
     * @brief The number of rows within the coefficient vector a for the
     * expression a^T x + b
     *
     * @return bopt_index
     */
    virtual bopt_index rows_a() const { return this->sz_in(); }

    /**
     * @brief Populates a sparse matrix with the sparsity pattern of the
     * coefficient matrix A
     *
     * @param A
     */
    virtual void sparsity_a(sparse_vector_t &out) const {
        if (ptr_) ptr_->sparsity_a(out);
    }

    /**
     * @brief Evaluates the dense jacobian for the expression \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_b(ValueType &out) { return eval_b_impl(out); }

   protected:
    virtual evaluator::return_status eval_a_impl(
        Eigen::Ref<dense_vector_t> out) {
        if (ptr_) return ptr_->eval_a(out);
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_a_impl(sparse_vector_t &out) {
        if (ptr_) return ptr_->eval_a(out);
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_b_impl(ValueType &out) {
        if (ptr_) return ptr_->eval_b(out);
        return evaluator::return_status::NotImplemented;
    }

   private:
    shared_ptr_t ptr_;
};

template <typename ValueType>
class quadratic_scalar_evaluator_tpl : public scalar_evaluator_tpl<ValueType> {
   public:
    using base_t = scalar_evaluator_tpl<ValueType>;

    using value_t = typename base_t::value_t;
    using dense_vector_t = typename base_t::dense_vector_t;
    using sparse_vector_t = typename base_t::sparse_vector_t;
    using dense_matrix_t = typename base_t::dense_matrix_t;
    using sparse_matrix_t = typename base_t::sparse_matrix_t;

    using shared_ptr_t =
        std::shared_ptr<quadratic_scalar_evaluator_tpl<value_t>>;

    quadratic_scalar_evaluator_tpl() = default;
    quadratic_scalar_evaluator_tpl(const bopt_index &sz_in,
                                   const bopt_index &sz_p = 0)
        : base_t(sz_in, out_size_t(1, 1), sz_p), ptr_(nullptr) {}

    quadratic_scalar_evaluator_tpl(const shared_ptr_t &ptr)
        : base_t(ptr), ptr_(ptr) {}

    /**
     * @brief Evaluates the dense jacobian for the expression \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_A(Eigen::Ref<dense_matrix_t> out) {
        return eval_A_impl(out);
    }

    /**
     * @brief Evaluates the sparse coefficient matrix for the quadratic
     * expression
     * \f$c(x)\f$ (i.e. \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_A(sparse_matrix_t &out) {
        return eval_A_impl(out);
    }

    /**
     * @brief The number of rows within the coefficient matrix A
     *
     * @return bopt_index
     */
    virtual bopt_index rows_A() const { return this->sz_in(); }

    /**
     * @brief The number of columns within the coefficient matrix A
     *
     * @return bopt_index
     */
    virtual bopt_index cols_A() const { return this->sz_in(); }

    /**
     * @brief Populates a sparse matrix with the sparsity pattern of the
     * coefficient matrix A
     *
     * @param out
     */
    virtual void sparsity_A(sparse_matrix_t &out) const {
        if (ptr_) ptr_->sparsity_A(out);
    }

    /**
     * @brief Evaluates the dense jacobian for the expression \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_b(Eigen::Ref<dense_vector_t> out) {
        return eval_b_impl(out);
    }

    /**
     * @brief Evaluates the dense jacobian for the expression \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
     *
     * @param x
     * @param out
     * @return evaluator::return_status
     */
    evaluator::return_status eval_b(sparse_vector_t &out) {
        return eval_b_impl(out);
    }

    /**
     * @brief The number of rows within the coefficient vector b
     *
     * @return bopt_index
     */
    virtual bopt_index rows_b() const { return this->sz_in(); }

    /**
     * @brief Populates a sparse vector with the sparsity pattern of the
     * coefficient vector b
     *
     * @param A
     */
    virtual void sparsity_b(sparse_vector_t &out) const {
        if (ptr_) ptr_->sparsity_b(out);
    }

    evaluator::return_status eval_c(value_t &out) { return eval_c_impl(out); }

   protected:
    virtual evaluator::return_status eval_A_impl(
        Eigen::Ref<dense_matrix_t> out) {
        if (ptr_) return ptr_->eval_A(out);
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_A_impl(sparse_matrix_t &out) {
        if (ptr_) return ptr_->eval_A(out);
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_b_impl(
        Eigen::Ref<dense_vector_t> out) {
        if (ptr_) return ptr_->eval_b(out);
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_b_impl(sparse_vector_t &out) {
        if (ptr_) return ptr_->eval_b(out);
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_c_impl(value_t &out) {
        if (ptr_) return ptr_->eval_c(out);
        return evaluator::return_status::NotImplemented;
    }

   private:
    shared_ptr_t ptr_;
};

/**
 * @brief A twice-differentiable expression of the form \f$y = f_p(x)\f$
 *
 * @tparam ValueType
 */
template <typename ValueType>
class differentiable_scalar_evaluator_tpl
    : public scalar_evaluator_tpl<ValueType>,
      public gradient_evaluator_tpl<ValueType>,
      public hessian_evaluator_tpl<ValueType> {
   public:
    using base_t = scalar_evaluator_tpl<ValueType>;

    using value_t = typename base_t::value_t;
    using dense_vector_t = typename base_t::dense_vector_t;
    using sparse_vector_t = typename base_t::sparse_vector_t;
    using dense_matrix_t = typename base_t::dense_matrix_t;
    using sparse_matrix_t = typename base_t::sparse_matrix_t;

    using parameter_data_t = typename base_t::parameter_data_t;

    using gradient_evaluator_t = gradient_evaluator_tpl<ValueType>;
    using hessian_evaluator_t = hessian_evaluator_tpl<ValueType>;

    using shared_ptr_t =
        std::shared_ptr<differentiable_scalar_evaluator_tpl<value_t>>;

    differentiable_scalar_evaluator_tpl() = default;

    differentiable_scalar_evaluator_tpl(const bopt_index &sz_in,
                                        const bopt_index &sz_p = 0)
        : base_t(sz_in, sz_p),
          gradient_evaluator_t(sz_in, sz_in, sz_p),
          hessian_evaluator_t(sz_in, out_size_t(sz_in, sz_in), sz_p) {
        // Set parameters to this
        gradient_evaluator_t::parameters(this->parameter());
        hessian_evaluator_t::parameters(this->parameter());
    }

    differentiable_scalar_evaluator_tpl(const shared_ptr_t &ptr)
        : base_t(ptr), gradient_evaluator_t(ptr), hessian_evaluator_t(ptr) {}

    ~differentiable_scalar_evaluator_tpl() = default;

    using base_t::parameter_data;
    using base_t::parameters;

    const bopt_index &sz_in() const { return base_t::sz_in(); }
    const out_size_t &sz_out() const { return base_t::sz_out(); }

   protected:
   private:
};

typedef differentiable_scalar_evaluator_tpl<double>
    differentiable_scalar_evaluator;

/**
 * @brief A twice-differentiable expression of the form \f$y = f_p(x)\f$
 *
 * @tparam ValueType
 */
template <typename ValueType>
class differentiable_vector_evaluator_tpl
    : public vector_evaluator_tpl<ValueType>,
      public jacobian_evaluator_tpl<ValueType>,
      public hessian_evaluator_tpl<ValueType> {
   public:
    using base_t = vector_evaluator_tpl<ValueType>;

    using value_t = typename base_t::value_t;
    using dense_vector_t = typename base_t::dense_vector_t;
    using sparse_vector_t = typename base_t::sparse_vector_t;
    using dense_matrix_t = typename base_t::dense_matrix_t;
    using sparse_matrix_t = typename base_t::sparse_matrix_t;

    using parameter_data_t = typename base_t::parameter_data_t;

    using jacobian_evaluator_t = jacobian_evaluator_tpl<ValueType>;
    using hessian_evaluator_t = hessian_evaluator_tpl<ValueType>;

    using shared_ptr_t =
        std::shared_ptr<differentiable_vector_evaluator_tpl<value_t>>;

    differentiable_vector_evaluator_tpl() = default;

    differentiable_vector_evaluator_tpl(const bopt_index &sz_in,
                                        const bopt_index &sz_out,
                                        const bopt_index &sz_p = 0)
        : base_t(sz_in, sz_p),
          jacobian_evaluator_t(sz_in, out_size_t(sz_out, sz_in), sz_p),
          hessian_evaluator_t(sz_in, out_size_t(sz_in, sz_in), sz_p) {
        // Set parameters to this
        jacobian_evaluator_t::parameters(this->parameter());
        hessian_evaluator_t::parameters(this->parameter());
    }

    differentiable_vector_evaluator_tpl(const shared_ptr_t &ptr)
        : base_t(ptr), jacobian_evaluator_t(ptr), hessian_evaluator_t(ptr) {}

    ~differentiable_vector_evaluator_tpl() = default;

    using base_t::parameter_data;
    using base_t::parameters;

    const bopt_index &sz_in() const { return base_t::sz_in(); }
    const out_size_t &sz_out() const { return base_t::sz_out(); }

    using base_t::sz_in;

   protected:
   private:
};

typedef differentiable_vector_evaluator_tpl<double>
    differentiable_vector_evaluator;

template <typename DenseType, typename SparseType>
struct dense_sparse_buffer_tpl {
    DenseType dense;
    SparseType sparse;
};

}  // namespace bopt
