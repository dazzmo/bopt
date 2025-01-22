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

class input_output_traits {
   public:
    input_output_traits() = default;
    ~input_output_traits() = default;

    input_output_traits(const bopt_index &sz_in, const bopt_index &sz_out)
        : sz_in_(sz_in), sz_out_(sz_out) {};

    const bopt_index &sz_in() const { return sz_in_; }
    const bopt_index &sz_out() const { return sz_out_; }

   private:
    // Input size
    bopt_index sz_in_;
    // Output size
    bopt_index sz_out_;
};

template <typename ValueType>
class parameter_traits {
   public:
    using dense_vector_t = dense_vector_tpl<ValueType>;

    parameter_traits() = default;
    ~parameter_traits() = default;

    parameter_traits(const bopt_index &sz_p)
        : sz_p_(sz_p), p_(dense_vector_t::Zero(sz_p)), ptr_(nullptr) {}

    /**
     * @brief Construct a new parameter traits object through forward
     * propagation of an existing parameter_traits object.
     *
     * @param parameters
     */
    parameter_traits(const std::shared_ptr<parameter_traits> &parameters)
        : sz_p_(parameters->sz_p()), p_(0), ptr_(parameters) {}

    /**
     * @brief Number of parameters
     *
     * @return const bopt_index&
     */
    const bopt_index &sz_p() const { return sz_p_; }

    const dense_vector_t &parameters() const {
        if (ptr_) return ptr_->parameters();
        return p_;
    }

    void set_parameters(const Eigen::Ref<const dense_vector_t> &p) {
        if (ptr_) {
            ptr_->set_parameters(p);
        } else {
            p_ = p;
        }
    }

   private:
    // Number of parameters
    bopt_index sz_p_;
    // Parameter vector
    dense_vector_t p_;

    // Pointer to existing parameter_traits object (if applicable)
    std::shared_ptr<parameter_traits<ValueType>> ptr_;
};

/**
 * @brief Evaluator class that computes an output given an input. Also provides
 * the option for a parameterised evaluator of the form \f$ y = f_p(x) \f$
 *
 */
template <typename ValueType>
class scalar_evaluator_tpl : public input_output_traits,
                             public parameter_traits<ValueType> {
   public:
    using value_t = ValueType;
    using dense_vector_t = dense_vector_tpl<value_t>;
    using sparse_vector_t = sparse_vector_tpl<value_t>;
    using dense_matrix_t = dense_matrix_tpl<value_t>;
    using sparse_matrix_t = sparse_matrix_tpl<value_t>;

    scalar_evaluator_tpl() = default;
    scalar_evaluator_tpl(const bopt_index &sz_in, const bopt_index &sz_p = 0)
        : input_output_traits(sz_in, 1),
          parameter_traits<ValueType>(sz_p),
          evaluator_(nullptr) {}

    /**
     * @brief Construct a new scalar evaluator tpl object using an existing
     * evaluator. Useful where evaluators are created through other means such
     * as code generation or symbolic algebra.
     *
     * @param evaluator shared_ptr
     */
    scalar_evaluator_tpl(
        const std::shared_ptr<scalar_evaluator_tpl<ValueType>> &evaluator)
        : input_output_traits(evaluator->sz_in(), evaluator->sz_out()),
          parameter_traits<ValueType>(evaluator),
          evaluator_(evaluator) {}

    ~scalar_evaluator_tpl() = default;

    evaluator::return_status eval(const Eigen::Ref<const dense_vector_t> &x,
                                  value_t &out) {
        DBGASSERT(check_vector(x) && "Invalid input");
        return eval_impl(x, out);
    }

   protected:
    virtual evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x, value_t &out) {
        if (evaluator_) return evaluator_->eval(x, out);
        return evaluator::return_status::NotImplemented;
    }

   private:
    static_assert(std::is_arithmetic<ValueType>::value,
                  "ValueType for evaluator_tpl "
                  "must be a numerical type.");

    std::shared_ptr<scalar_evaluator_tpl<ValueType>> evaluator_;
};

typedef scalar_evaluator_tpl<double> scalar_evaluator;

/**
 * @brief Evaluator class that computes an output given an input. Also provides
 * the option for a parameterised evaluator of the form \f$ y = f_p(x) \f$
 *
 */
template <typename ValueType>
class vector_evaluator_tpl : public input_output_traits,
                             public parameter_traits<ValueType> {
   public:
    using value_t = ValueType;
    using dense_vector_t = dense_vector_tpl<value_t>;
    using sparse_vector_t = sparse_vector_tpl<value_t>;
    using dense_matrix_t = dense_matrix_tpl<value_t>;
    using sparse_matrix_t = sparse_matrix_tpl<value_t>;

   public:
    vector_evaluator_tpl()
        : input_output_traits(0, 0), parameter_traits<ValueType>(0) {}

    vector_evaluator_tpl(const bopt_index &sz_in, const bopt_index &sz_out,
                         const bopt_index &sz_p = 0)
        : input_output_traits(sz_in, sz_out),
          parameter_traits<ValueType>(sz_p) {}

    /**
     * @brief Construct a new vector_evaluator_tpl object using an existing
     * evaluator. Useful where evaluators are created through other means such
     * as code generation or symbolic algebra.
     *
     * @param evaluator shared_ptr
     */
    vector_evaluator_tpl(
        const std::shared_ptr<vector_evaluator_tpl<ValueType>> &evaluator)
        : input_output_traits(evaluator->sz_in(), evaluator->sz_out()),
          parameter_traits<ValueType>(evaluator),
          evaluator_(evaluator) {}

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
        if (evaluator_) return evaluator_->eval(x, out);
        return evaluator::return_status::NotImplemented;
    }

   private:
    static_assert(std::is_arithmetic<ValueType>::value,
                  "ValueType for evaluator_tpl "
                  "must be a numerical type.");

    std::shared_ptr<vector_evaluator_tpl<ValueType>> evaluator_;
};

typedef vector_evaluator_tpl<double> vector_evaluator;

template <typename ValueType>
class gradient_evaluator_tpl {
   public:
    using value_t = ValueType;
    using dense_vector_t = dense_vector_tpl<value_t>;
    using sparse_vector_t = sparse_vector_tpl<value_t>;
    using dense_matrix_t = dense_matrix_tpl<value_t>;
    using sparse_matrix_t = sparse_matrix_tpl<value_t>;

    gradient_evaluator_tpl() = default;
    ~gradient_evaluator_tpl() = default;

    gradient_evaluator_tpl(const bopt_int &sz) : sz_(sz), ptr_(nullptr) {}

    gradient_evaluator_tpl(
        const std::shared_ptr<gradient_evaluator_tpl<ValueType>> &ptr)
        : sz_(0), ptr_(ptr) {}

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
        return eval_gradient_impl(x, out);
    }

    /**
     * @brief The number of rows within the expression gradient
     *
     * @return bopt_index
     */
    virtual bopt_index cols_gradient() const {
        if (ptr_) return ptr_->cols_gradient();
        return sz_;
    }

    /**
     * @brief Populates a sparse matrix with the sparsity pattern of the
     * gradient
     *
     * @param gradient
     */
    virtual void sparsity_gradient(sparse_vector_t &gradient) const {}

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
    bopt_index sz_;

    std::shared_ptr<gradient_evaluator_tpl<ValueType>> ptr_;
};

typedef gradient_evaluator_tpl<double> gradient_evaluator;

/**
 * @brief Evaluator class containing interface methods for evaluation of a
 * Jacobian.
 *
 * @tparam ValueType
 */
template <typename ValueType>
class jacobian_evaluator_tpl {
   public:
    using value_t = ValueType;
    using dense_vector_t = dense_vector_tpl<value_t>;
    using sparse_vector_t = sparse_vector_tpl<value_t>;
    using dense_matrix_t = dense_matrix_tpl<value_t>;
    using sparse_matrix_t = sparse_matrix_tpl<value_t>;

    jacobian_evaluator_tpl() = default;
    ~jacobian_evaluator_tpl() = default;

    jacobian_evaluator_tpl(const bopt_index &sz_m, const bopt_index &sz_n)
        : sz_m_(sz_m), sz_n_(sz_n), ptr_(nullptr) {}

    jacobian_evaluator_tpl(
        const std::shared_ptr<jacobian_evaluator_tpl<ValueType>> &ptr)
        : sz_m_(ptr->rows_jacobian()), sz_n_(ptr->cols_jacobian()), ptr_(ptr) {}

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
    virtual bopt_index rows_jacobian() const { return sz_m_; }

    /**
     * @brief The number of columns within the expression jacobian
     *
     * @return bopt_index
     */
    virtual bopt_index cols_jacobian() const { return sz_n_; }

    /**
     * @brief Populates a sparse matrix with the sparsity pattern of the
     * jacobian
     *
     * @param jacobian
     */
    virtual void sparsity_jacobian(sparse_matrix_t &jacobian) const {
        if (ptr_) ptr_->sparsity_jacobian(jacobian);
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
    bopt_index sz_m_;
    bopt_index sz_n_;

    std::shared_ptr<jacobian_evaluator_tpl<ValueType>> ptr_;
};

typedef jacobian_evaluator_tpl<double> jacobian_evaluator;

template <typename ValueType>
class hessian_evaluator_tpl {
   public:
    using value_t = ValueType;
    using dense_vector_t = dense_vector_tpl<value_t>;
    using sparse_vector_t = sparse_vector_tpl<value_t>;
    using dense_matrix_t = dense_matrix_tpl<value_t>;
    using sparse_matrix_t = sparse_matrix_tpl<value_t>;

    hessian_evaluator_tpl() = default;
    ~hessian_evaluator_tpl() = default;

    hessian_evaluator_tpl(const bopt_index &sz) : sz_(sz), ptr_(nullptr) {}

    hessian_evaluator_tpl(
        const std::shared_ptr<hessian_evaluator_tpl<ValueType>> &ptr)
        : sz_(0), ptr_(ptr) {}

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
    virtual bopt_index rows_hessian() const { return sz_; }

    /**
     * @brief The number of columns within the expression hessian
     *
     * @return bopt_index
     */
    virtual bopt_index cols_hessian() const { return sz_; }

    /**
     * @brief Populates a sparse matrix with the sparsity pattern of the
     * hessian
     *
     * @param hessian
     */
    virtual void sparsity_hessian(sparse_matrix_t &hessian) const {
        if (ptr_) ptr_->sparsity_hessian(hessian);
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
    bopt_index sz_;

    std::shared_ptr<hessian_evaluator_tpl<ValueType>> ptr_;
};

typedef hessian_evaluator_tpl<double> hessian_evaluator;

template <typename ValueType>
class linear_vector_evaluator_tpl : public vector_evaluator_tpl<ValueType> {
   public:
    using typename vector_evaluator_tpl<ValueType>::value_t;
    using typename vector_evaluator_tpl<ValueType>::dense_vector_t;
    using typename vector_evaluator_tpl<ValueType>::sparse_vector_t;
    using typename vector_evaluator_tpl<ValueType>::dense_matrix_t;
    using typename vector_evaluator_tpl<ValueType>::sparse_matrix_t;

    linear_vector_evaluator_tpl() = default;
    ~linear_vector_evaluator_tpl() = default;

    linear_vector_evaluator_tpl(const bopt_index &sz_in,
                                const bopt_index &sz_out,
                                const bopt_index &sz_p)
        : vector_evaluator_tpl<ValueType>(sz_in, sz_out, sz_p) {}

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
     * @brief Evaluates the sparse coefficient matrix for the linear expression
     * \f$c(x)\f$ (i.e.
     * \f$ \frac{\partial c}{\partial x}\f$)
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
    virtual bopt_index rows_A() const { return this->sz_out(); }

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
    virtual void sparsity_A(sparse_matrix_t &A) const {}

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
    virtual bopt_index rows_b() const { return this->sz_out(); }

    /**
     * @brief Populates a sparse matrix with the sparsity pattern of the
     * coefficient matrix A
     *
     * @param A
     */
    virtual void sparsity_b(sparse_vector_t &b) const {}

   protected:
    virtual evaluator::return_status eval_A_impl(
        Eigen::Ref<dense_matrix_t> out) {
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_A_impl(sparse_matrix_t &out) {
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_b_impl(
        Eigen::Ref<dense_vector_t> out) {
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_b_impl(sparse_vector_t &out) {
        return evaluator::return_status::NotImplemented;
    }

   private:
};

template <typename ValueType>
class linear_scalar_evaluator_tpl : public scalar_evaluator_tpl<ValueType> {
   public:
    using typename scalar_evaluator_tpl<ValueType>::value_t;
    using typename scalar_evaluator_tpl<ValueType>::dense_vector_t;
    using typename scalar_evaluator_tpl<ValueType>::sparse_vector_t;
    using typename scalar_evaluator_tpl<ValueType>::dense_matrix_t;
    using typename scalar_evaluator_tpl<ValueType>::sparse_matrix_t;

    linear_scalar_evaluator_tpl() = default;

    linear_scalar_evaluator_tpl(const bopt_index &sz_in,
                                const bopt_index &sz_p = 0)
        : scalar_evaluator_tpl<ValueType>(sz_in, sz_p) {}

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
    virtual void sparsity_a(sparse_vector_t &a) const {}

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
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_a_impl(sparse_vector_t &out) {
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_b_impl(ValueType &out) {
        return evaluator::return_status::NotImplemented;
    }

   private:
};

template <typename ValueType>
class quadratic_scalar_evaluator_tpl : public scalar_evaluator_tpl<ValueType> {
   public:
    using typename scalar_evaluator_tpl<ValueType>::value_t;
    using typename scalar_evaluator_tpl<ValueType>::dense_vector_t;
    using typename scalar_evaluator_tpl<ValueType>::sparse_vector_t;
    using typename scalar_evaluator_tpl<ValueType>::dense_matrix_t;
    using typename scalar_evaluator_tpl<ValueType>::sparse_matrix_t;

    quadratic_scalar_evaluator_tpl() = default;
    quadratic_scalar_evaluator_tpl(const bopt_index &sz_in,
                                   const bopt_index &sz_p = 0)
        : scalar_evaluator_tpl<ValueType>(sz_in, sz_p) {}

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
     * @param A
     */
    virtual void sparsity_A(sparse_matrix_t &A) const {}

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
    virtual void sparsity_b(sparse_vector_t &b) const {}

    evaluator::return_status eval_c(value_t &out) { return eval_c_impl(out); }

   protected:
    virtual evaluator::return_status eval_A_impl(
        Eigen::Ref<dense_matrix_t> out) {
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_A_impl(sparse_matrix_t &out) {
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_b_impl(
        Eigen::Ref<dense_vector_t> out) {
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_b_impl(sparse_vector_t &out) {
        return evaluator::return_status::NotImplemented;
    }

    virtual evaluator::return_status eval_c_impl(value_t &out) {
        return evaluator::return_status::NotImplemented;
    }

   private:
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
    using typename scalar_evaluator_tpl<ValueType>::value_t;
    using typename scalar_evaluator_tpl<ValueType>::dense_vector_t;
    using typename scalar_evaluator_tpl<ValueType>::sparse_vector_t;
    using typename scalar_evaluator_tpl<ValueType>::dense_matrix_t;
    using typename scalar_evaluator_tpl<ValueType>::sparse_matrix_t;

    differentiable_scalar_evaluator_tpl() = default;

    differentiable_scalar_evaluator_tpl(const bopt_index &sz_in,
                                        const bopt_index &sz_p = 0)
        : scalar_evaluator_tpl<ValueType>(sz_in, sz_p) {}

    differentiable_scalar_evaluator_tpl(
        const std::shared_ptr<differentiable_scalar_evaluator_tpl<ValueType>>
            &ptr)
        : scalar_evaluator_tpl<ValueType>(ptr),
          gradient_evaluator_tpl<ValueType>(ptr),
          hessian_evaluator_tpl<ValueType>(ptr) {}

    ~differentiable_scalar_evaluator_tpl() = default;

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
    using typename vector_evaluator_tpl<ValueType>::value_t;
    using typename vector_evaluator_tpl<ValueType>::dense_vector_t;
    using typename vector_evaluator_tpl<ValueType>::sparse_vector_t;
    using typename vector_evaluator_tpl<ValueType>::dense_matrix_t;
    using typename vector_evaluator_tpl<ValueType>::sparse_matrix_t;

    differentiable_vector_evaluator_tpl() = default;
    ~differentiable_vector_evaluator_tpl() = default;

    differentiable_vector_evaluator_tpl(const bopt_index &sz_in,
                                        const bopt_index &sz_out,
                                        const bopt_index &sz_p = 0)
        : vector_evaluator_tpl<ValueType>(sz_in, sz_out, sz_p) {}

    differentiable_vector_evaluator_tpl(
        const std::shared_ptr<differentiable_vector_evaluator_tpl<ValueType>>
            &ptr)
        : vector_evaluator_tpl<ValueType>(ptr),
          jacobian_evaluator_tpl<ValueType>(ptr),
          hessian_evaluator_tpl<ValueType>(ptr) {}

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
