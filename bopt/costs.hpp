#pragma once

#include <memory>

#include "bopt/evaluator.hpp"
#include "bopt/logging.hpp"

namespace bopt {

class Cost : public EvaluatorBase {
   public:
    Cost(const Index &n_inputs, const std::string &name = "")
        : EvaluatorBase(n_inputs, 1), name_(name), scaling_factor_(1.0) {
        setDescription("Default Cost");
    }

    const std::string &name() const { return name_; }
    void setName(const std::string &name) { name_ = name; }

    const double &scaling_factor() const { return scaling_factor_; }
    void setScalingFactor(const double &factor) { scaling_factor_ = factor; }

   protected:
   private:
    std::string name_;
    double scaling_factor_;
};

std::ostream &operator<<(std::ostream &os, const Cost &c);

class LinearCost : public Cost {
   public:
    const VectorXd &a() const { return a_; }
    const double &b() const { return b_; }

    void seta(const Eigen::Ref<const VectorXd> &a) { a_ = a; }
    void setb(const double &b) { b_ = b; }

    // void setSparsityPattern();

   protected:
    void jacobianImpl(const Eigen::Ref<const VectorXd> &x,
                      Eigen::Ref<Eigen::MatrixXd> jacobian) override {
        jacobian = a();
    }

   private:
    VectorXd a_;
    double b_;
};

class QuadraticCost : public Cost {
   public:
    const MatrixXd &A() const { return A_; }
    const VectorXd &b() const { return b_; }

    // const SparseMatrix<double> &A_sparse() {}

    void setA(const Eigen::Ref<const MatrixXd> &A) { A_ = A; }
    void setA(const SparseMatrix<double> &A) { A_sparse_ = A; }
    void setb(const VectorXd &b) { b_ = b; }

   protected:
    void gradientImpl(const Eigen::Ref<const VectorXd> &x,
                      Eigen::Ref<VectorXd> out) {
        out = 2.0 * A() * x + b();
    }

   private:
    MatrixXd A_;
    SparseMatrix<double> A_sparse_;

    VectorXd b_;
    double c_;
};

// class LeastSquaresCost : public QuadraticCost {
//    public:
//     LeastSquaresCost(const std::shared_ptr<LinearCost> &linear_cost) {}

//    protected:
//     void evalImpl(const Eigen::Ref<const VectorXd> &x, double &out) {
//         // linear_cost_->eval(x, out);
//         // setA(linear_cost_->a().transpose() * linear_cost_->a());
//     }

//    private:
// };

template <typename ValueType>
class cost_tpl : public evaluator::differentiable::scalar_tpl<ValueType> {
   public:
    using base = evaluator::differentiable::scalar_tpl<ValueType>;
    using typename base::dense_matrix_t;
    using typename base::dense_vector_t;
    using typename base::sparse_matrix_t;
    using typename base::sparse_vector_t;
    using typename base::value_t;

    typedef std::string string_t;

    using vector_buffer_t =
        evaluator::dense_sparse_buffer_tpl<dense_vector_t, sparse_vector_t>;
    using matrix_buffer_t =
        evaluator::dense_sparse_buffer_tpl<dense_matrix_t, sparse_matrix_t>;

    cost_tpl() = default;
    ~cost_tpl() = default;

    cost_tpl(const bopt_index &sz_in, const bopt_index &sz_p = 0)
        : base(sz_in, sz_p), name_("") {
        buffer_gradient_.dense =
            dense_vector_t::Zero(this->sz_gradient().second);
        buffer_hessian_.dense = dense_matrix_t::Zero(this->sz_hessian().first,
                                                     this->sz_hessian().first);
    }

    cost_tpl(const typename base::shared_ptr_t &ptr) : base(ptr), name_("") {
        buffer_gradient_.dense =
            dense_vector_t::Zero(ptr->sz_gradient().second);
        buffer_hessian_.dense = dense_matrix_t::Zero(ptr->sz_hessian().first,
                                                     ptr->sz_hessian().first);
    }

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
                        public evaluator::linear::scalar_tpl<ValueType> {
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
          evaluator::linear::scalar_tpl<ValueType>(sz_in, sz_p) {
        // Initialise buffers
        this->buffer_a().dense = dense_vector_t::Zero(this->sz_a().first);
    }

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

    evaluator::return_status eval_hessian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_matrix_t> out) override {
        out.setZero();
        return evaluator::return_status::Success;
    }

    evaluator::return_status eval_hessian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        sparse_matrix_t &out) override {
        for (int k = 0; k < out.outerSize(); ++k)
            for (typename sparse_matrix_t::InnerIterator it(out, k); it; ++it)
                it.valueRef() = 0.0;
        return evaluator::return_status::Success;
    }

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
                           public evaluator::quadratic::scalar_tpl<ValueType> {
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
          evaluator::quadratic::scalar_tpl<ValueType>(sz_in, sz_p) {
        // Initialise buffers
        this->buffer_A().dense =
            dense_matrix_t::Zero(this->sz_A().first, this->sz_A().second);
        this->buffer_b().dense = dense_vector_t::Zero(this->sz_b().first);
    }

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

        VLOG(10) << "A: " << A;
        VLOG(10) << "b: " << b;
        VLOG(10) << "x: " << x;

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

        VLOG(10) << "A: " << A;
        VLOG(10) << "b: " << b;
        VLOG(10) << "x: " << x;

        out = ValueType(2.0) * A * x + b;
        return evaluator::return_status::Success;
    }

    void get_hessian_sparsity_impl(sparse_matrix_t &out) const override {
        this->get_A_sparsity(out);
    }

    evaluator::return_status eval_hessian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_matrix_t> out) override {
        return this->eval_A(out);
    }

    evaluator::return_status eval_hessian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        sparse_matrix_t &out) override {
        return this->eval_A(out);
    }

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
        typename evaluator::linear::scalar_tpl<ValueType>::shared_ptr_t
            &expression)
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
    std::shared_ptr<evaluator::linear::scalar_tpl<ValueType>> expression_;
};

}  // namespace bopt
