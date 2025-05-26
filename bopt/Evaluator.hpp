#pragma once

#include "bopt/Common.hpp"
#include "bopt/EvaluatorData.hpp"
#include "bopt/EvaluatorTraits.hpp"

namespace bopt {

template <int OutputSizeAtCompileTime>
class EvaluatorBase {
    using ParameterVector = typename MathTypes<Real>::VectorX;

   public:
    /// @brief Whether the output of the evaluator is a scalar
    static constexpr bool IsOutputScalar = OutputSizeAtCompileTime == 1;

    /**
     * @brief Dimension of the input variable vector, commonly denoted as x.
     *
     * @return const Size&
     */
    Size inputSize() const { return n_in_; }

    /**
     * @brief Dimension of the output vector y.
     *
     * @return const Size&
     */
    Size outputSize() const { return n_out_; }

    /**
     * @brief Dimension of the input space, this is equal to inputSize().
     *
     * @return const Size&
     */
    Size dimInputSpace() const { return n_in_; }

    /**
     * @brief Dimension of the tangent space of input, typically
     * this is equal to inputSize().
     *
     * @return const Size&
     */
    Size dimInputTangentSpace() const { return dim_tangent_space_; }

    /**
     * @brief Dimension of the input space, this is equal to inputSize().
     *
     * @return const Size&
     */
    Size dimOutputSpace() const { return n_out_; }

    /**
     * @brief Dimension of the parameter vector p.
     *
     * @return const Size&
     */
    Size numParameters() const { return n_parameters_; }

    const String &getDescription() const { return description_; }

    void setDescription(const String &description) {
        description_ = description;
    }

    const ParameterVector &getParameters() const { return parameters_; }

    /**
     * @brief Set the parameter vector p (numParameters() x 1).
     *
     * @param p
     */
    void setParameters(const Eigen::Ref<const ParameterVector> &p) {
        assert(p.size() == numParameters());
        parameters_ = p;
    }

    friend std::ostream &operator<<(std::ostream &os, const EvaluatorBase &e) {
        os << "Evaluator\n";
        os << "Description: " << e.getDescription() << '\n';
        os << "Input Size: " << e.inputSize() << '\n';
        os << "Output Size: " << e.outputSize();
        return os;
    }

   protected:
    EvaluatorBase(const Index &n_in, const Index &n_out,
                  const String &description = "")
        : n_in_(n_in),
          dim_tangent_space_(n_in),
          n_out_(n_out),
          n_parameters_(0),
          parameters_(ParameterVector::Zero(0)),
          description_(description) {}

    /**
     * @brief Sets the dimension of the tangent space for the input variables.
     *
     * @param dim Dimension of the vector
     */
    void setTangentSpaceDimension(const Index &dim) {
        dim_tangent_space_ = dim;
    }

    /**
     * @brief Sets the dimension of the evaluator parameter vector.
     *
     * @param dim Dimension of the vector
     */
    void setNumParameters(const Index &dim) {
        n_parameters_ = dim;
        parameters_ = ParameterVector::Zero(dim);
    }

   private:
    /// @brief Dimension of the input vector
    Index n_in_;
    Index n_out_;
    /// @brief Dimension of the input tangent space
    Index dim_tangent_space_;
    Index n_parameters_;

    ParameterVector parameters_;
    String description_;
};

struct GradientEvaluationFlags {
    GradientEvaluationFlags() : compute_x(true), compute_p(false) {}
    GradientEvaluationFlags(bool compute_x, bool compute_p)
        : compute_x(compute_x), compute_p(compute_p) {}

    /// @brief Compute ∂f/∂x
    bool compute_x;
    /// @brief Compute ∂f/∂p
    bool compute_p;
};

struct JacobianEvaluationFlags {
    JacobianEvaluationFlags() : compute_x(true), compute_p(false) {}
    JacobianEvaluationFlags(bool compute_x, bool compute_p = false)
        : compute_x(compute_x), compute_p(compute_p) {}

    /// @brief Compute ∂f/∂x
    bool compute_x;
    /// @brief Compute ∂f/∂p
    bool compute_p;
};

struct HessianEvaluationFlags {
    HessianEvaluationFlags()
        : compute_xx(true), compute_xp(false), compute_pp(false) {}
    HessianEvaluationFlags(bool compute_xx, bool compute_xp = false,
                           bool compute_pp = false)
        : compute_xx(compute_xx),
          compute_xp(compute_xp),
          compute_pp(compute_pp) {}
    /// @brief Compute ∂²(λᵀf)/∂x²
    bool compute_xx;
    /// @brief Compute ∂²(λᵀf)/∂x∂p
    bool compute_xp;
    /// @brief Compute ∂²(λᵀf)/∂p²
    bool compute_pp;
};

/**
 * @brief Evaluator class related the evaluation of a function y = fₚ(x)
 *
 * @tparam Scalar
 */
template <typename EvaluatorTraits, int OutputSizeAtCompileTime>
class EvaluatorTpl : public EvaluatorBase<OutputSizeAtCompileTime> {
   public:
    using Scalar = typename EvaluatorTraits::Scalar;
    using Traits = EvaluatorTraits;
    using DenseVector = typename EvaluatorTraits::DenseVector;
    using InputVector = typename EvaluatorTraits::InputVector;
    using InputVectorConstRef = typename EvaluatorTraits::InputVectorConstRef;

    /// @brief The standard type of data to be used for the evaluation functions
    using Data = EvaluatorDataTpl<EvaluatorTraits, OutputSizeAtCompileTime>;

    std::shared_ptr<Data> createData() const {
        return std::make_shared<Data>(*this);
    }

    /**
     * @brief Set the sparsity of any entries within the provided data
     * structure.
     *
     * @param data
     */
    void setDataSparsity(Data &data) const { this->setDataSparsityImpl(data); }

    /**
     * @brief Evaluates the expression y = fₚ(x) using variables x and
     * parameters p (set through \ref EvaluatorTpl::setParameters()).
     *
     * @param x The input vector (inputSize() x 1)
     * @param data
     */
    void eval(const InputVectorConstRef &x, Data &data) const {
        evalImpl(x, data);
    }

    /**
     * @brief Computes the jacobians of the expression f.
     *
     * @param x
     * @param data
     * @param flags Flags to indicate which Jacobians to compute
     */
    void evalJacobians(const InputVectorConstRef &x, Data &data,
                       const JacobianEvaluationFlags &flags =
                           JacobianEvaluationFlags()) const {
        evalJacobiansImpl(x, data, flags);
    }

    /**
     * @brief Computes the lower-triangular hessians of the vector-product of
     * the expression λᵀf.
     *
     * @param x
     * @param lambda
     * @param data
     * @param flags Flags to indicate which Hessians to compute
     */
    void evalHessians(
        const InputVectorConstRef &x, const InputVectorConstRef &lambda,
        Data &data,
        const HessianEvaluationFlags &flags = HessianEvaluationFlags()) const {
        evalHessiansImpl(x, lambda, data, flags);
    }

   protected:
    EvaluatorTpl(const Index &n_in, const Index &n_out,
                 const String &description = "")
        : EvaluatorBase<OutputSizeAtCompileTime>(n_in, n_out, description) {}

    virtual void setDataSparsityImpl(Data &data) const {}

    /**
     * @brief Implementation of the evaluator
     *
     * @param x
     * @param out
     */
    virtual void evalImpl(const InputVectorConstRef &x, Data &data) const {}

    /**
     * \copydoc EvaluatorTpl::evalJacobians(const Eigen::Ref<const
     * VectorX<Scalar>>, Data &)
     *
     */
    virtual void evalJacobiansImpl(const InputVectorConstRef &x, Data &data,
                                   const JacobianEvaluationFlags &flags) const {
    }

    /**
     * \copydoc EvaluatorTpl::evalHessians(const Eigen::Ref<const
     * VectorX<Scalar>>, const InputVectorConstRef, Data
     * &)
     *
     */
    virtual void evalHessiansImpl(const InputVectorConstRef &x,
                                  const InputVectorConstRef &lambda, Data &data,
                                  const HessianEvaluationFlags &flags) const {}
};

/**
 * @brief Evaluator class specialisation for scalar functions y = fₚ(x)
 *
 * @tparam Scalar
 */
template <typename EvaluatorTraits>
class EvaluatorTpl<EvaluatorTraits, 1> : public EvaluatorBase<1> {
   public:
    using Scalar = typename EvaluatorTraits::Scalar;
    using Traits = EvaluatorTraits;
    using DenseVector = typename EvaluatorTraits::DenseVector;
    using InputVector = typename EvaluatorTraits::InputVector;
    using InputVectorConstRef = typename EvaluatorTraits::InputVectorConstRef;

    /// @brief The standard type of data to be used for the evaluation functions
    using Data = EvaluatorDataTpl<EvaluatorTraits, 1>;

    std::shared_ptr<Data> createData() const {
        return std::make_shared<Data>(*this);
    }

    /**
     * @brief Set the sparsity of any entries within the provided data
     * structure.
     *
     * @param data
     */
    void setDataSparsity(Data &data) const { this->setDataSparsityImpl(data); }

    /**
     * @brief Evaluates the expression y = fₚ(x) using variables x and
     * parameters p (set through \ref EvaluatorTpl::setParameters()).
     *
     * @param x The input vector (inputSize() x 1)
     * @param data
     */
    void eval(const InputVectorConstRef &x, Data &data) const {
        evalImpl(x, data);
    }

    /**
     * @brief Computes the gradients of the expression f.
     *
     * @param x
     * @param data
     * @param flags
     */
    void evalGradients(const InputVectorConstRef &x, Data &data,
                       const GradientEvaluationFlags &flags =
                           GradientEvaluationFlags()) const {
        evalGradientsImpl(x, data, flags);
    }

    /**
     * @brief Computes the lower-triangular hessians of the vector-product of
     * the expression f.
     *
     * @param x
     * @param lambda
     * @param data
     * @param compute_xx Compute ∂²f/∂x²
     * @param compute_xp Compute ∂²f/∂x∂p
     * @param compute_pp Compute ∂²f/∂p²
     */
    void evalHessians(
        const InputVectorConstRef &x, Data &data,
        const HessianEvaluationFlags &flags = HessianEvaluationFlags()) const {
        evalHessiansImpl(x, data, flags);
    }

   protected:
    EvaluatorTpl(const Index &n_in, const String &description = "")
        : EvaluatorBase(n_in, 1, description) {}

    virtual void setDataSparsityImpl(Data &data) const {}

    /**
     * @brief Implementation of the evaluator
     *
     * @param x
     * @param out
     */
    virtual void evalImpl(const InputVectorConstRef &x, Data &data) const {}

    /**
     * \copydoc EvaluatorTpl::evalGradients(const Eigen::Ref<const
     * VectorX<Scalar>>, Data &)
     *
     */
    virtual void evalGradientsImpl(const InputVectorConstRef &x, Data &data,
                                   const GradientEvaluationFlags &flags) const {
    }

    /**
     * \copydoc EvaluatorTpl::evalHessians(const Eigen::Ref<const
     * VectorX<Scalar>>, const InputVectorConstRef, Data
     * &)
     *
     */
    virtual void evalHessiansImpl(const InputVectorConstRef &x, Data &data,
                                  const HessianEvaluationFlags &flags) const {}
};

template <typename EvaluatorTraits, int OutputSizeAtCompileTime>
class LinearEvaluatorTpl
    : public EvaluatorTpl<EvaluatorTraits, OutputSizeAtCompileTime> {
    using Base = EvaluatorTpl<EvaluatorTraits, OutputSizeAtCompileTime>;

   public:
    using Data =
        LinearEvaluatorDataTpl<EvaluatorTraits, OutputSizeAtCompileTime>;

    // Constructor for vector-valued evaluators
    template <int S = OutputSizeAtCompileTime,
              typename std::enable_if_t<(S != 1), int> = 0>
    LinearEvaluatorTpl(const Size &n_in, const Size &n_out,
                       const String &description = "")
        : Base(n_in, n_out, description) {}

    // Constructor for scalar-valued evaluators
    template <int S = OutputSizeAtCompileTime,
              typename std::enable_if_t<(S == 1), int> = 0>
    LinearEvaluatorTpl(const Size &n_in, const String &description = "")
        : Base(n_in, description) {}

    std::shared_ptr<Data> createData() const {
        return std::make_shared<Data>(*this);
    }

    void evalCoefficients(Data &data) const { evalCoefficientsImpl(data); }
    void setDataSparsity(Data &data) const { this->setDataSparsityImpl(data); }

   protected:
    virtual void evalCoefficientsImpl(Data &data) const {}
    virtual void setDataSparsityImpl(Data &data) const {}
};

template <typename EvaluatorTraits>
class QuadraticEvaluatorTpl : public EvaluatorTpl<EvaluatorTraits, 1> {
    using Base = EvaluatorTpl<EvaluatorTraits, 1>;

   public:
    using Data = QuadraticEvaluatorDataTpl<EvaluatorTraits>;

    QuadraticEvaluatorTpl(const Size &n_in, const String &description = "")
        : Base(n_in, description) {}

    std::shared_ptr<Data> createData() const {
        return std::make_shared<Data>(*this);
    }

    void evalCoefficients(Data &data) const { evalCoefficientsImpl(data); }
    void setDataSparsity(Data &data) const { this->setDataSparsityImpl(data); }

   protected:
    virtual void evalCoefficientsImpl(Data &data) const {}
    virtual void setDataSparsityImpl(Data &data) const {}
};

namespace internal {

template <typename EvaluatorType>
class EvaluatorWrapper {
   public:
    using Scalar = typename EvaluatorType::Scalar;

    using DenseVector = typename EvaluatorType::DenseVector;
    using InputVector = typename EvaluatorType::InputVector;
    using InputVectorConstRef = typename EvaluatorType::InputVectorConstRef;

    using Evaluator = EvaluatorType;
    using Data = typename EvaluatorType::Data;

    /**
     * @brief Returns the evaluator for the function
     *
     * @return Evaluator&
     */
    Evaluator &getEvaluator() const { return *evaluator_; }

    /**
     * @brief Set an evaluator for the of the constraint.
     *
     * @param evaluator
     */
    void setEvaluator(const std::shared_ptr<Evaluator> &evaluator) {
        evaluator_ = evaluator;
    }

    Size inputSize() const { return getEvaluator().inputSize(); }
    Size outputSize() const { return getEvaluator().outputSize(); }
    Size dimInputSpace() const { return getEvaluator().dimInputSpace(); }
    Size dimInputTangentSpace() const {
        return getEvaluator().dimInputTangentSpace();
    }
    Size numParameters() const { return getEvaluator().numParameters(); }

    std::shared_ptr<Data> createData() const {
        return getEvaluator().createData();
    }

    void eval(const InputVectorConstRef &x, Data &data) const {
        evaluator_->eval(x, data);
    }

    void evalGradients(const InputVectorConstRef &x, Data &data,
                       const GradientEvaluationFlags &flags =
                           GradientEvaluationFlags()) const {
        static_assert(EvaluatorType::IsOutputScalar,
                      "You are calling evalGradients() on a vector evaluator, "
                      "use evalJacobians() instead");
        this->getEvaluator().evalGradients(x, data, flags);
    }

    void evalJacobians(const InputVectorConstRef &x, Data &data,
                       const JacobianEvaluationFlags &flags =
                           JacobianEvaluationFlags()) const {
        static_assert(!EvaluatorType::IsOutputScalar,
                      "You are calling evalJacobians() on a scalar evaluator, "
                      "use evalGradients() instead");
        this->getEvaluator().evalJacobians(x, data, flags);
    }

    void evalHessians(const InputVectorConstRef &x,
                      const InputVectorConstRef &lambda, Data &data,
                      const HessianEvaluationFlags &flags,
                      HessianEvaluationFlags()) const {
        static_assert(
            EvaluatorType::IsOutputScalar,
            "You are calling evalHessians(x, lambda, data, flags) on a "
            "scalar evaluator, "
            "use evalHessians(x, data, flags) instead");
        this->getEvaluator().evalHessians(x, lambda, data, flags);
    }

    void evalHessians(const InputVectorConstRef &x, Data &data,
                      const HessianEvaluationFlags &flags,
                      HessianEvaluationFlags()) const {
        static_assert(EvaluatorType::IsOutputScalar,
                      "You are calling evalHessians(x, data, flags) on a "
                      "vector evaluator, "
                      "use evalHessians(x, lambda, data, flags) instead");
        this->getEvaluator().evalHessians(x, data, flags);
    }

    EvaluatorWrapper(const std::shared_ptr<Evaluator> &evaluator)
        : evaluator_(evaluator) {}

   protected:
   private:
    /// @brief Shared pointer to the evaluator the wrapper is associated with
    std::shared_ptr<Evaluator> evaluator_{nullptr};
};

}  // namespace internal

}  // namespace bopt