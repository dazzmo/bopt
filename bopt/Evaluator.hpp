#pragma once

#include "bopt/Common.hpp"
#include "bopt/EvaluatorData.hpp"
#include "bopt/EvaluatorTraits.hpp"

namespace bopt {

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
template <typename ScalarType, int OutputSizeAtCompileTime = Eigen::Dynamic,
          SparsityType Sparsity = SparsityType::DENSE>
class EvaluatorTpl {
   public:
    static constexpr bool IsOutputScalar = (OutputSizeAtCompileTime == 1);

    using Scalar = ScalarType;
    using Traits =
        EvaluatorTraits<ScalarType, OutputSizeAtCompileTime, Sparsity>;

    using DenseVector = typename Traits::DenseVector;
    using InputVector = typename Traits::InputVectorType;
    using OutputType = typename Traits::OutputType;

    /// @brief The standard type of data to be used for the evaluation functions
    using Data =
        EvaluatorDataTpl<ScalarType, OutputSizeAtCompileTime, Sparsity>;

    using ParameterVector = typename MathTypes<Real>::VectorX;

   public:
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

    friend std::ostream &operator<<(std::ostream &os, const EvaluatorTpl &e) {
        os << "Evaluator\n";
        os << "Description: " << e.getDescription() << '\n';
        os << "Input Size: " << e.inputSize() << '\n';
        os << "Output Size: " << e.outputSize();
        return os;
    }

    /**
     * @brief Sets the sparsity of the provided data, if applicable
     *
     */
    void setupDataSparsity(Data &data) const { setupDataSparsityImpl(data); }

    /**
     * @brief Evaluates the expression y = fₚ(x) using variables x and
     * parameters p (set through \ref EvaluatorTpl::setParameters()).
     *
     * @param x The input vector (inputSize() x 1)
     * @param data
     */
    void eval(const Eigen::Ref<const InputVector> &x, Data &data) const {
        evalImpl(x, data);
    }

    template <bool B = IsOutputScalar,
              typename std::enable_if<B, int>::type = 0>
    void evalGradients(const Eigen::Ref<const InputVector> &x, Data &data,
                       const GradientEvaluationFlags &flags =
                           GradientEvaluationFlags()) const {
        evalGradientsImpl(x, data, flags);
    }

    /**
     * @brief Computes the jacobians of the expression f.
     *
     * @param x
     * @param data
     * @param flags Flags to indicate which Jacobians to compute
     */
    template <bool B = IsOutputScalar,
              typename std::enable_if<!B, int>::type = 0>
    void evalJacobians(const Eigen::Ref<const InputVector> &x, Data &data,
                       const JacobianEvaluationFlags &flags =
                           JacobianEvaluationFlags()) const {
        evalJacobiansImpl(x, data, flags);
    }

    /**
     * @brief Computes the lower-triangular hessians of the expression f.
     *
     * @param x
     * @param data
     * @param flags Flags to indicate which Hessians to compute
     */
    template <bool B = IsOutputScalar,
              typename std::enable_if<B, int>::type = 0>
    void evalHessians(
        const Eigen::Ref<const InputVector> &x, Data &data,
        const HessianEvaluationFlags &flags = HessianEvaluationFlags()) const {
        evalHessiansImpl(x, data, flags);
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
    template <bool B = IsOutputScalar,
              typename std::enable_if<!B, int>::type = 0>
    void evalHessians(
        const Eigen::Ref<const InputVector> &x,
        const Eigen::Ref<const InputVector> &lambda, Data &data,
        const HessianEvaluationFlags &flags = HessianEvaluationFlags()) const {
        evalHessiansImpl(x, lambda, data, flags);
    }

   protected:
    template <bool B = IsOutputScalar,
              typename std::enable_if<B, int>::type = 0>
    EvaluatorTpl(const Index &n_in, const String &description = "")
        : n_in_(n_in),
          dim_tangent_space_(n_in),
          n_out_(1),
          n_parameters_(0),
          parameters_(DenseVector::Zero(n_parameters_)),
          description_(description) {}

    template <bool B = IsOutputScalar,
              typename std::enable_if<!B, int>::type = 0>
    EvaluatorTpl(const Index &n_in, const Index &n_out,
                 const String &description = "")
        : n_in_(n_in),
          dim_tangent_space_(n_in),
          n_out_(n_out),
          n_parameters_(0),
          parameters_(DenseVector::Zero(n_parameters_)),
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

    /**
     * @brief Implementation of the evaluator
     *
     * @param x
     * @param out
     */
    virtual void evalImpl(const Eigen::Ref<const InputVector> &x,
                          Data &data) const {}

    /**
     * \copydoc EvaluatorTpl::evalGradients(const Eigen::Ref<const
     * VectorX<Scalar>>, Data &)
     *
     */
    virtual void evalGradientsImpl(const Eigen::Ref<const InputVector> &x,
                                   Data &data,
                                   const GradientEvaluationFlags &flags) const {
    }

    /**
     * \copydoc EvaluatorTpl::evalJacobians(const Eigen::Ref<const
     * VectorX<Scalar>>, Data &)
     *
     */
    virtual void evalJacobiansImpl(const Eigen::Ref<const InputVector> &x,
                                   Data &data,
                                   const JacobianEvaluationFlags &flags) const {
    }

    /**
     * \copydoc EvaluatorTpl::evalHessians(const Eigen::Ref<const
     * VectorX<Scalar>>, const Eigen::Ref<const InputVector>, Data
     * &)
     *
     */
    virtual void evalHessiansImpl(const Eigen::Ref<const InputVector> &x,
                                  Data &data,
                                  const HessianEvaluationFlags &flags) const {}

    /**
     * \copydoc EvaluatorTpl::evalHessians(const Eigen::Ref<const
     * VectorX<Scalar>>, const Eigen::Ref<const InputVector>, Data
     * &)
     *
     */
    virtual void evalHessiansImpl(const Eigen::Ref<const InputVector> &x,
                                  const Eigen::Ref<const InputVector> &lambda,
                                  Data &data,
                                  const HessianEvaluationFlags &flags) const {}

    virtual void setupDataSparsityImpl(Data &data) const {}

   private:
    /// @brief Dimension of the input vector
    Index n_in_;
    Index n_out_;
    /// @brief Dimension of the input tangent space
    Index dim_tangent_space_;
    Index n_parameters_{0};

    ParameterVector parameters_;
    String description_;
};

template <typename PolynomialData>
class PolynomialEvaluator {
   public:
    PolynomialEvaluator() = default;
    ~PolynomialEvaluator() = default;

    void evalCoefficients(PolynomialData &data) const {
        evalCoefficientsImpl(data);
    }

    void setupDataSparsity(PolynomialData &data) const {
        setupDataSparsityImpl(data);
    }

   protected:
    virtual void evalCoefficientsImpl(PolynomialData &data) const {}
    virtual void setupDataSparsityImpl(PolynomialData &data) const {}
};

}  // namespace bopt