#pragma once

#include "bopt/Common.hpp"
#include "bopt/EvaluatorData.hpp"
#include "bopt/EvaluatorTraits.hpp"

namespace bopt {

/**
 * @brief Evaluator class related the evaluation of a function y = fₚ(x)
 *
 * @tparam Scalar
 */
template <typename EvaluatorTraits, int OutputSize = Eigen::Dynamic>
class EvaluatorTpl {
   public:
    using Scalar = typename EvaluatorTraits::Scalar;

    using DenseVector = typename EvaluatorTraits::DenseVector;
    using InputVector = typename EvaluatorTraits::InputVector;
    using InputVectorConstRef = typename EvaluatorTraits::InputVectorConstRef;

    /// @brief The standard type of data to be used for the evaluation functions
    using Data = EvaluatorDataTpl<EvaluatorTraits, OutputSize>;

    /**
     * @brief Dimension of the input variable vector, commonly denoted as x.
     *
     * @return const Size&
     */
    Size numInputs() const { return n_in_; }

    /**
     * @brief Dimension of the output vector y.
     *
     * @return const Size&
     */
    Size numOutputs() const { return n_out_; }

    /**
     * @brief Dimension of the input space, this is equal to numInputs().
     *
     * @return const Size&
     */
    Size dimInputSpace() const { return n_in_; }

    /**
     * @brief Dimension of the tangent space of input, typically
     * this is equal to numInputs().
     *
     * @return const Size&
     */
    Size dimInputTangentSpace() const { return dim_tangent_space_; }

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

    const DenseVector &getParameters() const { return parameters_; }

    /**
     * @brief Set the parameter vector p (numParameters() x 1).
     *
     * @param p
     */
    void setParameters(const InputVectorConstRef &p) {
        assert(p.size() == numParameters());
        parameters_ = p;
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
     * @param x The input vector (numInputs() x 1)
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
     * @param compute_x Compute ∂y/∂x
     * @param compute_p Compute ∂y/∂p
     */
    void evalJacobians(const InputVectorConstRef &x, Data &data,
                       bool compute_x = true, bool compute_p = false) const {
        evalJacobiansImpl(x, data, compute_x, compute_p);
    }

    /**
     * @brief Computes the lower-triangular hessians of the vector-product of
     * the expression λᵀf.
     *
     * @param x
     * @param lambda
     * @param data
     * @param compute_xx Compute ∂²(λᵀf)/∂x²
     * @param compute_xp Compute ∂²(λᵀf)/∂x∂p
     * @param compute_pp Compute ∂²(λᵀf)/∂p²
     */
    void evalHessians(const InputVectorConstRef &x,
                      const InputVectorConstRef &lambda, Data &data,
                      bool compute_xx = true, bool compute_xp = false,
                      bool compute_pp = false) const {
        evalHessiansImpl(x, lambda, data, compute_xx, compute_xp, compute_pp);
    }

   protected:
    EvaluatorTpl(const Index &n_in, const Index &n_out,
                 const String &description = "")
        : n_in_(n_in),
          dim_tangent_space_(n_in),
          n_out_(n_out),
          n_parameters_(0),
          parameters_(InputVector::Zero(0)),
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
    void setParameterDimension(const Index &dim) {
        n_parameters_ = dim;
        parameters_ = InputVector::Zero(dim);
    }

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
                                   bool compute_x, bool compute_p) const {}

    /**
     * \copydoc EvaluatorTpl::evalHessians(const Eigen::Ref<const
     * VectorX<Scalar>>, const InputVectorConstRef, Data
     * &)
     *
     */
    virtual void evalHessiansImpl(const InputVectorConstRef &x,
                                  const InputVectorConstRef &lambda, Data &data,
                                  bool compute_xx, bool compute_xp,
                                  bool compute_pp) const {}

   private:
    /// @brief Dimension of the input vector
    Index n_in_;
    Index n_out_;
    /// @brief Dimension of the input tangent space
    Index dim_tangent_space_;
    Index n_parameters_;

    DenseVector parameters_;
    String description_;
};

template <typename EvaluatorTraits, int OutputSize>
std::ostream &operator<<(std::ostream &os,
                         const EvaluatorTpl<EvaluatorTraits, OutputSize> &e) {
    os << "Evaluator\n";
    os << "Description: " << e.getDescription() << '\n';
    os << "Input Size: " << e.numInputs() << '\n';
    os << "Output Size: " << e.numOutputs();
    return os;
}

template <typename Scalar, int OutputSize>
using DenseEvaluatorTpl =
    EvaluatorTpl<DenseEvaluatorTraits<Scalar>, OutputSize>;
template <int OutputSize>
using DenseEvaluator = DenseEvaluatorTpl<Real, OutputSize>;

template <typename Scalar, int OutputSize>
using SparseEvaluatorTpl =
    EvaluatorTpl<SparseEvaluatorTraits<Scalar>, OutputSize>;
template <int OutputSize>
using SparseEvaluator = SparseEvaluatorTpl<Real, OutputSize>;

/**
 * @brief An evaluator of an expression that can be represented in polynomial
 * form.
 *
 * @tparam DataType The data type, where the coefficients can be stored and
 * evaluated through evalCoefficients()
 * @tparam EvaluatorTraits Traits of the underlying evaluator
 * @tparam OutputSize
 */
template <typename DataType, typename EvaluatorTraits,
          int OutputSize = Eigen::Dynamic>
class PolynomialEvaluatorTpl
    : public EvaluatorTpl<EvaluatorTraits, OutputSize> {
   public:
    using Base = EvaluatorTpl<EvaluatorTraits, OutputSize>;
    /// @brief Data type for the polynomial data for computation of the
    /// coefficients
    using Data = DataType;
    /// @brief Evaluator data for all other evaluator-type functions
    using EvaluatorData = typename Base::Data;

    void evalCoefficients(Data &data) const { evalCoefficientsImpl(data); }

   protected:
    virtual void evalCoefficientsImpl(Data &data) const {}
};

template <typename EvaluatorTraits, int OutputSize>
class LinearEvaluatorTpl
    : public PolynomialEvaluatorTpl<
          LinearEvaluatorDataTpl<EvaluatorTraits, OutputSize>, EvaluatorTraits,
          OutputSize> {};

/**
 * @brief Quadratic evaluator for scalar expressions of the form
 *
 * @tparam EvaluatorTraits
 */
template <typename EvaluatorTraits>
class QuadraticEvaluatorTpl
    : public PolynomialEvaluatorTpl<QuadraticEvaluatorDataTpl<EvaluatorTraits>,
                                    EvaluatorTraits, 1> {};

}  // namespace bopt