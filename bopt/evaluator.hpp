#pragma once

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <optional>
#include <unsupported/Eigen/AutoDiff>

#include "bopt/assert.hpp"
#include "bopt/common.hpp"
// #include "bopt/evaluator/base.hpp"
// #include "bopt/evaluator/differentiable.hpp"
// #include "bopt/evaluator/linear.hpp"
// #include "bopt/evaluator/quadratic.hpp"

namespace bopt {

// Forward declarations
template <typename Scalar>
struct EvaluatorDataTpl;

template <typename ScalarType>
struct FunctionTraits {
    using Scalar = ScalarType;

    using InputVector = Eigen::VectorX<Scalar>;
    using InputVectorConstRef = Eigen::Ref<const InputVector>;
};

template <typename ScalarType>
struct SparseFunctionTraits : public FunctionTraits<ScalarType> {
    using Base = FunctionTraits<ScalarType>;

    using Scalar = typename Base::Scalar;
    using InputVector = typename Base::InputVector;
    using InputVectorConstRef = typename Base::InputVectorConstRef;

    using OutputVector = Eigen::SparseVector<Scalar>;
    using OutputMatrix = Eigen::SparseMatrix<Scalar>;

    static constexpr std::string_view type = "Sparse";
};

template <typename ScalarType>
struct DenseFunctionTraits : public FunctionTraits<ScalarType> {
    using Base = FunctionTraits<ScalarType>;

    using Scalar = typename Base::Scalar;
    using InputVector = typename Base::InputVector;
    using InputVectorConstRef = typename Base::InputVectorConstRef;

    using OutputVector = Eigen::VectorX<Scalar>;
    using OutputMatrix = Eigen::MatrixX<Scalar>;

    static constexpr std::string_view type = "Dense";
};

/**
 * @brief Evaluator class related the evaluation of a function y = fₚ(x)
 *
 * @tparam Scalar
 */
template <typename FunctionTraits>
class EvaluatorTpl {
   public:
    using Scalar = typename FunctionTraits::Scalar;

    using InputVector = typename FunctionTraits::InputVector;
    using InputVectorConstRef = typename FunctionTraits::InputVectorConstRef;

    using Vector = typename FunctionTraits::OutputVector;
    using Matrix = typename FunctionTraits::OutputMatrix;

    /// @brief The standard type of data to be used for the evaluation functions
    using Data = EvaluatorDataTpl<FunctionTraits>;

    EvaluatorTpl(const std::shared_ptr<EvaluatorTpl<FunctionTraits>> &ptr)
        : ptr_(ptr),
          dim_input_(ptr->getInputDimension()),
          dim_tangent_space_(ptr->getInputTangentSpaceDimension()),
          dim_output_(ptr->getOutputDimension()),
          num_parameters_(ptr->getNumberOfParameters()),
          parameters_(InputVector::Zero(ptr->getNumberOfParameters())),
          description_(ptr->description()) {}

    std::shared_ptr<Data> createData() const {
        auto ptr = std::shared_ptr<Data>(this->createDataImpl());
        setDataSparsityImpl(*ptr);
    };

    /**
     * @brief Evaluates the expression y = fₚ(x) using variables x and
     * parameters p (set through \ref EvaluatorTpl::setParameters()).
     *
     * @param x The input vector (getInputDimension() x 1)
     * @param data
     */
    void eval(const InputVectorConstRef &x, Data &data) const {
        assert(x.rows() == getInputDimension());
        assert(data.y.rows() == getOutputDimension());
        // Ensure vector is valid
        // CHECK(x.allFinite() && !x.hasNaN() && x.size());
        evalImpl(x, data);
        // Ensure output is valid
        // CHECK(data.y.allFinite() && !data.y.hasNaN() && data.y.size());
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
        assert(x.rows() == getInputDimension());
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
        assert(x.rows() == getInputDimension());
        assert(lambda.rows() == getOutputDimension());
        evalHessiansImpl(x, lambda, data, compute_xx, compute_xp, compute_pp);
    }

    /**
     * @brief Dimension of the input variable vector, commonly denoted as x.
     *
     * @return const Index&
     */
    const Index &getInputDimension() const { return dim_input_; }

    /**
     * @brief Dimension of the tangent space for the input vector, typically
     * this is equal to getInputDimension().
     *
     * @return const Index&
     */
    const Index &getInputTangentSpaceDimension() const {
        return dim_tangent_space_;
    }

    /**
     * @brief Dimension of the output vector y.
     *
     * @return const Index&
     */
    const Index &getOutputDimension() const { return dim_output_; }

    /**
     * @brief Dimension of the parameter vector p.
     *
     * @return const Index&
     */
    const Index &getNumberOfParameters() const { return num_parameters_; }

    const std::string &description() const { return description_; }

    void setDescription(const std::string &description) {
        description_ = description;
    }

    const InputVector &parameters() const { return parameters_; }

    /**
     * @brief Set the parameter vector p (getNumberOfParameters() x 1).
     *
     * @param p
     */
    void setParameters(const InputVectorConstRef &p) {
        assert(p.size() == getNumberOfParameters());
        parameters_ = p;
    }

    /**
     * @brief Set the sparsity of any entries within the provided data
     * structure.
     *
     * @param data
     */
    void setDataSparsity(Data &data) const { this->setDataSparsityImpl(data); }

   protected:
    EvaluatorTpl(const Index &n_inputs, const Index &n_outputs,
                 const std::string &description = "")
        : ptr_(nullptr),
          dim_input_(n_inputs),
          dim_tangent_space_(n_inputs),
          dim_output_(n_outputs),
          num_parameters_(0),
          parameters_(InputVector::Zero(0)),
          description_(description) {}

    virtual Data *createDataImpl() const { return new Data(*this); }

    /**
     * @brief Sets the dimension of the evaluator output.
     *
     * @param dim Dimension of the vector
     */
    void setOutputDimension(const Index &dim) { dim_output_ = dim; }

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
        num_parameters_ = dim;
        parameters_ = InputVector::Zero(dim);
    }

    virtual void setDataSparsityImpl(Data &data) const {
        if (ptr_) ptr_->setDataSparsity(data);
    }

    /**
     * @brief Implementation of the evaluator
     *
     * @param x
     * @param out
     */
    virtual void evalImpl(const InputVectorConstRef &x, Data &data) const {
        if (ptr_) ptr_->eval(x, data);
    }

    /**
     * \copydoc EvaluatorTpl::evalJacobians(const Eigen::Ref<const
     * VectorX<Scalar>>, Data &)
     *
     */
    virtual void evalJacobiansImpl(const InputVectorConstRef &x, Data &data,
                                   bool compute_x, bool compute_p) const {
        if (ptr_) ptr_->evalJacobians(x, data, compute_x, compute_p);
    }

    /**
     * \copydoc EvaluatorTpl::evalHessians(const Eigen::Ref<const
     * VectorX<Scalar>>, const InputVectorConstRef, Data
     * &)
     *
     */
    virtual void evalHessiansImpl(const InputVectorConstRef &x,
                                  const InputVectorConstRef &lambda, Data &data,
                                  bool compute_xx, bool compute_xp,
                                  bool compute_pp) const {
        if (ptr_)
            ptr_->evalHessians(x, lambda, data, compute_xx, compute_xp,
                               compute_pp);
    }

   private:
    /// Shared pointer for evaluator instance (if made from a copy)
    std::shared_ptr<EvaluatorTpl<FunctionTraits>> ptr_;

    /// @brief Dimension of the input vector
    Index dim_input_;
    Index dim_tangent_space_;
    Index dim_output_;
    Index num_parameters_;

    InputVector parameters_;
    std::string description_;
};

template <typename FunctionTraits>
std::ostream &operator<<(std::ostream &os,
                         const EvaluatorTpl<FunctionTraits> &e) {
    os << "Evaluator\n";
    os << "description: " << e.description() << '\n';
    os << "input dim: " << e.getInputDimension() << '\n';
    os << "output dim: " << e.getOutputDimension();
    return os;
}

template <typename Scalar>
using DenseEvaluatorTpl = EvaluatorTpl<DenseFunctionTraits<Scalar>>;

template <typename Scalar>
using SparseEvaluatorTpl = EvaluatorTpl<SparseFunctionTraits<Scalar>>;

using DenseEvaluator = DenseEvaluatorTpl<Real>;
using SparseEvaluator = SparseEvaluatorTpl<Real>;

template <typename InputTraitType>
struct EvaluatorDataTpl {
    using DenseVector = typename InputTraitType::InputVector;
    using Vector = typename InputTraitType::OutputVector;
    using Matrix = typename InputTraitType::OutputMatrix;

    EvaluatorDataTpl(const EvaluatorTpl<InputTraitType> &e) {
        y = DenseVector::Zero(e.getOutputDimension());

        if constexpr (InputTraitType::type == "Sparse") {
            // Sparse: allocate sparse objects properly
            Jx.resize(e.getOutputDimension(),
                      e.getInputTangentSpaceDimension());
            Jp.resize(e.getOutputDimension(), e.getNumberOfParameters());
            Hxx.resize(e.getInputTangentSpaceDimension(),
                       e.getInputTangentSpaceDimension());
            Hxp.resize(e.getInputTangentSpaceDimension(),
                       e.getNumberOfParameters());
            Hpp.resize(e.getNumberOfParameters(), e.getNumberOfParameters());
            // Optionally set values to zero explicitly if needed
        } else {
            // Dense
            Jx = Matrix::Zero(e.getOutputDimension(),
                              e.getInputTangentSpaceDimension());
            Jp =
                Matrix::Zero(e.getOutputDimension(), e.getNumberOfParameters());
            Hxx = Matrix::Zero(e.getInputTangentSpaceDimension(),
                               e.getInputTangentSpaceDimension());
            Hxp = Matrix::Zero(e.getInputTangentSpaceDimension(),
                               e.getNumberOfParameters());
            Hpp = Matrix::Zero(e.getNumberOfParameters(),
                               e.getNumberOfParameters());
        }

        // Perform initialisation depending on what type we have?
    }

    /// Evaluator output vector y
    DenseVector y;

    /// Matrix for ∂y/∂x
    Matrix Jx;
    /// Matrix for ∂y/∂p
    Matrix Jp;

    /// Matrix for lower-triangular matrix ∂²(λᵀy)/∂x²
    Matrix Hxx;
    /// Matrix for lower-triangular matrix ∂²(λᵀy)/∂x∂p
    Matrix Hxp;
    /// Matrix for lower-triangular matrix ∂²(λᵀy)/∂p²
    Matrix Hpp;
};

}  // namespace bopt