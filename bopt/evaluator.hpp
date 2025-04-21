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

using Index = Eigen::Index;

template <typename ScalarType>
struct SparseInputTraits {
    using Scalar = ScalarType;

    using Matrix = Eigen::SparseMatrix<Scalar>;
    using Vector = Eigen::SparseVector<Scalar>;

    using VectorInput = Vector;
    using MatrixInput = Matrix;

    using VectorConstInput = const Vector;
    using MatrixConstInput = const Matrix;

    static constexpr const char *type = "Sparse";
};

template <typename ScalarType>
struct DenseInputTraits {
    using Scalar = ScalarType;

    using Matrix = Eigen::MatrixX<Scalar>;
    using Vector = Eigen::VectorX<Scalar>;

    using VectorInput = Eigen::Ref<Vector>;
    using MatrixInput = Eigen::Ref<Matrix>;

    using VectorConstInput = Eigen::Ref<const Vector>;
    using MatrixConstInput = Eigen::Ref<const Matrix>;

    static constexpr const char *type = "Dense";
};

/**
 * @brief Evaluator class related the evaluation of a function y = fₚ(x)
 *
 * @tparam Scalar
 */
template <typename InputTraits>
class EvaluatorTpl {
   public:
    using Scalar = typename InputTraits::Scalar;

    using InputVector = Eigen::VectorX<Scalar>;
    using InputVectorConstRef = Eigen::Ref<const InputVector>;

    using Vector = typename InputTraits::Vector;
    using Matrix = typename InputTraits::Matrix;

    using VectorInput = typename InputTraits::VectorInput;
    using MatrixInput = typename InputTraits::MatrixInput;

    using VectorConstInput = typename InputTraits::VectorConstInput;
    using MatrixConstInput = typename InputTraits::MatrixConstInput;

    /// @brief The type of data to be used for the evaluation functions
    using Data = EvaluatorDataTpl<InputTraits>;

    EvaluatorTpl(const std::shared_ptr<EvaluatorTpl<InputTraits>> &ptr)
        : ptr_(ptr),
          dim_input_(ptr->dim_input()),
          dim_tangent_space_(ptr->dim_tangent_space()),
          dim_output_(ptr->dim_output()),
          dim_parameter_(ptr->dim_parameter()),
          parameters_(InputVector::Zero(ptr->dim_parameter())),
          description_(ptr->description()) {}

    virtual std::shared_ptr<Data> createData() = 0;

    /**
     * @brief Evaluates the expression y = fₚ(x) using variables x and
     * parameters p (set through \ref EvaluatorTpl::setParameters()).
     *
     * @param x The input vector (dim_input() x 1)
     * @param data
     */
    void eval(const InputVectorConstRef &x, Data &data) const {
        BOPT_ASSERT(x.rows() == dim_input());
        BOPT_ASSERT(data.y.rows() == dim_output());
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
        BOPT_ASSERT(x.rows() == dim_input());
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
        BOPT_ASSERT(x.rows() == dim_input());
        BOPT_ASSERT(lambda.rows() == dim_output());
        evalHessiansImpl(x, lambda, data, compute_xx, compute_xp, compute_pp);
    }

    /**
     * @brief Dimension of the input variable vector, commonly denoted as x.
     *
     * @return const Index&
     */
    const Index &dim_input() const { return dim_input_; }

    /**
     * @brief Dimension of the tangent space for the input vector, typically
     * this is equal to dim_input().
     *
     * @return const Index&
     */
    const Index &dim_tangent_space() const { return dim_tangent_space_; }

    /**
     * @brief Dimension of the output vector y.
     *
     * @return const Index&
     */
    const Index &dim_output() const { return dim_output_; }

    /**
     * @brief Dimension of the parameter vector p.
     *
     * @return const Index&
     */
    const Index &dim_parameter() const { return dim_parameter_; }

    const std::string &description() const { return description_; }

    void setDescription(const std::string &description) {
        description_ = description;
    }

    const InputVector &parameters() const { return parameters_; }

    /**
     * @brief Set the parameter vector p (dim_parameter() x 1).
     *
     * @param p
     */
    void setParameters(const InputVectorConstRef &p) {
        BOPT_ASSERT(p.size() == dim_parameter());
        parameters_ = p;
    }

   protected:
    EvaluatorTpl(const Index &n_inputs, const Index &n_outputs,
                 const std::string &description = "")
        : ptr_(nullptr),
          dim_input_(n_inputs),
          dim_tangent_space_(n_inputs),
          dim_output_(n_outputs),
          dim_parameter_(0),
          parameters_(InputVector::Zero(0)),
          description_(description) {}

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
        dim_parameter_ = dim;
        parameters_ = InputVector::Zero(dim);
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
    std::shared_ptr<EvaluatorTpl<InputTraits>> ptr_;

    /// @brief Dimension of the input vector
    Index dim_input_;
    Index dim_tangent_space_;
    Index dim_parameter_;
    Index dim_output_;

    InputVector parameters_;
    std::string description_;
};

typedef EvaluatorTpl<double> Evaluator;

template <typename Scalar>
using DenseEvaluatorTpl = EvaluatorTpl<DenseInputTraits<Scalar>>;

template <typename Scalar>
using SparseEvaluatorTpl = EvaluatorTpl<SparseInputTraits<Scalar>>;

// template <typename InputTraits>
// std::ostream &operator<<(std::ostream &os, const EvaluatorTpl<Scalar> &e) {
//     os << "Evaluator\n";
//     os << "description: " << e.description() << '\n';
//     os << "input dim: " << e.dim_input() << '\n';
//     os << "output dim: " << e.dim_output();
//     return os;
// }

template <typename InputTraitType>
struct EvaluatorDataTpl {
    using Vector = typename InputTraitType::Vector;
    using Matrix = typename InputTraitType::Matrix;

    using VectorInput = typename InputTraitType::VectorInput;
    using MatrixInput = typename InputTraitType::MatrixInput;

    using VectorConstInput = typename InputTraitType::VectorConstInput;
    using MatrixConstInput = typename InputTraitType::MatrixConstInput;

    EvaluatorDataTpl(const EvaluatorTpl<InputTraitType> &e) {
        if constexpr (InputTraitType::type == "Sparse") {
            // Sparse: allocate sparse objects properly
            y.resize(e.dim_output());
            Jx.resize(e.dim_output(), e.dim_tangent_space());
            Jp.resize(e.dim_output(), e.dim_parameter());
            Hxx.resize(e.dim_tangent_space(), e.dim_tangent_space());
            Hxp.resize(e.dim_tangent_space(), e.dim_parameter());
            Hpp.resize(e.dim_parameter(), e.dim_parameter());
            // Optionally set values to zero explicitly if needed
        } else {
            // Dense
            y = Vector::Zero(e.dim_output());
            Jx = Matrix::Zero(e.dim_output(), e.dim_tangent_space());
            Jp = Matrix::Zero(e.dim_output(), e.dim_parameter());
            Hxx = Matrix::Zero(e.dim_tangent_space(), e.dim_tangent_space());
            Hxp = Matrix::Zero(e.dim_tangent_space(), e.dim_parameter());
            Hpp = Matrix::Zero(e.dim_parameter(), e.dim_parameter());
        }

        // Perform initialisation depending on what type we have?
    }

    /// Evaluator output vector y
    Vector y;

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

typedef EvaluatorDataTpl<double> EvaluatorData;

}  // namespace bopt