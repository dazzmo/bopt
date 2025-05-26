#pragma once

#include "bopt/Evaluator.hpp"
#include "bopt/EvaluatorTraits.hpp"

namespace bopt {

// Forward declarations of evaluators
template <typename EvaluatorTraits, int OutputSizeAtCompileTime>
class EvaluatorTpl;

template <typename EvaluatorTraits, int OutputSizeAtCompileTime>
class LinearEvaluatorTpl;

template <typename EvaluatorTraits>
class QuadraticEvaluatorTpl;

/**
 * @brief Evaluator data struct for evaluation of EvaluatorTpl classes.
 *
 * @tparam EvaluatorTraits Traits of the evaluator function
 * @tparam OutputSizeAtCompileTime The dimension of the output
 */
template <typename EvaluatorTraits, int OutputSizeAtCompileTime>
struct EvaluatorDataTpl {
    using Scalar = typename EvaluatorTraits::Scalar;

    using DenseVector = typename EvaluatorTraits::DenseVector;

    /// @brief The output type for the evaluator
    using OutputType = typename EvaluatorTraits::DenseVector;
    /// @brief The Jacobian type for the evaluator
    using JacobianType = typename EvaluatorTraits::MatrixType;
    /// @brief The Hessian type for the evaluator
    using HessianType = typename EvaluatorTraits::MatrixType;

    EvaluatorDataTpl(
        const EvaluatorTpl<EvaluatorTraits, OutputSizeAtCompileTime> &e);

    /// @brief Output y
    OutputType y;

    /// @brief Jacobian ∂y/∂x
    JacobianType Jx;
    /// @brief Jacobian ∂y/∂p
    JacobianType Jp;

    /// @brief Lower-triangular Hessian matrix ∂²(λᵀy)/∂x²
    HessianType Hxx;
    /// @brief Lower-triangular Hessian matrix ∂²(λᵀy)/∂x∂p
    HessianType Hxp;
    /// @brief Lower-triangular Hessian matrix ∂²(λᵀy)/∂p²
    HessianType Hpp;
};

/**
 * @brief Template specialisation for scalar outputs
 *
 * @tparam EvaluatorTraits
 */
template <typename EvaluatorTraits>
struct EvaluatorDataTpl<EvaluatorTraits, 1> {
    using Scalar = typename EvaluatorTraits::Scalar;

    using DenseVector = typename EvaluatorTraits::DenseVector;

    /// @brief The output type for the evaluator
    using OutputType = Scalar;
    /// @brief The gradient type for the evaluator
    using GradientType = typename EvaluatorTraits::VectorType;
    /// @brief The Hessian type for the evaluator
    using HessianType = typename EvaluatorTraits::MatrixType;

    EvaluatorDataTpl(const EvaluatorTpl<EvaluatorTraits, 1> &e);

    /// @brief Output y
    OutputType y;

    /// @brief Gradient ∂y/∂x
    GradientType gx;
    /// @brief Gradient ∂y/∂p
    GradientType gp;

    /// @brief Lower-triangular Hessian matrix ∂²(λᵀy)/∂x²
    HessianType Hxx;
    /// @brief Lower-triangular Hessian matrix ∂²(λᵀy)/∂x∂p
    HessianType Hxp;
    /// @brief Lower-triangular Hessian matrix ∂²(λᵀy)/∂p²
    HessianType Hpp;
};

/**
 * @brief Evaluator data struct for evaluation of EvaluatorTpl classes.
 *
 * @tparam EvaluatorTraits Traits of the evaluator function
 * @tparam OutputSizeAtCompileTime The dimension of the output
 */
template <typename EvaluatorTraits, int OutputSizeAtCompileTime>
struct LinearEvaluatorDataTpl
    : public EvaluatorDataTpl<EvaluatorTraits, OutputSizeAtCompileTime> {
    using Base = EvaluatorDataTpl<EvaluatorTraits, OutputSizeAtCompileTime>;
    using Scalar = typename Base::Scalar;
    using DenseVector = typename Base::DenseVector;
    using OutputType = typename Base::OutputType;
    using JacobianType = typename Base::JacobianType;
    using HessianType = typename Base::HessianType;

    LinearEvaluatorDataTpl(
        const LinearEvaluatorTpl<EvaluatorTraits, OutputSizeAtCompileTime> &e);

    /// @brief Coefficient matrix A
    JacobianType A;
    /// @brief Constant vector b
    OutputType b;
};

/**
 * @brief Template specialisation for scalar outputs
 *
 * @tparam EvaluatorTraits Traits of the evaluator function
 * @tparam OutputSizeAtCompileTime The dimension of the output
 */
template <typename EvaluatorTraits>
struct LinearEvaluatorDataTpl<EvaluatorTraits, 1>
    : public EvaluatorDataTpl<EvaluatorTraits, 1> {
    using Base = EvaluatorDataTpl<EvaluatorTraits, 1>;

    using DenseVector = typename Base::DenseVector;
    using OutputType = typename Base::OutputType;
    using GradientType = typename Base::GradientType;
    using HessianType = typename Base::HessianType;

    LinearEvaluatorDataTpl(const LinearEvaluatorTpl<EvaluatorTraits, 1> &e);

    /// @brief Coefficient vector a
    GradientType a;
    /// @brief Constant vector b
    OutputType b;
};

/**
 * @brief Evaluator data for scalar quadratic expressions of the form (1/2) xᵀ
 * Aₚ x + bₚᵀ x + cₚ
 *
 * @tparam EvaluatorTraits Traits of the evaluator function
 */
template <typename EvaluatorTraits>
struct QuadraticEvaluatorDataTpl : public EvaluatorDataTpl<EvaluatorTraits, 1> {
    using Base = EvaluatorDataTpl<EvaluatorTraits, 1>;
    using Scalar = typename Base::Scalar;
    using OutputType = typename Base::OutputType;
    using GradientType = typename Base::GradientType;
    using HessianType = typename Base::HessianType;

    QuadraticEvaluatorDataTpl(const EvaluatorTpl<EvaluatorTraits, 1> &e);

    /// @brief Coefficient matrix A
    HessianType A;
    /// @brief Coefficient vector b
    GradientType b;
    /// @brief Constant vector c
    OutputType c;
};

}  // namespace bopt

// -------------------- Details -------------------------- //
#include "bopt/EvaluatorData.hxx"