#pragma once

#include "bopt/EvaluatorTraits.hpp"

namespace bopt {

template <typename ScalarType, int OutputSizeAtCompileTime,
          SparsityType Sparsity>
class EvaluatorTpl;

/**
 * @brief Evaluator data struct for evaluation of EvaluatorTpl classes.
 *
 * @tparam EvaluatorTraits Traits of the evaluator function
 * @tparam OutputSizeAtCompileTime The dimension of the output
 */
template <typename ScalarType, int OutputSizeAtCompileTime = Eigen::Dynamic,
          SparsityType Sparsity = SparsityType::DENSE>
struct EvaluatorDataTpl {
    using Scalar = ScalarType;
    using Traits =
        EvaluatorTraits<ScalarType, OutputSizeAtCompileTime, Sparsity>;

    /// @brief The output type for the evaluator
    using OutputType = typename Traits::OutputType;

    /// @brief The Jacobian type for the evaluator
    using JacobianType = typename Traits::OutputJacobianType;
    /// @brief The Hessian type for the evaluator
    using HessianType = typename Traits::OutputHessianType;

    EvaluatorDataTpl(const Size &n, const Size &m, const Size &p);
    EvaluatorDataTpl(const EvaluatorTpl<ScalarType, OutputSizeAtCompileTime,
                                        Sparsity> &evaluator);

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
template <typename ScalarType, SparsityType Sparsity>
struct EvaluatorDataTpl<ScalarType, 1, Sparsity> {
    using Scalar = ScalarType;
    using Traits = EvaluatorTraits<ScalarType, 1, Sparsity>;

    /// @brief The output type for the evaluator
    using OutputType = typename Traits::OutputType;

    /// @brief The Jacobian type for the evaluator
    using GradientType = typename Traits::OutputGradientType;
    /// @brief The Hessian type for the evaluator
    using HessianType = typename Traits::OutputHessianType;

    EvaluatorDataTpl(const Size &n, const Size &p);
    EvaluatorDataTpl(const EvaluatorTpl<ScalarType, 1, Sparsity> &evaluator);
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
 * @brief Data struct for evaluation of a linear expression of the form A x + b.
 *
 * @tparam EvaluatorTraits Traits of the evaluator function
 * @tparam OutputSizeAtCompileTime The dimension of the output
 */
template <typename ScalarType, int OutputSizeAtCompileTime = Eigen::Dynamic,
          SparsityType Sparsity = SparsityType::DENSE>
struct LinearDataTpl
    : public EvaluatorDataTpl<ScalarType, OutputSizeAtCompileTime, Sparsity> {
    using DataBase =
        EvaluatorDataTpl<ScalarType, OutputSizeAtCompileTime, Sparsity>;

    using Scalar = typename DataBase::Scalar;

    using OutputType = typename DataBase::OutputType;
    using JacobianType = typename DataBase::JacobianType;
    using HessianType = typename DataBase::HessianType;

    LinearDataTpl(const Size &n, const Size &m, const Size &p);
    LinearDataTpl(const EvaluatorTpl<ScalarType, OutputSizeAtCompileTime,
                                     Sparsity> &evaluator);

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
template <typename ScalarType, SparsityType Sparsity>
struct LinearDataTpl<ScalarType, 1, Sparsity>
    : public EvaluatorDataTpl<ScalarType, 1, Sparsity> {
    using DataBase = EvaluatorDataTpl<ScalarType, 1, Sparsity>;

    using OutputType = typename DataBase::OutputType;
    using GradientType = typename DataBase::GradientType;
    using HessianType = typename DataBase::HessianType;

    LinearDataTpl(const Size &n, const Size &p);
    LinearDataTpl(const EvaluatorTpl<ScalarType, 1, Sparsity> &evaluator);

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
template <typename ScalarType, SparsityType Sparsity = SparsityType::DENSE>
struct QuadraticDataTpl : public EvaluatorDataTpl<ScalarType, 1, Sparsity> {
    using DataBase = EvaluatorDataTpl<ScalarType, 1, Sparsity>;
    using Scalar = typename DataBase::Scalar;
    using OutputType = typename DataBase::OutputType;
    using GradientType = typename DataBase::GradientType;
    using HessianType = typename DataBase::HessianType;

    QuadraticDataTpl(const Size &n, const Size &p);
    QuadraticDataTpl(const EvaluatorTpl<ScalarType, 1, Sparsity> &evaluator);

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