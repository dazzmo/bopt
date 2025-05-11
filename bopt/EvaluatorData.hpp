#pragma once

#include "bopt/Evaluator.hpp"

namespace bopt {

/**
 * @brief Evaluator data struct for evaluation of EvaluatorTpl classes.
 *
 * @tparam EvaluatorTraits Traits of the evaluator function
 * @tparam OutputSize The dimension of the output
 */
template <typename EvaluatorTraits, int OutputSize = Eigen::Dynamic>
struct EvaluatorDataTpl {
    using Scalar = typename EvaluatorTraits::Scalar;

    using OutputType =
        std::conditional_t<OutputSize == 1, Scalar,
                           typename EvaluatorTraits::DenseVector<OutputSize>>;

    using JacobianType =
        std::conditional_t<OutputSize == 1,
                           typename EvaluatorTraits::VectorType,
                           typename EvaluatorTraits::MatrixType>;

    using HessianType = typename EvaluatorTraits::HessianType;

    EvaluatorDataTpl(const EvaluatorTpl<EvaluatorTraits, OutputSize> &e) {
        const auto &nx = e.tangentSpaceDimension();
        const auto &np = e.tangentSpaceDimension();
        const auto &m = e.numOutputs();

        if constexpr (OutputSize == 1) {
            y = Scalar(0);
        } else {
            y = Output::Zero(m);
        }

        if constexpr (EvaluatorTraits::type == FunctionType::SPARSE) {
            if constexpr (OutputSize == 1) {
                Jx.resize(nx);
                Jp.resize(np);
            } else {
                Jx.resize(m, nx);
                Jp.resize(m, np);
            }
            Hxx.resize(nx, nx);
            Hxp.resize(nx, np);
            Hpp.resize(np, np);

        } else {
            if constexpr (OutputSize == 1) {
                // If scalar output, Jacobian is a vector (i.e. a gradient)
                Jx = JacobianType::Zero(nx);
                Jp = JacobianType::Zero(np);
            } else {
                Jx = JacobianType::Zero(m, nx);
                Jp = JacobianType::Zero(m, np);
            }

            Hxx = HessianType::Zero(nx, nx);
            Hxp = HessianType::Zero(nx, np);
            Hpp = HessianType::Zero(np, np);
        }

        // Set up data sparsity patterns
        e.setDataSparsity(*this);
    }

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

}  // namespace bopt