#pragma once

#include "bopt/Common.hpp"
#include "bopt/FunctionTraits.hpp"

namespace bopt {

template <typename FunctionTraits, int OutputSize>
struct EvaluatorDataTpl {
    using Scalar = typename FunctionTraits::Scalar;

    using OutputType = std::conditional_t<
        OutputSize == 1, Scalar,
        Eigen::Matrix<
            Scalar, OutputSize == Eigen::Dynamic ? Eigen::Dynamic : OutputSize,
            1>>;

    using JacobianType =
        std::conditional_t<OutputSize == 1, typename FunctionTraits::VectorType,
                           typename FunctionTraits::MatrixType>;

    using HessianType = typename FunctionTraits::HessianType;

    EvaluatorDataTpl(const EvaluatorTpl<FunctionTraits, OutputSize> &e) {
        const auto &nx = e.getInputTangentSpaceDimension();
        const auto &np = e.getInputTangentSpaceDimension();
        const auto &m = e.getOutputDimension();

        if constexpr (OutputSize == 1) {
            y = Scalar(0);
        } else {
            y = Output::Zero(m);
        }

        if constexpr (FunctionTraits::type == FunctionType::SPARSE) {
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