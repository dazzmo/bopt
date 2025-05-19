#pragma once

#include "bopt/EvaluatorData.hpp"

namespace bopt {

/**
 * @brief Evaluator data struct for evaluation of EvaluatorTpl classes.
 *
 * @tparam EvaluatorTraits Traits of the evaluator function
 * @tparam OutputSize The dimension of the output
 */
template <typename EvaluatorTraits, int OutputSize>
EvaluatorDataTpl<EvaluatorTraits, OutputSize>::EvaluatorDataTpl(
    const EvaluatorTpl<EvaluatorTraits, OutputSize> &e) {
    const auto &nx = e.dimInputTangentSpace();
    const auto &np = e.numParameters();
    const auto &m = e.numOutputs();

    y = DenseVector::Zero(m);

    if constexpr (EvaluatorTraits::type == FunctionType::SPARSE) {
        Jx.resize(m, nx);
        Jp.resize(m, np);
        Hxx.resize(nx, nx);
        Hxp.resize(nx, np);
        Hpp.resize(np, np);

    } else {
        Jx = JacobianType::Zero(m, nx);
        Jp = JacobianType::Zero(m, np);
        Hxx = HessianType::Zero(nx, nx);
        Hxp = HessianType::Zero(nx, np);
        Hpp = HessianType::Zero(np, np);
    }

    // Set up data sparsity patterns
    e.setDataSparsity(*this);
}

/**
 * @brief Template specialisation for scalar outputs
 *
 * @tparam EvaluatorTraits
 */
template <typename EvaluatorTraits>
EvaluatorDataTpl<EvaluatorTraits, 1>::EvaluatorDataTpl(
    const EvaluatorTpl<EvaluatorTraits, 1> &e) {
    const auto &nx = e.dimInputTangentSpace();
    const auto &np = e.numParameters();
    const auto &m = e.numOutputs();

    y = Scalar(0);

    if constexpr (EvaluatorTraits::type == FunctionType::SPARSE) {
        gx.resize(nx);
        gp.resize(np);
        Hxx.resize(nx, nx);
        Hxp.resize(nx, np);
        Hpp.resize(np, np);
    } else {
        gx = GradientType::Zero(nx);
        gp = GradientType::Zero(np);
        Hxx = HessianType::Zero(nx, nx);
        Hxp = HessianType::Zero(nx, np);
        Hpp = HessianType::Zero(np, np);
    }

    // Set up data sparsity patterns
    e.setDataSparsity(*this);
}

/**
 * @brief Evaluator data struct for evaluation of EvaluatorTpl classes.
 *
 * @tparam EvaluatorTraits Traits of the evaluator function
 * @tparam OutputSize The dimension of the output
 */
template <typename EvaluatorTraits, int OutputSize>
LinearEvaluatorDataTpl<EvaluatorTraits, OutputSize>::LinearEvaluatorDataTpl(
    const LinearEvaluatorTpl<EvaluatorTraits, OutputSize> &e)
    : EvaluatorDataTpl<EvaluatorTraits, OutputSize>(e) {
    const auto &nx = e.dimInputTangentSpace();
    const auto &np = e.numParameters();
    const auto &m = e.numOutputs();

    if constexpr (EvaluatorTraits::type == FunctionType::SPARSE) {
        A.resize(m, nx);
        b = OutputType::Zero(m);
    } else {
        A = JacobianType::Zero(m, nx);
        b = OutputType::Zero(m);
    }

    // Set up data sparsity patterns
    e.setDataSparsity(*this);
}

/**
 * @brief Template specialisation for scalar outputs
 *
 * @tparam EvaluatorTraits Traits of the evaluator function
 * @tparam OutputSize The dimension of the output
 */
template <typename EvaluatorTraits>
LinearEvaluatorDataTpl<EvaluatorTraits, 1>::LinearEvaluatorDataTpl(
    const LinearEvaluatorTpl<EvaluatorTraits, 1> &e)
    : EvaluatorDataTpl<EvaluatorTraits, 1>(e) {
    const auto &nx = e.dimInputTangentSpace();
    const auto &np = e.numParameters();
    const auto &m = e.numOutputs();

    if constexpr (EvaluatorTraits::type == FunctionType::SPARSE) {
        a.resize(nx);
    } else {
        a = GradientType::Zero(nx);
    }
    b = OutputType(0);

    // Set up data sparsity patterns
    e.setDataSparsity(*this);
}

/**
 * @brief Evaluator data for scalar quadratic expressions of the form (1/2) xᵀ
 * Aₚ x + bₚᵀ x + cₚ
 *
 * @tparam EvaluatorTraits Traits of the evaluator function
 */
template <typename EvaluatorTraits>
QuadraticEvaluatorDataTpl<EvaluatorTraits>::QuadraticEvaluatorDataTpl(
    const EvaluatorTpl<EvaluatorTraits, 1> &e)
    : EvaluatorDataTpl<EvaluatorTraits, 1>(e) {
    const auto &nx = e.dimInputTangentSpace();
    const auto &m = e.numOutputs();

    if constexpr (EvaluatorTraits::type == FunctionType::SPARSE) {
        A.resize(nx, nx);
        b.resize(nx);
    } else {
        A = HessianType::Zero(nx, nx);
        b = GradientType::Zero(nx);
    }
    c = Scalar(0);

    // Set up data sparsity patterns
    e.setDataSparsity(*this);
}

}  // namespace bopt