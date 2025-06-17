#pragma once

#include "bopt/EvaluatorData.hpp"

namespace bopt {

template <typename ScalarType, int OutputSizeAtCompileTime,
          SparsityType Sparsity>
EvaluatorDataTpl<ScalarType, OutputSizeAtCompileTime,
                 Sparsity>::EvaluatorDataTpl(const Size &n, const Size &m,
                                             const Size &p) {
    y = OutputType::Zero(m);

    if constexpr (Sparsity == SparsityType::SPARSE) {
        Jx.resize(m, n);
        Jp.resize(m, p);
        Hxx.resize(n, n);
        Hxp.resize(n, p);
        Hpp.resize(p, p);

    } else {
        Jx = JacobianType::Zero(m, n);
        Jp = JacobianType::Zero(m, p);
        Hxx = HessianType::Zero(n, n);
        Hxp = HessianType::Zero(n, p);
        Hpp = HessianType::Zero(p, p);
    }
}

template <typename ScalarType, int OutputSizeAtCompileTime,
          SparsityType Sparsity>
EvaluatorDataTpl<ScalarType, OutputSizeAtCompileTime, Sparsity>::
    EvaluatorDataTpl(const EvaluatorTpl<ScalarType, OutputSizeAtCompileTime,
                                        Sparsity> &evaluator)
    : EvaluatorDataTpl(evaluator.dimInputTangentSpace(), evaluator.outputSize(),
                       evaluator.numParameters()) {}

template <typename ScalarType, SparsityType Sparsity>
EvaluatorDataTpl<ScalarType, 1, Sparsity>::EvaluatorDataTpl(const Size &n,
                                                            const Size &p) {
    y = OutputType(0);

    if constexpr (Sparsity == SparsityType::SPARSE) {
        gx.resize(n);
        gp.resize(p);
        Hxx.resize(n, n);
        Hxp.resize(n, p);
        Hpp.resize(p, p);
    } else {
        gx = GradientType::Zero(n);
        gp = GradientType::Zero(p);
        Hxx = HessianType::Zero(n, n);
        Hxp = HessianType::Zero(n, p);
        Hpp = HessianType::Zero(p, p);
    }
}

template <typename ScalarType, SparsityType Sparsity>
EvaluatorDataTpl<ScalarType, 1, Sparsity>::EvaluatorDataTpl(
    const EvaluatorTpl<ScalarType, 1, Sparsity> &evaluator)
    : EvaluatorDataTpl(evaluator.dimInputTangentSpace(),
                       evaluator.numParameters()) {}

template <typename ScalarType, int OutputSizeAtCompileTime,
          SparsityType Sparsity>
LinearDataTpl<ScalarType, OutputSizeAtCompileTime, Sparsity>::LinearDataTpl(
    const Size &n, const Size &m, const Size &p)
    : EvaluatorDataTpl<ScalarType, OutputSizeAtCompileTime, Sparsity>(n, m, p) {
    if constexpr (Sparsity == SparsityType::SPARSE) {
        A.resize(m, n);
        b = OutputType::Zero(m);
    } else {
        A = JacobianType::Zero(m, n);
        b = OutputType::Zero(m);
    }
}

template <typename ScalarType, int OutputSizeAtCompileTime,
          SparsityType Sparsity>
LinearDataTpl<ScalarType, OutputSizeAtCompileTime, Sparsity>::LinearDataTpl(
    const EvaluatorTpl<ScalarType, OutputSizeAtCompileTime, Sparsity>
        &evaluator)
    : LinearDataTpl(evaluator.dimInputTangentSpace(), evaluator.outputSize(),
                    evaluator.numParameters()) {}

template <typename ScalarType, SparsityType Sparsity>
LinearDataTpl<ScalarType, 1, Sparsity>::LinearDataTpl(const Size &n,
                                                      const Size &p)
    : EvaluatorDataTpl<ScalarType, 1, Sparsity>(n, p) {
    if constexpr (Sparsity == SparsityType::SPARSE) {
        a.resize(n);
    } else {
        a = GradientType::Zero(n);
    }
    b = OutputType(0);
}

template <typename ScalarType, SparsityType Sparsity>
LinearDataTpl<ScalarType, 1, Sparsity>::LinearDataTpl(
    const EvaluatorTpl<ScalarType, 1, Sparsity> &evaluator)
    : LinearDataTpl(evaluator.dimInputTangentSpace(),
                    evaluator.numParameters()) {}

template <typename ScalarType, SparsityType Sparsity>
QuadraticDataTpl<ScalarType, Sparsity>::QuadraticDataTpl(const Size &n,
                                                         const Size &p)
    : EvaluatorDataTpl<ScalarType, 1, Sparsity>(n, p) {
    if constexpr (Sparsity == SparsityType::SPARSE) {
        A.resize(n, n);
        b.resize(n);
    } else {
        A = HessianType::Zero(n, n);
        b = GradientType::Zero(n);
    }
    c = Scalar(0);
}

template <typename ScalarType, SparsityType Sparsity>
QuadraticDataTpl<ScalarType, Sparsity>::QuadraticDataTpl(
    const EvaluatorTpl<ScalarType, 1, Sparsity> &evaluator)
    : QuadraticDataTpl(evaluator.dimInputTangentSpace(),
                       evaluator.numParameters()) {}

}  // namespace bopt