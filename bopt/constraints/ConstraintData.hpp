#pragma once

#include "bopt/Evaluator.hpp"
#include "bopt/Logging.hpp"

namespace bopt {

template <typename EvaluatorTraits>
struct ConstraintDataTpl : public EvaluatorDataTpl<EvaluatorTraits> {
    using Base = EvaluatorDataTpl<EvaluatorTraits>;

    using DenseVector = typename Base::DenseVector;
    using JacobianType = typename Base::JacobianType;
    using HessianType = typename Base::HessianType;

    ConstraintDataTpl(const ConstraintTpl<EvaluatorTraits> &c)
        : EvaluatorDataTpl<EvaluatorTraits>(c) {
        const Size &p = c.numParameters();
        const Size &m = c.numOutputs();

        lb = DenseVector::Zero(m);
        ub = DenseVector::Zero(m);

        if constexpr (EvaluatorTraits::type == FunctionType::SPARSE) {
            // Sparse: allocate sparse objects properly
            Jlb_p.resize(m, p);
            Jub_p.resize(m, p);
            Hlb_pp.resize(p, p);
            Hub_pp.resize(p, p);
        } else {
            // Dense
            Jlb_p = Matrix::Zero(m, p);
            Jub_p = Matrix::Zero(m, p);
            Hlb_pp = Matrix::Zero(p, p);
            Hub_pp = Matrix::Zero(p, p);
        }
    }

    /// @brief Lower bound cₗ
    DenseVector lb;
    /// @brief Upper bound cᵤ
    DenseVector ub;

    /// @brief Lower bound Jacobian ∂cₗ/∂p
    JacobianType Jlb_p;
    /// @brief Upper bound Jacobian ∂cᵤ/∂p
    JacobianType Jub_p;

    /// @brief Lower-triangular lower bound Hessian matrix ∂²(λᵀcₗ)/∂p²
    HessianType Hlb_pp;
    /// @brief Lower-triangular lower bound Hessian matrix ∂²(λᵀcᵤ)/∂p²
    HessianType Hub_pp;
};

}  // namespace bopt
