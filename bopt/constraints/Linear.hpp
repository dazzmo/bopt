#pragma once

#include "bopt/constraints/ConstraintBase.hpp"

namespace bopt {

template <typename Scalar>
struct LinearConstraintDataTpl;

/**
 * @brief Constraint of the form lb ≤ Ax ≤ ub
 *
 */
template <typename EvaluatorTraits>
class LinearConstraintTpl : public ConstraintTpl<EvaluatorTraits> {
   public:
    using Base = ConstraintTpl<EvaluatorTraits>;
    using EvaluatorData = typename Base::EvaluatorData;
    using ConstraintData = typename Base::Data;
    using Data = LinearConstraintDataTpl<EvaluatorTraits>;

    void setDataSparsity(Data &data) const { this->setDataSparsityImpl(data); }

    /**
     * @brief Evaluates the vector coeffcient vector bₚ for the constraint lb ≤
     * Ax ≤ ub
     *
     * @param A Coefficient matrix Aₚ
     */
    void evalCoefficients(Data &data) const { evalCoefficientsImpl(data); }

   protected:
    LinearConstraintTpl(const Size &dim_input, const Size &dim_output)
        : Base(dim_input, dim_output) {
        this->setName("linear_constraint");
    }

    /**
     * @brief Construct a new Linear Constraint Tpl object
     *
     * @param constraint
     */
    LinearConstraintTpl(const std::shared_ptr<Base> &constraint)
        : Base(constraint) {
        this->setName("linear_constraint");
    }

    virtual void evalCoefficientsImpl(Data &data) const {}
    virtual void setDataSparsityImpl(Data &data) const {}

   private:
};

template <typename Scalar>
using DenseLinearConstraintTpl =
    LinearConstraintTpl<DenseEvaluatorTraits<Scalar>>;
using DenseLinearConstraint = DenseLinearConstraintTpl<Real>;

template <typename Scalar>
using SparseLinearConstraintTpl =
    LinearConstraintTpl<SparseEvaluatorTraits<Scalar>>;
using SparseLinearConstraint = SparseLinearConstraintTpl<Real>;

/**
 * @brief Data associated with a linear constraint
 *
 * @tparam EvaluatorTraits
 */
template <typename EvaluatorTraits>
struct LinearConstraintDataTpl : public ConstraintDataTpl<EvaluatorTraits> {
    using Base = ConstraintDataTpl<EvaluatorTraits>;
    using Matrix = typename Base::Matrix;

    LinearConstraintDataTpl(const LinearConstraintTpl<EvaluatorTraits> &c)
        : ConstraintDataTpl<EvaluatorTraits>(c) {
        if constexpr (EvaluatorTraits::type == FunctionType::SPARSE) {
            A.resize(c.numOutputs(), c.numInputs());
        } else {
            A = Matrix::Zero(c.numOutputs(), c.numInputs());
        }

        c.setDataSparsity(*this);
    }

    /// @brief Coefficient matrix A
    Matrix A;
};

}  // namespace bopt
