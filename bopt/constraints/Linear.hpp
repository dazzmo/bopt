#pragma once

#include "bopt/Constraint.hpp"

namespace bopt {

template <typename Scalar>
struct LinearConstraintDataTpl;

/**
 * @brief Constraint of the form lb ≤ Ax ≤ ub
 *
 */
template <typename FunctionTraits>
class LinearConstraintTpl : public ConstraintTpl<FunctionTraits> {
   public:
    using EvaluatorData = typename ConstraintTpl<FunctionTraits>::EvaluatorData;
    using ConstraintData = typename ConstraintTpl<FunctionTraits>::Data;
    using Data = LinearConstraintDataTpl<FunctionTraits>;

    std::shared_ptr<Data> createData() const {
        return std::shared_ptr<Data>(this->createDataImpl());
    }

    /**
     * @brief Evaluates the vector coeffcient vector bₚ for the constraint lb ≤
     * Ax ≤ ub
     *
     * @param A Coefficient matrix Aₚ
     */
    void evalCoefficients(Data &data) const { evalCoefficientsImpl(data); }

   protected:
    LinearConstraintTpl(const Index &dim_input, const Index &dim_output)
        : ConstraintTpl<FunctionTraits>(dim_input, dim_output) {
        this->setName("linear_constraint");
    }

    LinearConstraintTpl(
        const std::shared_ptr<ConstraintTpl<FunctionTraits>> &constraint)
        : ConstraintTpl<FunctionTraits>(constraint) {
        this->setName("linear_constraint");
    }

    virtual void evalCoefficientsImpl(Data &data) const {}

    virtual Data *createDataImpl() const { return new Data(*this); }

   private:
};

template <typename Scalar>
using DenseLinearConstraintTpl =
    LinearConstraintTpl<DenseFunctionTraits<Scalar>>;

template <typename Scalar>
using SparseLinearConstraintTpl =
    LinearConstraintTpl<SparseFunctionTraits<Scalar>>;

using DenseLinearConstraint = DenseLinearConstraintTpl<Real>;
using SparseLinearConstraint = SparseLinearConstraintTpl<Real>;

template <typename FunctionTraits>
struct LinearConstraintDataTpl : public ConstraintDataTpl<FunctionTraits> {
    using Base = ConstraintDataTpl<FunctionTraits>;
    using Matrix = typename Base::Matrix;

    LinearConstraintDataTpl(const LinearConstraintTpl<FunctionTraits> &c)
        : ConstraintDataTpl<FunctionTraits>(c) {
        if constexpr (FunctionTraits::type == "Sparse") {
            // Sparse: allocate sparse objects properly
            A.resize(c.getOutputDimension(), c.getInputDimension());
        } else {
            A = Matrix::Zero(c.getOutputDimension(), c.getInputDimension());
        }
    }

    Matrix A;
};

}  // namespace bopt
