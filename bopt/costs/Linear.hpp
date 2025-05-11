#pragma once

#include "bopt/costs/CostBase.hpp"

namespace bopt {

// Forward declaration of data type
template <typename EvaluatorTraits>
struct LinearCostDataTpl;

/**
 * @brief Linear cost of the form fₚ(x) = aₚᵀx + bₚ
 *
 */
template <typename EvaluatorTraits>
class LinearCostTpl : public CostTpl<EvaluatorTraits> {
   public:
    using Base = CostTpl<EvaluatorTraits>;
    using EvaluatorData = typename Base::EvaluatorData;
    using Data = LinearCostDataTpl<EvaluatorTraits>;

    void setDataSparsity(Data &data) const { this->setDataSparsityImpl(data); }

    /**
     * @brief Evaluates the vector coeffcient vector bₚ for the cost fₚ(x) = aₚ
     * x + bₚ
     *
     * @param a Coefficient vector aₚ
     * @param b Constant bₚ
     */
    void evalCoefficients(Data &data) const { evalCoefficientsImpl(data); }

   protected:
    LinearCostTpl(const Index &nx) : CostTpl<EvaluatorTraits>(nx) {
        this->setName("linear cost");
    }

    LinearCostTpl(const std::shared_ptr<LinearCostTpl<EvaluatorTraits>> &cost)
        : CostTpl<EvaluatorTraits>(cost) {
        this->setName("linear cost");
    }

    virtual void evalCoefficientsImpl(Data &data) const {
        if (ptr_) ptr_->evalCoefficients(data);
    }

    virtual void setDataSparsityImpl(Data &data) const {
        if (ptr_) ptr_->setDataSparsity(data);
    }

   private:
    std::shared_ptr<LinearCostTpl<EvaluatorTraits>> ptr_{nullptr};
};

template <typename Scalar>
using DenseLinearCostTpl = LinearCostTpl<DenseEvaluatorTraits<Scalar>>;
using DenseLinearCost = DenseLinearCostTpl<Real>;

template <typename Scalar>
using SparseLinearCostTpl = LinearCostTpl<SparseEvaluatorTraits<Scalar>>;
using SparseLinearCost = SparseLinearCostTpl<Real>;

/**
 * @brief Contains the data associated with a linear cost
 *
 * @tparam EvaluatorTraits
 */
template <typename EvaluatorTraits>
struct LinearCostDataTpl : public CostDataTpl<EvaluatorTraits> {
    using Scalar = typename EvaluatorTraits::Scalar;
    using Vector = typename EvaluatorTraits::VectorType;
    using Matrix = typename EvaluatorTraits::MatrixType;

    LinearCostDataTpl(const LinearCostTpl<EvaluatorTraits> &c)
        : CostDataTpl<EvaluatorTraits>(c) {
        if constexpr (EvaluatorTraits::type == FunctionType::SPARSE) {
            a.resize(c.numInputs());
        } else {
            a = Vector::Zero(c.numInputs());
        }
        c.setDataSparsity(*this);
    }

    /// Coefficient vector a
    Vector a;
    /// Constant term b
    Scalar b;
};

}  // namespace bopt
