#pragma once

#include "bopt/costs/CostBase.hpp"

namespace bopt {

template <typename Scalar>
struct LinearCostDataTpl;

/**
 * @brief Linear cost of the form fₚ(x) = aₚᵀx + bₚ
 *
 */
template <typename FunctionTraits>
class LinearCostTpl : public CostTpl<FunctionTraits> {
   public:
    using Base = CostTpl<FunctionTraits>;
    using EvaluatorData = typename Base::Data;

    using Data = LinearCostDataTpl<FunctionTraits>;

    std::shared_ptr<Data> createData() const {
        return std::shared_ptr<Data>(this->createDataImpl());
    }

    /**
     * @brief Evaluates the vector coeffcient vector bₚ for the cost fₚ(x) = aₚ
     * x + bₚ
     *
     * @param a Coefficient vector aₚ
     * @param b Constant bₚ
     */
    void evalCoefficients(Data &data) const { evalCoefficientsImpl(data); }

   protected:
    LinearCostTpl(const Index &dim_input) : CostTpl<FunctionTraits>(dim_input) {
        this->setName("linear_cost");
    }

    LinearCostTpl(const std::shared_ptr<CostTpl<FunctionTraits>> &cost)
        : CostTpl<FunctionTraits>(cost) {
        this->setName("linear_cost");
    }

    virtual void evalCoefficientsImpl(Data &data) const {}

    Data *createDataImpl() const override {
        auto data = new Data(*this);
        return data;
    }

   private:
};

template <typename Scalar>
using DenseLinearCostTpl = LinearCostTpl<DenseFunctionTraits<Scalar>>;
using DenseLinearCost = DenseLinearCostTpl<Real>;

template <typename Scalar>
using SparseLinearCostTpl = LinearCostTpl<SparseFunctionTraits<Scalar>>;
using SparseLinearCost = SparseLinearCostTpl<Real>;

/**
 * @brief Contains the data associated with a linear cost
 *
 * @tparam FunctionTraits
 */
template <typename FunctionTraits>
struct LinearCostDataTpl : public CostDataTpl<FunctionTraits> {
    using Scalar = typename FunctionTraits::Scalar;

    using Vector = typename FunctionTraits::OutputVector;
    using Matrix = typename FunctionTraits::OutputMatrix;

    LinearCostDataTpl(const LinearCostTpl<FunctionTraits> &c)
        : CostDataTpl<FunctionTraits>(c) {
        if constexpr (FunctionTraits::type == "Sparse") {
            // Sparse: allocate sparse objects properly
            a.resize(c.getInputDimension());
        } else {
            // Dense
            a = Vector::Zero(c.getInputDimension());
        }
    }

    /// Dense coefficient vector a
    Vector a;
    /// Constant term b
    Scalar b;
};

}  // namespace bopt
