#pragma once

#include "bopt/costs/CostBase.hpp"

namespace bopt {

/**
 * @brief Types of hessians
 *
 */
enum class HessianType { kPositiveDefinite, kPositiveSemiDefinite, Indefinite };

template <typename FunctionTraits>
struct QuadraticCostDataTpl;

/**
 * @brief Quadratic cost of the form fₚ(x) = (1/2) xᵀ Aₚ x + bₚᵀ x + cₚ
 *
 */
template <typename FunctionTraits>
class QuadraticCostTpl : public CostTpl<FunctionTraits> {
   public:
    using Data = QuadraticCostDataTpl<FunctionTraits>;

    std::shared_ptr<Data> createData() const {
        return std::shared_ptr<Data>(this->createDataImpl());
    }

    /**
     * @brief Evaluates the vector coeffcient vector bₚ for the cost fₚ(x) =
     * (1/2) xᵀ Aₚ x + bₚᵀ x + cₚ
     *
     * @param A Lower triangular matrix Aₚ
     * @param b Vector bₚ
     * @param c Constant cₚ
     */
    void evalCoefficients(Data &data) const { evalCoefficientsImpl(data); }

   protected:
    QuadraticCostTpl<FunctionTraits>(const Index &dim_input)
        : CostTpl<FunctionTraits>(dim_input) {
        this->setName("quadratic_cost");
    }

    QuadraticCostTpl<FunctionTraits>(
        const std::shared_ptr<CostTpl<FunctionTraits>> &cost)
        : CostTpl<FunctionTraits>(cost) {
        this->setName("quadratic_cost");
    }

    virtual Data *createDataImpl() const { return new Data(*this); }

    virtual void evalCoefficientsImpl(Data &data) const {}

   private:
};

template <typename Scalar>
using DenseQuadraticCostTpl = QuadraticCostTpl<DenseFunctionTraits<Scalar>>;

template <typename Scalar>
using SparseQuadraticCostTpl = QuadraticCostTpl<SparseFunctionTraits<Scalar>>;

using DenseQuadraticCost = DenseQuadraticCostTpl<Real>;
using SparseQuadraticCost = SparseQuadraticCostTpl<Real>;

template <typename FunctionTraits>
struct QuadraticCostDataTpl : public CostDataTpl<FunctionTraits> {
    using Scalar = typename FunctionTraits::Scalar;

    using Vector = typename FunctionTraits::OutputVector;
    using Matrix = typename FunctionTraits::OutputMatrix;

    QuadraticCostDataTpl(const QuadraticCostTpl<FunctionTraits> &c)
        : CostDataTpl<FunctionTraits>(c) {
        if constexpr (FunctionTraits::type == "Sparse") {
            // Sparse: allocate sparse objects properly
            A.resize(c.getInputDimension(), c.getInputDimension());
            b.resize(c.getInputDimension(), c.getInputDimension());
        } else {
            // Dense
            A = Matrix::Zero(c.getInputDimension(), c.getInputDimension());
            b = Vector::Zero(c.getInputDimension());
        }
    }

    Matrix A;
    Vector b;
    Scalar c;
};

}  // namespace bopt
