#pragma once

#include "bopt/costs/CostBase.hpp"

namespace bopt {

/**
 * @brief Types of hessians
 *
 */
enum class HessianType { POSITIVE_DEFINITE, POSITIVE_SEMIDEFINITE, INDEFINITE };

template <typename EvaluatorTraits>
struct QuadraticCostDataTpl;

/**
 * @brief Quadratic cost of the form fₚ(x) = (1/2) xᵀ Aₚ x + bₚᵀ x + cₚ
 *
 */
template <typename EvaluatorTraits>
class QuadraticCostTpl : public CostTpl<EvaluatorTraits> {
   public:
    using Data = QuadraticCostDataTpl<EvaluatorTraits>;

    void setDataSparsity(Data &data) const { setDataSparsityImpl(data); }

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
    QuadraticCostTpl<EvaluatorTraits>(const Index &dim_input)
        : CostTpl<EvaluatorTraits>(dim_input) {
        this->setName("quadratic_cost");
    }

    QuadraticCostTpl<EvaluatorTraits>(
        const std::shared_ptr<QuadraticCostTpl<EvaluatorTraits>> &cost)
        : CostTpl<EvaluatorTraits>(cost) {
        this->setName("quadratic_cost");
    }

    virtual void evalCoefficientsImpl(Data &data) const {
        if (ptr_) ptr_->evalCoefficients(data);
    }
    virtual void setDataSparsityImpl(Data &data) const {
        if (ptr_) ptr_->setDataSparsity(data);
    }

   private:
};

template <typename Scalar>
using DenseQuadraticCostTpl = QuadraticCostTpl<DenseEvaluatorTraits<Scalar>>;
using DenseQuadraticCost = DenseQuadraticCostTpl<Real>;

template <typename Scalar>
using SparseQuadraticCostTpl = QuadraticCostTpl<SparseEvaluatorTraits<Scalar>>;
using SparseQuadraticCost = SparseQuadraticCostTpl<Real>;

template <typename EvaluatorTraits>
struct QuadraticCostDataTpl : public CostDataTpl<EvaluatorTraits> {
    using Scalar = typename EvaluatorTraits::Scalar;
    using Vector = typename EvaluatorTraits::OutputVector;
    using Matrix = typename EvaluatorTraits::OutputMatrix;

    QuadraticCostDataTpl(const QuadraticCostTpl<EvaluatorTraits> &c)
        : CostDataTpl<EvaluatorTraits>(c) {
        if constexpr (EvaluatorTraits::type == "Sparse") {
            // Sparse: allocate sparse objects properly
            A.resize(c.numInputs(), c.numInputs());
            b.resize(c.numInputs(), c.numInputs());
        } else {
            // Dense
            A = Matrix::Zero(c.numInputs(), c.numInputs());
            b = Vector::Zero(c.numInputs());
        }
    }

    Matrix A;
    Vector b;
    Scalar c;
};

}  // namespace bopt
