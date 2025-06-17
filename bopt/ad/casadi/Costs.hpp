#pragma once

#include "bopt/Costs.hpp"
#include "bopt/ad/casadi/Internal.hpp"

namespace bopt {
namespace casadi {

// Class implementations for constraints and costs
template <typename ScalarType, SparsityType Sparsity = SparsityType::DENSE>
class Cost : public bopt::CostTpl<ScalarType, Sparsity>,
             public internal::FunctionGenerator<ScalarType, 1, Sparsity> {
   public:
    using SX = ::casadi::SX;
    using InputVector =
        typename EvaluatorTraits<ScalarType, 1, Sparsity>::InputVectorType;
    using Data = typename bopt::CostTpl<ScalarType, Sparsity>::Data;

    Cost(const SX &f, const SX &x, const SX &p, bool codegen = false)
        : bopt::CostTpl<ScalarType, Sparsity>("casadi_generated_cost", x.rows(),
                                              p.rows()),
          internal::FunctionGenerator<ScalarType, 1, Sparsity>(f, x, p,
                                                               codegen) {}

   protected:
    void evalImpl(const Eigen::Ref<const InputVector> &x,
                  Data &data) const override {
        this->compute(x, this->getParameters(), data.y);
    }

    void evalGradientsImpl(
        const Eigen::Ref<const InputVector> &x, Data &data,
        const GradientEvaluationFlags &flags) const override {
        this->computeGradients(x, this->getParameters(), data.gx, data.gp,
                               flags.compute_x, flags.compute_p);
    }

    void evalHessiansImpl(const Eigen::Ref<const InputVector> &x, Data &data,
                          const HessianEvaluationFlags &flags) const override {
        this->computeHessians(x, this->getParameters(), data.Hxx, data.Hxp,
                              data.Hpp, flags.compute_xx, flags.compute_xp,
                              flags.compute_pp);
    }

    void setupDataSparsityImpl(Data &data) const override {
        if constexpr (Sparsity == SparsityType::SPARSE) {
            setupSparseEigenMatrix(data.gx, this->g.sparsity_out(0));
            setupSparseEigenMatrix(data.gp, this->g.sparsity_out(1));
            setupSparseEigenMatrix(data.Hxx, this->H.sparsity_out(0));
            setupSparseEigenMatrix(data.Hxp, this->H.sparsity_out(1));
            setupSparseEigenMatrix(data.Hpp, this->H.sparsity_out(2));
        }
    }
};

template <typename ScalarType, SparsityType Sparsity = SparsityType::DENSE>
class LinearCost
    : public bopt::LinearCostTpl<ScalarType, Sparsity>,
      public internal::FunctionGenerator<ScalarType, 1, Sparsity>,
      public internal::LinearFunctionGenerator<ScalarType, 1, Sparsity> {
   public:
    using SX = ::casadi::SX;
    using InputVector =
        typename EvaluatorTraits<ScalarType, 1, Sparsity>::InputVectorType;
    using Data = typename bopt::LinearCostTpl<ScalarType, Sparsity>::Data;
    using CostBaseData =
        typename bopt::LinearCostTpl<ScalarType, Sparsity>::CostBase::Data;

    LinearCost(const SX &f, const SX &x, const SX &p, bool codegen = false)
        : bopt::LinearCostTpl<ScalarType, Sparsity>(
              "casadi_generated_linear_cost", x.rows(), p.rows()),
          internal::FunctionGenerator<ScalarType, 1, Sparsity>(f, x, p,
                                                               codegen),
          internal::LinearFunctionGenerator<ScalarType, 1, Sparsity>(f, x, p,
                                                                     codegen) {}

   protected:
    void evalImpl(const Eigen::Ref<const InputVector> &x,
                  CostBaseData &data) const override {
        this->compute(x, this->getParameters(), data.y);
    }

    void evalGradientsImpl(
        const Eigen::Ref<const InputVector> &x, CostBaseData &data,
        const GradientEvaluationFlags &flags) const override {
        this->computeGradients(x, this->getParameters(), data.gx, data.gp,
                               flags.compute_x, flags.compute_p);
    }

    void evalHessiansImpl(const Eigen::Ref<const InputVector> &x,
                          CostBaseData &data,
                          const HessianEvaluationFlags &flags) const override {
        this->computeHessians(x, this->getParameters(), data.Hxx, data.Hxp,
                              data.Hpp, flags.compute_xx, flags.compute_xp,
                              flags.compute_pp);
    }

    void evalCoefficientsImpl(Data &data) const override {
        this->computeLinearCoefficients(this->getParameters(), data.a, data.b);
    }

    void setupDataSparsityImpl(CostBaseData &data) const override {
        if constexpr (Sparsity == SparsityType::SPARSE) {
            setupSparseEigenMatrix(data.gx, this->g.sparsity_out(0));
            setupSparseEigenMatrix(data.gp, this->g.sparsity_out(1));
            setupSparseEigenMatrix(data.Hxx, this->H.sparsity_out(0));
            setupSparseEigenMatrix(data.Hxp, this->H.sparsity_out(1));
            setupSparseEigenMatrix(data.Hpp, this->H.sparsity_out(2));
        }
    }

    void setupDataSparsityImpl(Data &data) const override {
        if constexpr (Sparsity == SparsityType::SPARSE) {
            setupSparseEigenMatrix(data.a, this->coefficients.sparsity_out(0));
        }
        setupDataSparsityImpl(static_cast<CostBaseData &>(data));
    }
};

template <typename ScalarType, SparsityType Sparsity = SparsityType::DENSE>
class QuadraticCost
    : public bopt::QuadraticCostTpl<ScalarType, Sparsity>,
      public internal::FunctionGenerator<ScalarType, 1, Sparsity>,
      public internal::QuadraticFunctionGenerator<ScalarType, Sparsity> {
   public:
    using SX = ::casadi::SX;
    using InputVector =
        typename EvaluatorTraits<ScalarType, 1, Sparsity>::InputVectorType;
    using Data = typename bopt::QuadraticCostTpl<ScalarType, Sparsity>::Data;
    using CostBaseData =
        typename bopt::QuadraticCostTpl<ScalarType, Sparsity>::CostBase::Data;

    QuadraticCost(const SX &f, const SX &x, const SX &p, bool codegen = false)
        : bopt::QuadraticCostTpl<ScalarType, Sparsity>(
              "casadi_generated_quadratic_cost", x.rows(), p.rows()),
          internal::FunctionGenerator<ScalarType, 1, Sparsity>(f, x, p,
                                                               codegen),
          internal::QuadraticFunctionGenerator<ScalarType, Sparsity>(f, x, p,
                                                                     codegen) {}

   protected:
    void evalImpl(const Eigen::Ref<const InputVector> &x,
                  CostBaseData &data) const override {
        this->compute(x, this->getParameters(), data.y);
    }

    void evalGradientsImpl(
        const Eigen::Ref<const InputVector> &x, CostBaseData &data,
        const GradientEvaluationFlags &flags) const override {
        this->computeGradients(x, this->getParameters(), data.gx, data.gp,
                               flags.compute_x, flags.compute_p);
    }

    void evalHessiansImpl(const Eigen::Ref<const InputVector> &x,
                          CostBaseData &data,
                          const HessianEvaluationFlags &flags) const override {
        this->computeHessians(x, this->getParameters(), data.Hxx, data.Hxp,
                              data.Hpp, flags.compute_xx, flags.compute_xp,
                              flags.compute_pp);
    }

    void evalCoefficientsImpl(Data &data) const override {
        this->computeQuadraticCoefficients(this->getParameters(), data.A,
                                           data.b, data.c);
    }

    void setupDataSparsityImpl(CostBaseData &data) const override {
        if constexpr (Sparsity == SparsityType::SPARSE) {
            setupSparseEigenMatrix(data.gx, this->g.sparsity_out(0));
            setupSparseEigenMatrix(data.gp, this->g.sparsity_out(1));
            setupSparseEigenMatrix(data.Hxx, this->H.sparsity_out(0));
            setupSparseEigenMatrix(data.Hxp, this->H.sparsity_out(1));
            setupSparseEigenMatrix(data.Hpp, this->H.sparsity_out(2));
        }
    }

    void setupDataSparsityImpl(Data &data) const override {
        if constexpr (Sparsity == SparsityType::SPARSE) {
            setupSparseEigenMatrix(data.A, this->coefficients.sparsity_out(0));
            setupSparseEigenMatrix(data.b, this->coefficients.sparsity_out(1));
        }
        setupDataSparsityImpl(static_cast<CostBaseData &>(data));
    }
};

}  // namespace casadi
}  // namespace bopt
