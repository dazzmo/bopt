#pragma once

#include "bopt/Evaluator.hpp"
#include "bopt/ad/casadi/Internal.hpp"

namespace bopt {
namespace casadi {

template <typename ScalarType, int OutputSizeAtCompileTime = Eigen::Dynamic,
          SparsityType Sparsity = SparsityType::DENSE>
class Evaluator
    : public bopt::EvaluatorTpl<ScalarType, OutputSizeAtCompileTime, Sparsity>,
      public internal::FunctionGenerator<ScalarType, OutputSizeAtCompileTime,
                                         Sparsity> {
   public:
    using Base =
        bopt::EvaluatorTpl<ScalarType, OutputSizeAtCompileTime, Sparsity>;
    static constexpr bool IsOutputScalar = Base::IsOutputScalar;

    using InputVector = typename Base::InputVector;

    using Data = typename Base::Data;

    Evaluator(const SymbolicVector &expression, const SymbolicVector &x,
              const SymbolicVector &p, bool codegen = false)
        : Base(x.rows(), expression.rows(), p.rows(),
               "CasADi generated evaluator"),
          internal::FunctionGenerator<ScalarType, OutputSizeAtCompileTime,
                                      Sparsity>(expression, x, p, codegen) {
        this->setNumParameters(p.rows());
    }

   protected:
    void evalImpl(const Eigen::Ref<const InputVector> &x,
                  Data &data) const override {
        this->compute(x, this->getParameters(), data.y);
    }

    void evalJacobiansImpl(
        const Eigen::Ref<const InputVector> &x, Data &data,
        const JacobianEvaluationFlags &flags) const override {
        this->computeJacobians(x, this->getParameters(), data.Jx, data.Jp,
                               flags.compute_x, flags.compute_p);
    }

    void evalHessiansImpl(const Eigen::Ref<const InputVector> &x,
                          const Eigen::Ref<const InputVector> &lambda,
                          Data &data,
                          const HessianEvaluationFlags &flags) const override {
        if constexpr (!IsOutputScalar) {
            this->computeHessians(x, lambda, this->getParameters(), data.Hxx,
                                  data.Hxp, data.Hpp, flags.compute_xx,
                                  flags.compute_xp, flags.compute_pp);
        }
    }

    void setupDataSparsityImpl(Data &data) const override {
        if constexpr (Sparsity == SparsityType::SPARSE) {
            setupSparseEigenMatrix(data.Jx, this->J.sparsity_out(0));
            setupSparseEigenMatrix(data.Jp, this->J.sparsity_out(1));
            setupSparseEigenMatrix(data.Hxx, this->H.sparsity_out(0));
            setupSparseEigenMatrix(data.Hxp, this->H.sparsity_out(1));
            setupSparseEigenMatrix(data.Hpp, this->H.sparsity_out(2));
        }
    }
};

template <typename ScalarType, SparsityType Sparsity>
class Evaluator<ScalarType, 1, Sparsity>
    : public bopt::EvaluatorTpl<ScalarType, 1, Sparsity>,
      public internal::FunctionGenerator<ScalarType, 1, Sparsity> {
   public:
    using Base = bopt::EvaluatorTpl<ScalarType, 1, Sparsity>;

    using InputVector = typename Base::InputVector;

    using Data = typename Base::Data;

    Evaluator(const SymbolicVector &expression, const SymbolicVector &x,
              const SymbolicVector &p, bool codegen = false)
        : Base(x.rows(), p.rows(), "CasADi generated evaluator"),
          internal::FunctionGenerator<ScalarType, 1, Sparsity>(expression, x, p,
                                                               codegen) {
        this->setNumParameters(p.rows());
    }

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

}  // namespace casadi
}  // namespace bopt
