#pragma once

#include "bopt/Constraints.hpp"
#include "bopt/ad/casadi/Internal.hpp"

namespace bopt {
namespace casadi {

// Class implementations for constraints and costs
template <typename ScalarType, SparsityType Sparsity = SparsityType::DENSE>
class Constraint
    : public bopt::ConstraintTpl<ScalarType, Sparsity>,
      public internal::FunctionGenerator<ScalarType, Eigen::Dynamic, Sparsity> {
   public:
    using SX = ::casadi::SX;
    using InputVector =
        typename EvaluatorTraits<ScalarType, 1, Sparsity>::InputVectorType;
    using Data = typename bopt::ConstraintTpl<ScalarType, Sparsity>::Data;

    Constraint(const SX &c, const SX &x, const SX &p,
               const ConstraintBoundType &bounds, bool codegen = false)
        : bopt::ConstraintTpl<ScalarType, Sparsity>(
              "casadi_generated_constraint", x.rows(), c.rows(), bounds,
              p.rows()),
          internal::FunctionGenerator<ScalarType, Eigen::Dynamic, Sparsity>(
              c, x, p, codegen) {}

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
        this->computeHessians(x, lambda, this->getParameters(), data.Hxx,
                              data.Hxp, data.Hpp, flags.compute_xx,
                              flags.compute_xp, flags.compute_pp);
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

// // Class implementations for constraints and costs
// template <typename ScalarType, SparsityType Sparsity>
// class LinearConstraint
//     : public bopt::LinearConstraintTpl<ScalarType, Sparsity>,
//       public internal::FunctionGenerator<ScalarType, Eigen::Dynamic,
//       Sparsity>, public internal::LinearFunctionGenerator<ScalarType,
//       Eigen::Dynamic,
//                                                Sparsity> {
//    protected:
//     void evalImpl(const Eigen::Ref<const InputVector> &x,
//                   Data &data) const override {
//         this->compute(x, this->getParameters(), data.y);
//     }

//     void evalJacobiansImpl(
//         const Eigen::Ref<const InputVector> &x, Data &data,
//         const JacobianEvaluationFlags &flags) const override {
//         this->computeJacobians(x, this->getParameters(), data.Jx, data.Jp,
//                                flags.compute_x, flags.compute_p);
//     }

//     void evalHessiansImpl(const Eigen::Ref<const InputVector> &x, Data &data,
//                           const HessianEvaluationFlags &flags) const override
//                           {
//         this->computeHessians(x, this->getParameters(), data.Hxx, data.Hxp,
//                               data.Hpp, flags.compute_xx, flags.compute_xp,
//                               flags.compute_pp);
//     }

//     void setupDataSparsityImpl(Data &data) const override {
//         if constexpr (Sparsity == SparsityType::SPARSE) {
//             setupSparseEigenMatrix(data.Jx, this->J.sparsity_out(0));
//             setupSparseEigenMatrix(data.Jp, this->J.sparsity_out(1));
//             setupSparseEigenMatrix(data.Hxx, this->H.sparsity_out(0));
//             setupSparseEigenMatrix(data.Hxp, this->H.sparsity_out(1));
//             setupSparseEigenMatrix(data.Hpp, this->H.sparsity_out(2));
//         }
//     }
// };

}  // namespace casadi
}  // namespace bopt
