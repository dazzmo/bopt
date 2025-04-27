#pragma once

#include "bopt/ad/casadi/evaluator.hpp"
#include "bopt/constraints.hpp"

namespace bopt {
namespace casadi {

/**
 * @brief Constraint of the form y = fₚ(x) ∈ ℝᵐ
 *
 */
template <typename FunctionTraits>
class ConstraintTpl : public bopt::ConstraintTpl<FunctionTraits> {
    using Base = bopt::ConstraintTpl<FunctionTraits>;

    using Scalar = typename Base::Scalar;

    using InputVector = typename Base::InputVector;
    using InputVectorConstRef = typename Base::InputVectorConstRef;

    using Data = typename Base::Data;

    using ConstraintData = ConstraintDataTpl<FunctionTraits>;

   public:
    ConstraintTpl(const SymbolicVector &expression, const SymbolicVector &x,
                  const SymbolicVector &p, const SymbolicVector &lb,
                  const SymbolicVector &ub, bool codegen = false)
        : bopt::ConstraintTpl<FunctionTraits>(
              std::make_shared<EvaluatorTpl<FunctionTraits>>(expression, x, p,
                                                             codegen),
              ConstraintBounds::CUSTOM) {
        std::vector<SymbolicVector> in;
        in = {p};

        // Jacobian
        SymbolicMatrix ljacp = Symbol::jacobian(lb, p);
        SymbolicMatrix ujacp = Symbol::jacobian(ub, p);

        // Hessian
        SymbolicVector l = Symbol::sym("l", expression.rows());
        SymbolicMatrix lhespp = Symbol::tril(
            Symbol::jacobian(Symbol::gradient(Symbol::dot(l, lb), p), p));
        SymbolicMatrix uhespp = Symbol::tril(
            Symbol::jacobian(Symbol::gradient(Symbol::dot(l, ub), p), p));

        // Create function
        f_bnd = Function("bounds_f", in,
                         {Symbol::densify(lb), Symbol::densify(ub)});

        if constexpr (FunctionTraits::type == "Dense") {
            J_bnd = Function("bounds_jac", in,
                             {Symbol::densify(ljacp), Symbol::densify(ujacp)});

            in = {x, l, p};
            H_bnd =
                Function("bounds_hes", in,
                         {Symbol::densify(lhespp), Symbol::densify(uhespp)});

        } else {
            J_bnd = Function("bounds_jac", in, {ljacp, ujacp});
            in = {x, l, p};
            H_bnd = Function("bounds_hes", in, {lhespp, uhespp});
        }

        // If function is to be code-generated, do so.
        if (codegen) {
            f_bnd = bopt::casadi::codegen(f_bnd);
            J_bnd = bopt::casadi::codegen(J_bnd);
            H_bnd = bopt::casadi::codegen(H_bnd);
        }
    }

   protected:
    void setDataSparsityImpl(Data &data) const override {
        Base::setDataSparsityImpl(data);
        if constexpr (FunctionTraits::type == "Sparse") {
            set_eigen_sparsity(data.Jlb_p, J_bnd.sparsity_out(0));
            set_eigen_sparsity(data.Jub_p, J_bnd.sparsity_out(1));

            set_eigen_sparsity(data.Hlb_pp, H_bnd.sparsity_out(0));
            set_eigen_sparsity(data.Hub_pp, H_bnd.sparsity_out(1));
        }
    }

    void evalBoundsImpl(Data &data) const override {
        std::vector<Scalar *> out(2);
        out[0] = data.lb.data();
        out[1] = data.ub.data();
        f_bnd({this->parameters().data()}, out);
    }

    void evalBoundJacobiansImpl(Data &data) const override {
        std::vector<Scalar *> out(2);
        if constexpr (FunctionTraits::type == "Sparse") {
            out[0] = data.Jlb_p.valuePtr();
            out[1] = data.Jub_p.valuePtr();
        } else {
            out[0] = data.Jlb_p.data();
            out[1] = data.Jub_p.data();
        }
        J_bnd({this->parameters().data()}, out);
    }

    void evalBoundHessiansImpl(const InputVectorConstRef &lambda,
                               Data &data) const override {
        std::vector<Scalar *> out(2);
        if constexpr (FunctionTraits::type == "Sparse") {
            out[0] = data.Hlb_pp.valuePtr();
            out[1] = data.Hub_pp.valuePtr();
        } else {
            out[0] = data.Hlb_pp.data();
            out[1] = data.Hub_pp.data();
        }
        H_bnd({lambda.data(), this->parameters().data()}, out);
    }

   private:
    Function f_bnd;
    Function J_bnd;
    Function H_bnd;
};

template <typename Scalar>
using DenseConstraintTpl = ConstraintTpl<DenseFunctionTraits<Scalar>>;
template <typename Scalar>
using SparseConstraintTpl = ConstraintTpl<SparseFunctionTraits<Scalar>>;

using DenseConstraint = DenseConstraintTpl<Real>;
using SparseConstraint = SparseConstraintTpl<Real>;
/**
 * @brief Constraint of the form lb ≤ Ax ≤ ub
 *
 */
template <typename FunctionTraits>
class LinearConstraintTpl : public bopt::LinearConstraintTpl<FunctionTraits> {
    using Base = bopt::LinearConstraintTpl<FunctionTraits>;

   public:
    using Data = typename Base::Data;

    LinearConstraintTpl(const Symbol &expression, const SymbolicVector &x,
                        const SymbolicVector &p, const SymbolicVector &lb,
                        const SymbolicVector &ub, bool codegen = false)
        : bopt::LinearConstraintTpl<FunctionTraits>(
              std::make_shared<ConstraintTpl<FunctionTraits>>(
                  expression, x, p, lb, ub, codegen)) {
        Symbol A, b;
        // Check expression is linear
        Symbol::linear_coeff(expression, x, A, b, true);
        assert(b.is_zero());

        // Create function
        fA = Function("A", {p}, {Symbol::densify(A)});

        // If function is to be code-generated, do so.
        if (codegen) {
            fA = bopt::casadi::codegen(fA);
        }
    }

   protected:
    virtual void createDataImpl() const {
        auto data = new Data(*this);

        // Update the sparsity patterns
        if constexpr (FunctionTraits::type == "Sparse") {
            set_eigen_sparsity(data->A, fA.sparsity_out(0));
        }

        return data;
    }

    virtual void evalCoefficientsImpl(Data &data) const {
        fA({this->parameters().data()}, {data.A.data()});
    }

   private:
    Function fA;
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
class BoundingBoxConstraintTpl : public ConstraintTpl<FunctionTraits> {
   public:
    BoundingBoxConstraintTpl(const SymbolicVector &x, const SymbolicVector &p,
                             const SymbolicVector &lb, const SymbolicVector &ub,
                             bool codegen = false)
        : ConstraintTpl<FunctionTraits>(x, x, p, lb, ub, codegen) {}

    BoundingBoxConstraintTpl(const SymbolicVector &x, const Real &lb,
                             const Real &ub, bool codegen = false)
        : ConstraintTpl<FunctionTraits>(
              x, x, SymbolicVector(), lb * SymbolicVector::ones(x.size1()),
              ub * SymbolicVector::ones(x.size1()), codegen) {}
};

template <typename Scalar>
using DenseBoundingBoxConstraintTpl =
    BoundingBoxConstraintTpl<DenseFunctionTraits<Scalar>>;
template <typename Scalar>
using SparseBoundingBoxConstraintTpl =
    BoundingBoxConstraintTpl<SparseFunctionTraits<Scalar>>;

using DenseBoundingBoxConstraint = DenseBoundingBoxConstraintTpl<Real>;
using SparseBoundingBoxConstraint = SparseBoundingBoxConstraintTpl<Real>;

}  // namespace casadi
}  // namespace bopt
