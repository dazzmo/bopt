#pragma once

#include <memory>

#include "bopt/ad/casadi/utils.hpp"
#include "bopt/costs.hpp"

namespace bopt {
namespace casadi {

/**
 *
 * @brief Cost function y = fₚ(x) ∈ ℝ
 *
 */
template <typename FunctionTraits>
class CostTpl : public bopt::CostTpl<FunctionTraits> {
    using Base = bopt::CostTpl<FunctionTraits>;

   public:
    using Scalar = typename Base::Scalar;

    using InputVector = typename Base::InputVector;
    using InputVectorConstRef = typename Base::InputVectorConstRef;

    using Data = typename Base::Data;

    /**
     * @brief Construct a new cost from a casadi expression
     *
     * @param expression
     * @param x
     * @param p
     * @param codegen Perform codegeneration
     * @param dense Create the dense expressions
     * @param sparse Create the sparse expressions
     */
    CostTpl(const Symbol &expression, const SymbolicVector &x,
            const SymbolicVector &p, bool codegen = false)
        : bopt::CostTpl<FunctionTraits>(x.rows(), "casadi generated cost") {
        assert(expression.is_scalar());

        // Set up variables
        this->setTangentSpaceDimension(x.rows());
        this->setParameterDimension(p.rows());

        std::vector<SymbolicVector> in;

        in = {x, p};

        // Jacobian
        SymbolicMatrix grdx = Symbol::gradient(expression, x);
        SymbolicMatrix grdp = Symbol::gradient(expression, p);

        // Hessian
        SymbolicMatrix hesxx = Symbol::tril(Symbol::jacobian(grdx, x));
        SymbolicMatrix hesxp = Symbol::tril(Symbol::jacobian(grdx, p));
        SymbolicMatrix hespp = Symbol::tril(Symbol::jacobian(grdp, p));

        // Create function
        f = Function("f", in, {expression});

        if constexpr (FunctionTraits::type == "Dense") {
            // Create jacobians
            g = Function("gradient", in,
                         {Symbol::densify(grdx), Symbol::densify(grdp)});

            H = Function("hessian", in,
                         {Symbol::densify(hesxx), Symbol::densify(hesxp),
                          Symbol::densify(hespp)});
        } else {
            // Create jacobians
            g = Function("gradient", in, {grdx, grdp});
            H = Function("hessian", in, {hesxx, hesxp, hespp});
        }

        // If function is to be code-generated, do so.
        if (codegen) {
            f = bopt::casadi::codegen(f);
            g = bopt::casadi::codegen(g);
            H = bopt::casadi::codegen(H);
        }
    }

   protected:
    // Sparsity patterns
    Data *createDataImpl() const override {
        auto data = new Data(*this);

        // Update the sparsity patterns
        if constexpr (FunctionTraits::type == "Sparse") {
            setupSparseEigenMatrix(data->gx, g.sparsity_out(0));
            setupSparseEigenMatrix(data->gp, g.sparsity_out(1));
            setupSparseEigenMatrix(data->Hxx, H.sparsity_out(0));
            setupSparseEigenMatrix(data->Hxp, H.sparsity_out(1));
            setupSparseEigenMatrix(data->Hpp, H.sparsity_out(2));
        }

        return data;
    }

    /**
     * @brief Implementation of the evaluator
     *
     * @param x
     * @param out
     */
    void evalImpl(const InputVectorConstRef &x, Data &data) const override {
        f({x.data(), this->parameters().data()}, {&data.y});
    }

    /**
     * \copydoc CostTpl::evalGradients(const Eigen::Ref<const
     * VectorXd>, Data &)
     *
     */
    void evalGradientsImpl(const InputVectorConstRef &x, Data &data,
                           bool compute_x, bool compute_p) const override {
        std::vector<Scalar *> out = {nullptr, nullptr};
        if constexpr (FunctionTraits::type == "Sparse") {
            if (compute_x) out[0] = data.gx.valuePtr();
            if (compute_p) out[1] = data.gp.valuePtr();
        } else {
            if (compute_x) out[0] = data.gx.data();
            if (compute_p) out[1] = data.gp.data();
        }
        g({x.data(), this->parameters().data()}, out);
    }

    /**
     * \copydoc CostTpl::evalHessians(const InputVectorConstRef,
     * const InputVectorConstRef, Data &)
     *
     */
    void evalHessiansImpl(const InputVectorConstRef &x, Data &data,
                          bool compute_xx, bool compute_xp,
                          bool compute_pp) const override {
        std::vector<Scalar *> out = {nullptr, nullptr, nullptr};
        if constexpr (FunctionTraits::type == "Sparse") {
            if (compute_xx) out[0] = data.Hxx.valuePtr();
            if (compute_xp) out[1] = data.Hxp.valuePtr();
            if (compute_pp) out[2] = data.Hpp.valuePtr();
        } else {
            if (compute_xx) out[0] = data.Hxx.data();
            if (compute_xp) out[1] = data.Hxp.data();
            if (compute_pp) out[2] = data.Hpp.data();
        }
        H({x.data(), this->parameters().data()}, out);
    }

   private:
    Function f;
    Function g;
    Function H;
};

template <typename Scalar>
using DenseCostTpl = CostTpl<DenseFunctionTraits<Scalar>>;
template <typename Scalar>
using SparseCostTpl = CostTpl<SparseFunctionTraits<Scalar>>;

using DenseCost = DenseCostTpl<Real>;
using SparseCost = SparseCostTpl<Real>;

/**
 * @brief Linear cost of the form fₚ(x) = aₚᵀx + bₚ
 *
 */
template <typename FunctionTraits>
class LinearCostTpl : public bopt::LinearCostTpl<FunctionTraits> {
    using Base = bopt::LinearCostTpl<FunctionTraits>;

   public:
    using Data = typename Base::Data;

    LinearCostTpl(const Symbol &expression, const SymbolicVector &x,
                  const SymbolicVector &p, bool codegen = false)
        : bopt::LinearCostTpl<FunctionTraits>(
              std::make_shared<CostTpl<FunctionTraits>>(expression, x, p,
                                                        codegen)) {
        SymbolicVector a;
        Symbol b;
        // Check expression is linear
        Symbol::linear_coeff(expression, x, a, b, true);

        // Create function
        if constexpr (FunctionTraits::type == "Dense") {
            fa = Function("a", {p}, {Symbol::densify(a)});
        } else {
            fa = Function("a", {p}, {a});
        }
        fb = Function("b", {p}, {Symbol::densify(b)});

        // If function is to be code-generated, do so.
        if (codegen) {
            fa = bopt::casadi::codegen(fa);
            fb = bopt::casadi::codegen(fb);
        }
    }

   protected:
    void evalCoefficientsImpl(Data &data) const override {
        fa({this->parameters().data()}, {data.a.data()});
        fb({this->parameters().data()}, {&data.b});
    }

   private:
    Function fa;
    Function fb;
};

template <typename Scalar>
using DenseLinearCostTpl = LinearCostTpl<DenseFunctionTraits<Scalar>>;
template <typename Scalar>
using SparseLinearCostTpl = LinearCostTpl<SparseFunctionTraits<Scalar>>;

using DenseLinearCost = DenseLinearCostTpl<Real>;
using SparseLinearCost = SparseLinearCostTpl<Real>;

/**
 * @brief Quadratic cost of the form fₚ(x) = (1/2) xᵀ Aₚ x + bₚᵀ x + cₚ
 *
 */
template <typename FunctionTraits>
class QuadraticCostTpl : public bopt::QuadraticCostTpl<FunctionTraits> {
    using Base = bopt::QuadraticCostTpl<FunctionTraits>;

   public:
    using Data = typename Base::Data;

    QuadraticCostTpl(const Symbol &expression, const SymbolicVector &x,
                     const SymbolicVector &p, bool codegen = false)
        : bopt::QuadraticCostTpl<FunctionTraits>(
              std::make_shared<CostTpl<FunctionTraits>>(expression, x, p,
                                                        codegen)) {
        SymbolicMatrix A;
        SymbolicVector b;
        Symbol c;
        // Check expression is linear
        Symbol::quadratic_coeff(expression, x, A, b, c, true);
        // Make A lower triangular
        A = Symbol::tril(A);

        // Create function
        if constexpr (FunctionTraits::type == "Dense") {
            fA = Function("A", {p}, {Symbol::densify(A)});
            fb = Function("b", {p}, {Symbol::densify(b)});
        } else {
            fA = Function("A", {p}, {A});
            fb = Function("b", {p}, {b});
        }

        fc = Function("c", {p}, {Symbol::densify(c)});

        // If function is to be code-generated, do so.
        if (codegen) {
            fA = bopt::casadi::codegen(fA);
            fb = bopt::casadi::codegen(fb);
            fc = bopt::casadi::codegen(fc);
        }
    }

   protected:
    void evalCoefficientsImpl(Data &data) const override {
        fA({this->parameters().data()}, {data.A.data()});
        fb({this->parameters().data()}, {data.b.data()});
        fc({this->parameters().data()}, {&data.c});
    }

   private:
    Function fA;
    Function fb;
    Function fc;
};

template <typename Scalar>
using DenseQuadraticCostTpl = QuadraticCostTpl<DenseFunctionTraits<Scalar>>;
template <typename Scalar>
using SparseQuadraticCostTpl = QuadraticCostTpl<SparseFunctionTraits<Scalar>>;

using DenseQuadraticCost = DenseQuadraticCostTpl<Real>;
using SparseQuadraticCost = SparseQuadraticCostTpl<Real>;

}  // namespace casadi
}  // namespace bopt