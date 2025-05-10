#pragma once

#include "bopt/ad/casadi/utils.hpp"
#include "bopt/Evaluator.hpp"

namespace bopt {
namespace casadi {

template <typename FunctionTraits>
class EvaluatorTpl : public bopt::EvaluatorTpl<FunctionTraits> {
   public:
    using Base = bopt::EvaluatorTpl<FunctionTraits>;

    using Scalar = typename Base::Scalar;

    using InputVector = typename Base::InputVector;
    using InputVectorConstRef = typename Base::InputVectorConstRef;

    using Data = typename Base::Data;

    // EvaluatorTpl(const std::string &hash) {}

    EvaluatorTpl(const SymbolicVector &expression, const SymbolicVector &x,
                 const SymbolicVector &p, bool codegen = false)
        : Base(x.rows(), expression.rows(), "CasADi generated evaluator") {
        // Set up variables
        this->setOutputDimension(expression.rows());
        this->setTangentSpaceDimension(x.rows());
        this->setParameterDimension(p.rows());

        std::vector<SymbolicVector> in;

        in = {x, p};

        // Jacobian
        SymbolicMatrix jacx = Symbol::jacobian(expression, x);
        SymbolicMatrix jacp = Symbol::jacobian(expression, p);

        // Hessian
        SymbolicVector l = Symbol::sym("l", expression.rows());
        SymbolicMatrix hesxx = Symbol::tril(Symbol::jacobian(
            Symbol::gradient(Symbol::dot(l, expression), x), x));
        SymbolicMatrix hesxp = Symbol::tril(Symbol::jacobian(
            Symbol::gradient(Symbol::dot(l, expression), x), p));
        SymbolicMatrix hespp = Symbol::tril(Symbol::jacobian(
            Symbol::gradient(Symbol::dot(l, expression), p), p));

        // Create function
        f = Function("f", in, {expression});

        if constexpr (FunctionTraits::type == "Dense") {
            // Create jacobians
            J = Function("jacobian", in,
                         {Symbol::densify(jacx), Symbol::densify(jacp)});

            in = {x, l, p};
            H = Function("hessian", in,
                         {Symbol::densify(hesxx), Symbol::densify(hesxp),
                          Symbol::densify(hespp)});
        } else {
            // Create jacobians
            J = Function("jacobian", in, {jacx, jacp});
            in = {x, l, p};
            H = Function("hessian", in, {hesxx, hesxp, hespp});
        }

        // If function is to be code-generated, do so.
        if (codegen) {
            f = bopt::casadi::codegen(f);
            J = bopt::casadi::codegen(J);
            H = bopt::casadi::codegen(H);
        }
    }

   protected:
    /**
     * @brief Implementation of the evaluator
     *
     * @param x
     * @param out
     */
    virtual void evalImpl(const InputVectorConstRef &x,
                          Data &data) const override {
        f({x.data(), this->parameters().data()}, {data.y.data()});
    }

    /**
     * \copydoc EvaluatorTpl::evalJacobians(const Eigen::Ref<const
     * VectorX<Scalar>>, Data &)
     *
     */
    void evalJacobiansImpl(const InputVectorConstRef &x, Data &data,
                           bool compute_x, bool compute_p) const override {
        std::vector<Scalar *> out = {nullptr, nullptr};
        if constexpr (FunctionTraits::type == "Sparse") {
            if (compute_x) out[0] = data.Jx.valuePtr();
            if (compute_p) out[1] = data.Jp.valuePtr();
        } else {
            if (compute_x) out[0] = data.Jx.data();
            if (compute_p) out[1] = data.Jp.data();
        }

        J({x.data(), this->parameters().data()}, out);
    }

    /**
     * \copydoc EvaluatorTpl::evalHessians(const Eigen::Ref<const
     * VectorX<Scalar>>, const InputVectorConstRef, Data
     * &)
     *
     */
    void evalHessiansImpl(const InputVectorConstRef &x,
                          const InputVectorConstRef &lambda, Data &data,
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

        H({x.data(), lambda.data(), this->parameters().data()}, out);
    }

    // Sparsity patterns
    void setDataSparsityImpl(Data &data) const override {
        if constexpr (FunctionTraits::type == "Sparse") {
            setupSparseEigenMatrix(data.Jx, J.sparsity_out(0));
            setupSparseEigenMatrix(data.Jp, J.sparsity_out(1));
            setupSparseEigenMatrix(data.Hxx, H.sparsity_out(0));
            setupSparseEigenMatrix(data.Hxp, H.sparsity_out(1));
            setupSparseEigenMatrix(data.Hpp, H.sparsity_out(2));
        }
    }

   private:
    Function f;
    Function J;
    Function H;
};

template <typename Scalar>
using DenseEvaluatorTpl = EvaluatorTpl<DenseFunctionTraits<Scalar>>;

template <typename Scalar>
using SparseEvaluatorTpl = EvaluatorTpl<SparseFunctionTraits<Scalar>>;

using DenseEvaluator = DenseEvaluatorTpl<Real>;
using SparseEvaluator = SparseEvaluatorTpl<Real>;

}  // namespace casadi
}  // namespace bopt