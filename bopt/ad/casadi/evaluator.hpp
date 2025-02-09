#pragma once

#include "bopt/ad/casadi/utils.hpp"
#include "bopt/evaluator.hpp"

namespace bopt {
namespace casadi {

template <typename Scalar>
class EvaluatorTpl : public bopt::EvaluatorTpl<Scalar> {
   public:
    EvaluatorTpl(const sym_t &expression, const sym_vector_t &x,
                 const sym_vector_t &p, bool codegen = false)
        : bopt::EvaluatorTpl<Scalar>(x.rows(), expression.rows(),
                                     "CasADi generated evaluator") {
        // Set up variables
        this->setOutputDimension(expression.rows());
        this->setTangentSpaceDimension(x.rows());
        this->setParameterDimension(p.rows());

        std::vector<sym_vector_t> in;

        in = {x, p};

        // Jacobian
        sym_t jacx = sym_t::jacobian(expression, x);
        sym_t jacp = sym_t::jacobian(expression, p);

        // Hessian
        sym_vector_t l = sym_t::sym("l", expression.rows());
        sym_t hesxx = sym_t::tril(
            sym_t::jacobian(sym_t::gradient(sym_t::dot(l, expression), x), x));
        sym_t hesxp = sym_t::tril(
            sym_t::jacobian(sym_t::gradient(sym_t::dot(l, expression), x), p));
        sym_t hespp = sym_t::tril(
            sym_t::jacobian(sym_t::gradient(sym_t::dot(l, expression), p), p));

        // Create function
        f = function_t("f", in, {expression});
        // Create jacobians
        J_s = function_t("sparse_jac", in, {jacx, jacp});
        J = function_t("dense_jac", in,
                       {sym_t::densify(jacx), sym_t::densify(jacp)});

        in = {x, l, p};
        H_s = function_t("sparse_hes", in, {hesxx, hesxp, hespp});
        H = function_t("dense_hes", in,
                       {sym_t::densify(hesxx), sym_t::densify(hesxp),
                        sym_t::densify(hespp)});

        // If function is to be code-generated, do so.
        if (codegen) {
            f = bopt::casadi::codegen(f);
            J = bopt::casadi::codegen(J);
            J_s = bopt::casadi::codegen(J_s);
            H = bopt::casadi::codegen(H);
            H_s = bopt::casadi::codegen(H_s);
        }
    }

    // Sparsity patterns

    virtual void setJacobianSparsityPatterns(EvaluatorData &data) const {
        set_eigen_sparsity(data.Jx_s, J_s.sparsity_out(0));
        set_eigen_sparsity(data.Jp_s, J_s.sparsity_out(1));
    }

    virtual void setHessianSparsityPatterns(EvaluatorData &data) const {
        set_eigen_sparsity(data.Hxx_s, H_s.sparsity_out(0));
        set_eigen_sparsity(data.Hxp_s, H_s.sparsity_out(1));
        set_eigen_sparsity(data.Hpp_s, H_s.sparsity_out(2));
    }

   protected:
    /**
     * @brief Implementation of the evaluator
     *
     * @param x
     * @param out
     */
    virtual void evalImpl(const Eigen::Ref<const VectorX<Scalar>> &x,
                          EvaluatorData &data) const override {
        f({x.data(), this->parameters().data()}, {data.y.data()});
    }

    /**
     * \copydoc EvaluatorTpl::evalJacobians(const Eigen::Ref<const
     * VectorX<Scalar>>, EvaluatorData &)
     *
     */
    virtual void evalJacobiansImpl(const Eigen::Ref<const VectorX<Scalar>> &x,
                                   EvaluatorData &data, bool compute_x,
                                   bool compute_p) const override {
        std::vector<Scalar *> out = {nullptr, nullptr};
        if (compute_x) out[0] = data.Jx.data();
        if (compute_p) out[1] = data.Jp.data();
        J({x.data(), this->parameters().data()}, out);
    }

    /**
     * \copydoc EvaluatorTpl::evalSparseJacobians(const Eigen::Ref<const
     * VectorX<Scalar>>, EvaluatorData &)
     *
     */
    virtual void evalSparseJacobiansImpl(
        const Eigen::Ref<const VectorX<Scalar>> &x, EvaluatorData &data,
        bool compute_x, bool compute_p) const override {
        std::vector<Scalar *> out = {nullptr, nullptr};
        if (compute_x) out[0] = data.Jx_s.valuePtr();
        if (compute_p) out[1] = data.Jp_s.valuePtr();
        J_s({x.data(), this->parameters().data()}, out);
    }

    /**
     * \copydoc EvaluatorTpl::evalHessians(const Eigen::Ref<const
     * VectorX<Scalar>>, const Eigen::Ref<const VectorX<Scalar>>, EvaluatorData
     * &)
     *
     */
    virtual void evalHessiansImpl(
        const Eigen::Ref<const VectorX<Scalar>> &x,
        const Eigen::Ref<const VectorX<Scalar>> &lambda, EvaluatorData &data,
        bool compute_xx, bool compute_xp, bool compute_pp) const override {
        std::vector<Scalar *> out = {nullptr, nullptr, nullptr};
        if (compute_xx) out[0] = data.Hxx.data();
        if (compute_xp) out[1] = data.Hxp.data();
        if (compute_pp) out[2] = data.Hpp.data();
        H({x.data(), lambda.data(), this->parameters().data()}, out);
    }

    /**
     * \copydoc EvaluatorTpl::evalSparseHessians(const Eigen::Ref<const
     * VectorX<Scalar>>, const Eigen::Ref<const VectorX<Scalar>>, EvaluatorData
     * &)
     *
     */
    virtual void evalSparseHessiansImpl(
        const Eigen::Ref<const VectorX<Scalar>> &x,
        const Eigen::Ref<const VectorX<Scalar>> &lambda, EvaluatorData &data,
        bool compute_xx, bool compute_xp, bool compute_pp) const override {
        std::vector<Scalar *> out = {nullptr, nullptr, nullptr};
        if (compute_xx) out[0] = data.Hxx_s.valuePtr();
        if (compute_xp) out[1] = data.Hxp_s.valuePtr();
        if (compute_pp) out[2] = data.Hpp_s.valuePtr();
        H_s({x.data(), lambda.data(), this->parameters().data()}, out);
    }

   private:
    function_t f;

    function_t J;
    function_t J_s;

    function_t H;
    function_t H_s;
};

typedef EvaluatorTpl<double> Evaluator;

}  // namespace casadi
}  // namespace bopt