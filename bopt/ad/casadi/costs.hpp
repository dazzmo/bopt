#pragma once

#include <memory>

#include "bopt/costs.hpp"

namespace bopt {
namespace casadi {

/**
 *
 * @brief Cost function y = fₚ(x) ∈ ℝ
 *
 */
template <typename Scalar>
class CostTpl : public bopt::CostTpl<Scalar> {
   public:
    using CostData = CostDataTpl<Scalar>;

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
    CostTpl(const sym_t &expression, const sym_vector_t &x,
            const sym_vector_t &p, bool codegen = false, bool dense = true,
            bool sparse = true)
        : bopt::CostTpl<Scalar>(x.rows(), "casadi generated cost") {
        assert(expression.is_scalar());

        // Set up variables
        this->setTangentSpaceDimension(x.rows());
        this->setParameterDimension(p.rows());

        std::vector<sym_vector_t> in;

        in = {x, p};

        // Jacobian
        sym_t grdx = sym_t::gradient(expression, x);
        sym_t grdp = sym_t::gradient(expression, p);

        // Hessian
        sym_t hesxx =
            sym_t::tril(sym_t::jacobian(sym_t::gradient(expression, x), x));
        sym_t hesxp =
            sym_t::tril(sym_t::jacobian(sym_t::gradient(expression, x), p));
        sym_t hespp =
            sym_t::tril(sym_t::jacobian(sym_t::gradient(expression, p), p));

        // Create function
        f = function_t("f", in, {expression});
        // Create gradients
        g_s = function_t("sparse_grd", in, {grdx, grdp});
        g = function_t("dense_grd", in,
                       {sym_t::densify(grdx), sym_t::densify(grdp)});

        in = {x, p};
        H_s = function_t("sparse_hes", in, {hesxx, hesxp, hespp});
        H = function_t("dense_hes", in,
                       {sym_t::densify(hesxx), sym_t::densify(hesxp),
                        sym_t::densify(hespp)});

        // If function is to be code-generated, do so.
        if (codegen) {
            f = bopt::casadi::codegen(f);
            g = bopt::casadi::codegen(g);
            g_s = bopt::casadi::codegen(g_s);
            H = bopt::casadi::codegen(H);
            H_s = bopt::casadi::codegen(H_s);
        }
    }

    // Sparsity patterns
    void setGradientSparsityPatterns(CostData &data) const override {
        set_eigen_sparsity(data.gx_s, g_s.sparsity_out(0));
        set_eigen_sparsity(data.gp_s, g_s.sparsity_out(1));
    }

    void setHessianSparsityPatterns(CostData &data) const override {
        set_eigen_sparsity(data.Hxx_s, H_s.sparsity_out(0));
        set_eigen_sparsity(data.Hxp_s, H_s.sparsity_out(1));
        set_eigen_sparsity(data.Hpp_s, H_s.sparsity_out(2));
    }

   protected:
    void evalImpl(const Eigen::Ref<const VectorXd> &x,
                  CostData &data) const override {
        VLOG(10) << "In casadi::CostTpl::evalImpl";
        f({x.data(), this->parameters().data()}, {&data.f});
    }

    /**
     * \copydoc CostTpl::evalGradients(const Eigen::Ref<const
     * VectorXd>, CostData &)
     *
     */
    void evalGradientsImpl(const Eigen::Ref<const VectorXd> &x, CostData &data,
                           bool compute_x, bool compute_p) const override {
        std::vector<Scalar *> out = {nullptr, nullptr};
        if (compute_x) out[0] = data.gx.data();
        if (compute_p) out[1] = data.gp.data();
        g({x.data(), this->parameters().data()}, out);
    }

    /**
     * \copydoc CostTpl::evalSparseGradients(const Eigen::Ref<const
     * VectorXd>, CostData &)
     *
     */
    void evalSparseGradientsImpl(const Eigen::Ref<const VectorXd> &x,
                                 CostData &data, bool compute_x,
                                 bool compute_p) const override {
        std::vector<Scalar *> out = {nullptr, nullptr};
        if (compute_x) out[0] = data.gx_s.valuePtr();
        if (compute_p) out[1] = data.gp_s.valuePtr();
        g_s({x.data(), this->parameters().data()}, out);
    }
    /**
     * \copydoc CostTpl::evalHessians(const Eigen::Ref<const VectorXd>,
     * const Eigen::Ref<const VectorXd>, CostData &)
     *
     */
    void evalHessiansImpl(const Eigen::Ref<const VectorXd> &x, CostData &data,
                          bool compute_xx, bool compute_xp,
                          bool compute_pp) const override {
        std::vector<Scalar *> out = {nullptr, nullptr, nullptr};
        if (compute_xx) out[0] = data.Hxx.data();
        if (compute_xp) out[1] = data.Hxp.data();
        if (compute_pp) out[2] = data.Hpp.data();
        H({x.data(), this->parameters().data()}, out);
    }

    /**
     * \copydoc CostTpl::evalSparseHessians(const Eigen::Ref<const
     * VectorXd>, const Eigen::Ref<const VectorXd>, CostData &)
     *
     */
    void evalSparseHessiansImpl(const Eigen::Ref<const VectorXd> &x,
                                CostData &data, bool compute_xx,
                                bool compute_xp,
                                bool compute_pp) const override {
        std::vector<Scalar *> out = {nullptr, nullptr, nullptr};
        if (compute_xx) out[0] = data.Hxx_s.valuePtr();
        if (compute_xp) out[1] = data.Hxp_s.valuePtr();
        if (compute_pp) out[2] = data.Hpp_s.valuePtr();
        H_s({x.data(), this->parameters().data()}, out);
    }

   private:
    function_t f;

    function_t g;
    function_t g_s;

    function_t H;
    function_t H_s;
};

typedef CostTpl<double> Cost;

/**
 * @brief Linear cost of the form fₚ(x) = aₚᵀx + bₚ
 *
 */
template <typename Scalar>
class LinearCostTpl : public bopt::LinearCostTpl<Scalar> {
   public:
    using LinearCostData = LinearCostDataTpl<Scalar>;

    LinearCostTpl(const sym_t &expression, const sym_vector_t &x,
                  const sym_vector_t &p, bool codegen = false)
        : bopt::LinearCostTpl<Scalar>(
              std::make_shared<CostTpl<Scalar>>(expression, x, p, codegen)) {
        sym_t a, b;
        // Check expression is linear
        sym_t::linear_coeff(expression, x, a, b, true);

        // Create function
        a_ = function_t("dense_a", {p}, {sym_t::densify(a)});
        a_s_ = function_t("sparse_a", {p}, {a});

        b_ = function_t("b", {p}, {b});

        // If function is to be code-generated, do so.
        if (codegen) {
            a_ = bopt::casadi::codegen(a_);
            a_s_ = bopt::casadi::codegen(a_s_);
            b_ = bopt::casadi::codegen(b_);
        }
    }

   protected:
    void evalCoefficientsImpl(LinearCostData &data) const override {
        a_({this->parameters().data()}, {data.a.data()});
        b_({this->parameters().data()}, {&data.b});
    }

    void evalSparseCoefficientsImpl(LinearCostData &data) const override {
        a_s_({this->parameters().data()}, {data.a_s.valuePtr()});
        b_({this->parameters().data()}, {&data.b});
    }

    void setCoefficientSparsityPatternsImpl(
        LinearCostData &data) const override {
        set_eigen_sparsity(data.a_s, a_s_.sparsity_out(0));
    }

   private:
    function_t a_;
    function_t a_s_;
    function_t b_;
};

typedef LinearCostTpl<double> LinearCost;

/**
 * @brief Quadratic cost of the form fₚ(x) = (1/2) xᵀ Aₚ x + bₚᵀ x + cₚ
 *
 */
template <typename Scalar>
class QuadraticCostTpl : public bopt::QuadraticCostTpl<Scalar> {
   public:
    using QuadraticCostData = QuadraticCostDataTpl<Scalar>;

    QuadraticCostTpl(const sym_t &expression, const sym_vector_t &x,
                     const sym_vector_t &p, bool codegen = false)
        : bopt::QuadraticCostTpl<Scalar>(
              std::make_shared<CostTpl<Scalar>>(expression, x, p, codegen)) {
        sym_t A, b, c;
        // Check expression is linear
        sym_t::quadratic_coeff(expression, x, A, b, c, true);
        // Make A lower triangular
        A = sym_t::tril(A);

        // Create function
        A_ = function_t("dense_A", {p}, {sym_t::densify(A)});
        A_s_ = function_t("sparse_A", {p}, {A});

        b_ = function_t("dense_b", {p}, {sym_t::densify(b)});
        b_s_ = function_t("sparse_b", {p}, {b});

        c_ = function_t("dense_c", {p}, {sym_t::densify(c)});

        // If function is to be code-generated, do so.
        if (codegen) {
            A_ = bopt::casadi::codegen(A_);
            A_s_ = bopt::casadi::codegen(A_s_);
            b_ = bopt::casadi::codegen(b_);
            b_s_ = bopt::casadi::codegen(b_s_);
            c_ = bopt::casadi::codegen(c_);
        }
    }

   protected:
    void evalCoefficientsImpl(QuadraticCostData &data) const override {
        A_({this->parameters().data()}, {data.A.data()});
        b_({this->parameters().data()}, {data.b.data()});
        c_({this->parameters().data()}, {&data.c});
    }

    void evalSparseCoefficientsImpl(QuadraticCostData &data) const override {
        A_s_({this->parameters().data()}, {data.A_s.valuePtr()});
        b_s_({this->parameters().data()}, {data.b_s.valuePtr()});
        c_({this->parameters().data()}, {&data.c});
    }

    void setCoefficientSparsityPatternsImpl(
        QuadraticCostData &data) const override {
        set_eigen_sparsity(data.A_s, A_s_.sparsity_out(0));
        set_eigen_sparsity(data.b_s, b_s_.sparsity_out(0));
    }

   private:
    function_t A_;
    function_t A_s_;
    function_t b_;
    function_t b_s_;
    function_t c_;
};

typedef QuadraticCostTpl<double> QuadraticCost;

}  // namespace casadi
}  // namespace bopt