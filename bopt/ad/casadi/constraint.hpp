#pragma once

#include "bopt/ad/casadi/evaluator.hpp"

namespace bopt {
namespace casadi {

/**
 * @brief Constraint of the form y = fₚ(x) ∈ ℝᵐ
 *
 */
template <typename Scalar>
class ConstraintTpl : public bopt::ConstraintTpl<Scalar> {
    using ConstraintData = ConstraintDataTpl<Scalar>;

   public:
    ConstraintTpl(const sym_t &expression, const sym_vector_t &x,
                  const sym_vector_t &p, const sym_vector_t &lb,
                  const sym_vector_t &ub, bool codegen = false)
        : bopt::ConstraintTpl<Scalar>(std::make_shared<EvaluatorTpl<Scalar>>(
              expression, x, p, codegen)) {
        // Set up variables
        this->setOutputDimension(expression.rows());
        this->setTangentSpaceDimension(x.rows());
        this->setParameterDimension(p.rows());

        std::vector<sym_vector_t> in;

        in = {p};

        // Jacobian
        sym_t ljacp = sym_t::jacobian(lb, p);
        sym_t ujacp = sym_t::jacobian(ub, p);

        // Hessian
        sym_vector_t l = sym_t::sym("l", expression.rows());
        sym_t lhespp = sym_t::tril(
            sym_t::jacobian(sym_t::gradient(sym_t::dot(l, lb), p), p));
        sym_t uhespp = sym_t::tril(
            sym_t::jacobian(sym_t::gradient(sym_t::dot(l, ub), p), p));

        // Create function
        bounds_f_ =
            function_t("bound_f", in, {sym_t::densify(lb), sym_t::densify(ub)});
        // Create jacobians
        bounds_J_s_ = function_t("bound_sparse_jac", in, {ljacp, ujacp});
        bounds_J_ = function_t("bound_dense_jac", in,
                               {sym_t::densify(ljacp), sym_t::densify(ujacp)});

        in = {x, l, p};
        bounds_H_s_ = function_t("bound_sparse_hes", in, {lhespp, uhespp});
        bounds_H_ =
            function_t("bound_dense_hes", in,
                       {sym_t::densify(lhespp), sym_t::densify(uhespp)});

        // If function is to be code-generated, do so.
        if (codegen) {
            bounds_f_ = bopt::casadi::codegen(bounds_f_);
            bounds_J_ = bopt::casadi::codegen(bounds_J_);
            bounds_J_s_ = bopt::casadi::codegen(bounds_J_s_);
            bounds_H_ = bopt::casadi::codegen(bounds_H_);
            bounds_H_s_ = bopt::casadi::codegen(bounds_H_s_);
        }
    }

   protected:
    void evalBoundsImpl(ConstraintDataTpl<Scalar> &data) const override {
        std::vector<Scalar *> out(2);
        out[0] = data.lb.data();
        out[1] = data.ub.data();
        bounds_f_({this->parameters().data()}, out);
    }

    void evalBoundJacobiansImpl(
        ConstraintDataTpl<Scalar> &data) const override {
        std::vector<Scalar *> out(2);
        out[0] = data.Jlb_p.data();
        out[1] = data.Jub_p.data();
        bounds_J_({this->parameters().data()}, out);
    }
    void evalBoundSparseJacobiansImpl(
        ConstraintDataTpl<Scalar> &data) const override {
        std::vector<Scalar *> out(2);
        out[0] = data.Jlb_p_s.valuePtr();
        out[1] = data.Jub_p_s.valuePtr();
        bounds_J_s_({this->parameters().data()}, out);
    }

    void evalBoundHessiansImpl(const Eigen::Ref<const VectorX<Scalar>> &lambda,
                               ConstraintDataTpl<Scalar> &data) const override {
        std::vector<Scalar *> out(2);
        out[0] = data.Hlb_pp.data();
        out[1] = data.Hub_pp.data();
        bounds_H_({lambda.data(), this->parameters().data()}, out);
    }
    void evalBoundSparseHessiansImpl(
        const Eigen::Ref<const VectorX<Scalar>> &lambda,
        ConstraintDataTpl<Scalar> &data) const override {
        std::vector<Scalar *> out(2);
        out[0] = data.Hlb_pp_s.valuePtr();
        out[1] = data.Hub_pp_s.valuePtr();
        bounds_H_s_({lambda.data(), this->parameters().data()}, out);
    }

   private:
    function_t bounds_f_;

    function_t bounds_J_;
    function_t bounds_J_s_;

    function_t bounds_H_;
    function_t bounds_H_s_;
};

typedef ConstraintTpl<double> Constraint;

/**
 * @brief Constraint of the form lb ≤ Ax ≤ ub
 *
 */
template <typename Scalar>
class LinearConstraintTpl : public bopt::LinearConstraintTpl<Scalar> {
    using LinearConstraintData = LinearConstraintDataTpl<Scalar>;

   public:
    LinearConstraintTpl(const sym_t &expression, const sym_vector_t &x,
                        const sym_vector_t &p, const sym_vector_t &lb,
                        const sym_vector_t &ub, bool codegen = false)
        : bopt::LinearConstraintTpl<Scalar>(
              std::make_shared<ConstraintTpl<Scalar>>(expression, x, p, lb, ub,
                                                      codegen)) {
        sym_t A, b;
        // Check expression is linear
        sym_t::linear_coeff(expression, x, A, b, true);
        assert(b.is_zero());

        // Create function
        A_ = function_t("dense_A", {p}, {sym_t::densify(A)});
        A_s_ = function_t("sparse_A", {p}, {A});

        // If function is to be code-generated, do so.
        if (codegen) {
            A_ = bopt::casadi::codegen(A_);
            A_s_ = bopt::casadi::codegen(A_s_);
        }
    }

   protected:
    virtual void evalCoefficientsImpl(LinearConstraintData &data) const {
        A_({this->parameters().data()}, {data.A.data()});
    }

    virtual void evalSparseCoefficientsImpl(LinearConstraintData &data) const {
        A_s_({this->parameters().data()}, {data.A_s.valuePtr()});
    }

    virtual void setCoefficientSparsityPatternsImpl(
        LinearConstraintData &data) const {
        set_eigen_sparsity(data.A_s, A_s_.sparsity_out(0));
    }

   private:
    function_t A_;
    function_t A_s_;
};

typedef LinearConstraintTpl<double> LinearConstraint;

/**
 * @brief Constraint of the form lower_bound() <= x <= upper_bound()
 *
 */
template <typename Scalar>
class BoundingBoxConstraintTpl : public LinearConstraintTpl<Scalar> {
   public:
    BoundingBoxConstraintTpl(const Index &dim_input,
                             const Eigen::Ref<const VectorXd> &lower_bound,
                             const Eigen::Ref<const VectorXd> &upper_bound)
        : LinearConstraintTpl<Scalar>(dim_input, dim_input) {}

    BoundingBoxConstraintTpl(const Index &dim_input, const double &lower_bound,
                             const double &upper_bound)
        : LinearConstraintTpl<Scalar>(dim_input, dim_input) {}

    static std::shared_ptr<BoundingBoxConstraintTpl> create(
        const Index &dim_input, const Eigen::Ref<const VectorXd> &lower_bound,
        const Eigen::Ref<const VectorXd> &upper_bound) {
        return std::make_shared<BoundingBoxConstraintTpl>(
            dim_input, lower_bound, upper_bound);
    }

   protected:
    void evalImpl(const Eigen::Ref<const VectorXd> &x,
                  EvaluatorDataTpl<Scalar> &data) const override {
        data.y = x;
    }

   private:
};

typedef BoundingBoxConstraintTpl<double> BoundingBoxConstraint;

}  // namespace casadi
}  // namespace bopt
