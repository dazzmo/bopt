#pragma once

#include "bopt/Evaluator.hpp"
#include "bopt/ad/casadi/Utils.hpp"

namespace bopt {
namespace casadi {
namespace internal {

/**
 * @brief Casadi function generator
 *
 * @tparam EvaluatorTraits
 * @tparam OutputSize
 */
template <typename EvaluatorTraits, int OutputSize>
class FunctionGenerator {
   public:
    FunctionGenerator(const SymbolicVector &expression, const SymbolicVector &x,
                      const SymbolicVector &p, bool codegen = false) {
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

        if constexpr (EvaluatorTraits::type == FunctionType::DENSE) {
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
    /// @brief Casadi generated function
    Function f;
    /// @brief Casadi generated Jacobians function
    Function J;
    /// @brief Casadi generated Hessians function
    Function H;
};

/**
 * @brief Casadi function evaluator for scalar functions
 *
 * @tparam EvaluatorTraits
 * @tparam OutputSize
 */
template <typename EvaluatorTraits>
class FunctionGenerator<EvaluatorTraits, 1> {
   public:
    FunctionGenerator(const SymbolicVector &expression, const SymbolicVector &x,
                      const SymbolicVector &p, bool codegen = false) {
        // Set up variables
        std::vector<SymbolicVector> in;
        in = {x, p};

        // Jacobian
        SymbolicMatrix grdx = Symbol::gradient(expression, x);
        SymbolicMatrix grdp = Symbol::gradient(expression, p);

        // Hessian
        SymbolicMatrix hesxx =
            Symbol::tril(Symbol::jacobian(Symbol::gradient(expression, x), x));
        SymbolicMatrix hesxp =
            Symbol::tril(Symbol::jacobian(Symbol::gradient(expression, x), p));
        SymbolicMatrix hespp =
            Symbol::tril(Symbol::jacobian(Symbol::gradient(expression, p), p));

        // Create function
        f = Function("f", in, {expression});

        if constexpr (EvaluatorTraits::type == FunctionType::DENSE) {
            // Create gradients
            g = Function("gradient", in,
                         {Symbol::densify(grdx), Symbol::densify(grdp)});
            H = Function("hessian", in,
                         {Symbol::densify(hesxx), Symbol::densify(hesxp),
                          Symbol::densify(hespp)});
        } else {
            // Create gradients
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
    Function f;
    Function g;
    Function H;
};

}  // namespace internal

template <typename EvaluatorTraits, int OutputSize = Eigen::Dynamic>
class EvaluatorTpl
    : public bopt::EvaluatorTpl<EvaluatorTraits>,
      public internal::FunctionGenerator<EvaluatorTraits, OutputSize> {
   public:
    using Base = bopt::EvaluatorTpl<EvaluatorTraits>;

    using Scalar = typename Base::Scalar;

    using InputVector = typename Base::InputVector;
    using InputVectorConstRef = typename Base::InputVectorConstRef;

    using Data = typename Base::Data;

    EvaluatorTpl(const SymbolicVector &expression, const SymbolicVector &x,
                 const SymbolicVector &p, bool codegen = false)
        : Base(x.rows(), expression.rows(), "CasADi generated evaluator"),
          internal::FunctionGenerator<EvaluatorTraits, OutputSize>(
              expression, x, p, codegen) {
        this->setNumParameters(p.rows());
    }

   protected:
    virtual void evalImpl(const InputVectorConstRef &x,
                          Data &data) const override {
        this->f({x.data(), this->getParameters().data()}, {data.y.data()});
    }

    void evalJacobiansImpl(const InputVectorConstRef &x, Data &data,
                           bool compute_x, bool compute_p) const override {
        std::vector<Scalar *> out = {nullptr, nullptr};
        if constexpr (EvaluatorTraits::type == FunctionType::SPARSE) {
            if (compute_x) out[0] = data.Jx.valuePtr();
            if (compute_p) out[1] = data.Jp.valuePtr();
        } else {
            if (compute_x) out[0] = data.Jx.data();
            if (compute_p) out[1] = data.Jp.data();
        }

        this->J({x.data(), this->getParameters().data()}, out);
    }

    void evalHessiansImpl(const InputVectorConstRef &x,
                          const InputVectorConstRef &lambda, Data &data,
                          bool compute_xx, bool compute_xp,
                          bool compute_pp) const override {
        std::vector<Scalar *> out = {nullptr, nullptr, nullptr};
        if constexpr (EvaluatorTraits::type == FunctionType::SPARSE) {
            if (compute_xx) out[0] = data.Hxx.valuePtr();
            if (compute_xp) out[1] = data.Hxp.valuePtr();
            if (compute_pp) out[2] = data.Hpp.valuePtr();
        } else {
            if (compute_xx) out[0] = data.Hxx.data();
            if (compute_xp) out[1] = data.Hxp.data();
            if (compute_pp) out[2] = data.Hpp.data();
        }

        this->H({x.data(), lambda.data(), this->getParameters().data()}, out);
    }

    void setDataSparsityImpl(Data &data) const override {
        if constexpr (EvaluatorTraits::type == FunctionType::SPARSE) {
            setupSparseEigenMatrix(data.Jx, this->J.sparsity_out(0));
            setupSparseEigenMatrix(data.Jp, this->J.sparsity_out(1));
            setupSparseEigenMatrix(data.Hxx, this->H.sparsity_out(0));
            setupSparseEigenMatrix(data.Hxp, this->H.sparsity_out(1));
            setupSparseEigenMatrix(data.Hpp, this->H.sparsity_out(2));
        }
    }
};

template <typename EvaluatorTraits>
class EvaluatorTpl<EvaluatorTraits, 1>
    : public bopt::EvaluatorTpl<EvaluatorTraits, 1>,
      public internal::FunctionGenerator<EvaluatorTraits, 1> {
   public:
    using Base = bopt::EvaluatorTpl<EvaluatorTraits, 1>;

    using Scalar = typename Base::Scalar;

    using InputVector = typename Base::InputVector;
    using InputVectorConstRef = typename Base::InputVectorConstRef;

    using Data = typename Base::Data;

    EvaluatorTpl(const SymbolicVector &expression, const SymbolicVector &x,
                 const SymbolicVector &p, bool codegen = false)
        : Base(x.rows(), expression.rows(), "CasADi generated evaluator"),
          internal::FunctionGenerator<EvaluatorTraits, 1>(expression, x, p,
                                                          codegen) {
        this->setNumParameters(p.rows());
    }

   protected:
    virtual void evalImpl(const InputVectorConstRef &x,
                          Data &data) const override {
        this->f({x.data(), this->getParameters().data()}, {&data.y});
    }

    void evalGradientsImpl(const InputVectorConstRef &x, Data &data,
                           bool compute_x, bool compute_p) const override {
        std::vector<Scalar *> out = {nullptr, nullptr};
        if constexpr (EvaluatorTraits::type == FunctionType::SPARSE) {
            if (compute_x) out[0] = data.gx.valuePtr();
            if (compute_p) out[1] = data.gp.valuePtr();
        } else {
            if (compute_x) out[0] = data.gx.data();
            if (compute_p) out[1] = data.gp.data();
        }

        this->g({x.data(), this->getParameters().data()}, out);
    }

    void evalHessiansImpl(const InputVectorConstRef &x, Data &data,
                          bool compute_xx, bool compute_xp,
                          bool compute_pp) const override {
        std::vector<Scalar *> out = {nullptr, nullptr, nullptr};
        if constexpr (EvaluatorTraits::type == FunctionType::SPARSE) {
            if (compute_xx) out[0] = data.Hxx.valuePtr();
            if (compute_xp) out[1] = data.Hxp.valuePtr();
            if (compute_pp) out[2] = data.Hpp.valuePtr();
        } else {
            if (compute_xx) out[0] = data.Hxx.data();
            if (compute_xp) out[1] = data.Hxp.data();
            if (compute_pp) out[2] = data.Hpp.data();
        }

        this->H({x.data(), this->getParameters().data()}, out);
    }

    void setDataSparsityImpl(Data &data) const override {
        if constexpr (EvaluatorTraits::type == FunctionType::SPARSE) {
            setupSparseEigenMatrix(data.gx, this->g.sparsity_out(0));
            setupSparseEigenMatrix(data.gp, this->g.sparsity_out(1));
            setupSparseEigenMatrix(data.Hxx, this->H.sparsity_out(0));
            setupSparseEigenMatrix(data.Hxp, this->H.sparsity_out(1));
            setupSparseEigenMatrix(data.Hpp, this->H.sparsity_out(2));
        }
    }
};

template <typename Scalar, int OutputSize = Eigen::Dynamic>
using DenseEvaluatorTpl =
    EvaluatorTpl<DenseEvaluatorTraits<Scalar>, OutputSize>;
template <int OutputSize = Eigen::Dynamic>
using DenseEvaluator = DenseEvaluatorTpl<Real, OutputSize>;

template <typename Scalar, int OutputSize = Eigen::Dynamic>
using SparseEvaluatorTpl =
    EvaluatorTpl<SparseEvaluatorTraits<Scalar>, OutputSize>;
template <int OutputSize = Eigen::Dynamic>
using SparseEvaluator = SparseEvaluatorTpl<Real, OutputSize>;

/* Linear Evaluators */

template <typename EvaluatorTraits, int OutputSize = Eigen::Dynamic>
class LinearEvaluatorTpl
    : public bopt::LinearEvaluatorTpl<EvaluatorTraits, OutputSize>,
      public internal::FunctionGenerator<EvaluatorTraits, OutputSize> {
   public:
    using Base = bopt::LinearEvaluatorTpl<EvaluatorTraits, OutputSize>;

    using Scalar = typename Base::Scalar;

    using InputVector = typename Base::InputVector;
    using InputVectorConstRef = typename Base::InputVectorConstRef;

    using Data = typename Base::Data;
    using EvaluatorData = typename Base::EvaluatorData;

    LinearEvaluatorTpl(const SymbolicVector &expression,
                       const SymbolicVector &x, const SymbolicVector &p,
                       bool codegen = false)
        : Base(x.rows(), expression.rows(), "CasADi generated evaluator"),
          internal::FunctionGenerator<EvaluatorTraits, OutputSize>(
              expression, x, p, codegen) {
        // Variables and parameters
        std::vector<SymbolicVector> in;
        in = {p};
        // Compute linear coefficients
        Symbol A, b;
        try {
            Symbol::linear_coeff(expression, x, A, b, true);
        } catch (std::exception &e) {
            throw std::runtime_error(
                "Expression provided is not linear in specified variable "
                "x!");
        }

        if constexpr (EvaluatorTraits::type == FunctionType::DENSE) {
            coefficients = Function("linear_coefficients", in,
                                    {Symbol::densify(A), Symbol::densify(b)});
        } else {
            coefficients =
                Function("linear_coefficients", in, {A, Symbol::densify(b)});
        }

        // If function is to be code-generated, do so.
        if (codegen) {
            coefficients = bopt::casadi::codegen(coefficients);
        }
    }

   protected:
    void evalImpl(const InputVectorConstRef &x,
                  EvaluatorData &data) const override {
        this->f({x.data(), this->getParameters().data()}, {data.y.data()});
    }

    void evalJacobiansImpl(const InputVectorConstRef &x, EvaluatorData &data,
                           bool compute_x, bool compute_p) const override {
        std::vector<Scalar *> out = {nullptr, nullptr};
        if constexpr (EvaluatorTraits::type == FunctionType::SPARSE) {
            if (compute_x) out[0] = data.Jx.valuePtr();
            if (compute_p) out[1] = data.Jp.valuePtr();
        } else {
            if (compute_x) out[0] = data.Jx.data();
            if (compute_p) out[1] = data.Jp.data();
        }

        this->J({x.data(), this->getParameters().data()}, out);
    }

    void evalHessiansImpl(const InputVectorConstRef &x,
                          const InputVectorConstRef &lambda,
                          EvaluatorData &data, bool compute_xx, bool compute_xp,
                          bool compute_pp) const override {
        std::vector<Scalar *> out = {nullptr, nullptr, nullptr};
        if constexpr (EvaluatorTraits::type == FunctionType::SPARSE) {
            if (compute_pp) out[2] = data.Hpp.valuePtr();
        } else {
            if (compute_pp) out[2] = data.Hpp.data();
        }

        this->H({x.data(), lambda.data(), this->getParameters().data()}, out);
    }

    void evalCoefficientsImpl(Data &data) const {
        std::vector<Scalar *> out = {nullptr, nullptr};
        if constexpr (EvaluatorTraits::type == FunctionType::SPARSE) {
            out[0] = data.A.valuePtr();
        } else {
            out[0] = data.A.data();
        }
        out[1] = data.b.data();

        this->coefficients({this->getParameters().data()}, out);
    }

    void setDataSparsityImpl(Data &data) const override {
        if constexpr (EvaluatorTraits::type == FunctionType::SPARSE) {
            setupSparseEigenMatrix(data.Jx, this->J.sparsity_out(0));
            setupSparseEigenMatrix(data.Jp, this->J.sparsity_out(1));
            setupSparseEigenMatrix(data.Hxx, this->H.sparsity_out(0));
            setupSparseEigenMatrix(data.Hxp, this->H.sparsity_out(1));
            setupSparseEigenMatrix(data.Hpp, this->H.sparsity_out(2));
            setupSparseEigenMatrix(data.A, this->coefficients.sparsity_out(0));
        }
    }

   private:
    Function coefficients;
};

template <typename EvaluatorTraits>
class LinearEvaluatorTpl<EvaluatorTraits, 1>
    : public bopt::LinearEvaluatorTpl<EvaluatorTraits, 1>,
      public internal::FunctionGenerator<EvaluatorTraits, 1> {
   public:
    using Base = bopt::LinearEvaluatorTpl<EvaluatorTraits, 1>;

    using Scalar = typename Base::Scalar;

    using InputVector = typename Base::InputVector;
    using InputVectorConstRef = typename Base::InputVectorConstRef;

    using Data = typename Base::Data;
    using EvaluatorData = typename Base::EvaluatorData;

    LinearEvaluatorTpl(const SymbolicVector &expression,
                       const SymbolicVector &x, const SymbolicVector &p,
                       bool codegen = false)
        : Base(x.rows(), "CasADi generated evaluator"),
          internal::FunctionGenerator<EvaluatorTraits, 1>(expression, x, p,
                                                          codegen) {
        // Variables and parameters
        std::vector<SymbolicVector> in;
        in = {p};
        // Compute linear coefficients
        Symbol a, b;
        try {
            Symbol::linear_coeff(expression, x, a, b, true);
        } catch (std::exception &e) {
            throw std::runtime_error(
                "Expression provided is not linear in specified variable "
                "x!");
        }

        if constexpr (EvaluatorTraits::type == FunctionType::DENSE) {
            coefficients = Function("linear_coefficients", in,
                                    {Symbol::densify(a), Symbol::densify(b)});
        } else {
            coefficients =
                Function("linear_coefficients", in, {a, Symbol::densify(b)});
        }

        // If function is to be code-generated, do so.
        if (codegen) {
            coefficients = bopt::casadi::codegen(coefficients);
        }
    }

   protected:
    void evalImpl(const InputVectorConstRef &x,
                  EvaluatorData &data) const override {
        this->f({x.data(), this->getParameters().data()}, {&data.y});
    }

    void evalGradientsImpl(const InputVectorConstRef &x, EvaluatorData &data,
                           bool compute_x, bool compute_p) const override {
        std::vector<Scalar *> out = {nullptr, nullptr};
        if constexpr (EvaluatorTraits::type == FunctionType::SPARSE) {
            if (compute_x) out[0] = data.gx.valuePtr();
            if (compute_p) out[1] = data.gp.valuePtr();
        } else {
            if (compute_x) out[0] = data.gx.data();
            if (compute_p) out[1] = data.gp.data();
        }

        this->g({x.data(), this->getParameters().data()}, out);
    }

    void evalHessiansImpl(const InputVectorConstRef &x, EvaluatorData &data,
                          bool compute_xx, bool compute_xp,
                          bool compute_pp) const override {
        std::vector<Scalar *> out = {nullptr, nullptr, nullptr};
        if constexpr (EvaluatorTraits::type == FunctionType::SPARSE) {
            if (compute_pp) out[2] = data.Hpp.valuePtr();
        } else {
            if (compute_pp) out[2] = data.Hpp.data();
        }

        this->H({x.data(), this->getParameters().data()}, out);
    }

    void evalCoefficientsImpl(Data &data) const {
        std::vector<Scalar *> out = {nullptr, nullptr};
        if constexpr (EvaluatorTraits::type == FunctionType::SPARSE) {
            out[0] = data.a.valuePtr();
        } else {
            out[0] = data.a.data();
        }
        out[1] = &data.b;

        this->coefficients({this->getParameters().data()}, out);
    }

    void setDataSparsityImpl(Data &data) const override {
        if constexpr (EvaluatorTraits::type == FunctionType::SPARSE) {
            setupSparseEigenMatrix(data.gx, this->g.sparsity_out(0));
            setupSparseEigenMatrix(data.gp, this->g.sparsity_out(1));
            setupSparseEigenMatrix(data.Hxx, this->H.sparsity_out(0));
            setupSparseEigenMatrix(data.Hxp, this->H.sparsity_out(1));
            setupSparseEigenMatrix(data.Hpp, this->H.sparsity_out(2));
            setupSparseEigenMatrix(data.a, this->coefficients.sparsity_out(0));
        }
    }

   private:
    Function coefficients;
};

template <typename Scalar, int OutputSize = Eigen::Dynamic>
using DenseLinearEvaluatorTpl =
    LinearEvaluatorTpl<DenseEvaluatorTraits<Scalar>, OutputSize>;
template <int OutputSize = Eigen::Dynamic>
using DenseLinearEvaluator = DenseLinearEvaluatorTpl<Real, OutputSize>;

template <typename Scalar, int OutputSize = Eigen::Dynamic>
using SparseLinearEvaluatorTpl =
    LinearEvaluatorTpl<SparseEvaluatorTraits<Scalar>, OutputSize>;
template <int OutputSize = Eigen::Dynamic>
using SparseLinearEvaluator = SparseLinearEvaluatorTpl<Real, OutputSize>;

template <typename EvaluatorTraits>
class QuadraticEvaluatorTpl
    : public bopt::QuadraticEvaluatorTpl<EvaluatorTraits>,
      public internal::FunctionGenerator<EvaluatorTraits, 1> {
   public:
    using Base = bopt::QuadraticEvaluatorTpl<EvaluatorTraits>;

    using Scalar = typename Base::Scalar;

    using InputVector = typename Base::InputVector;
    using InputVectorConstRef = typename Base::InputVectorConstRef;

    using Data = typename Base::Data;
    using EvaluatorData = typename Base::EvaluatorData;

    QuadraticEvaluatorTpl(const SymbolicVector &expression,
                          const SymbolicVector &x, const SymbolicVector &p,
                          bool codegen = false)
        : Base(x.rows(), "CasADi generated evaluator"),
          internal::FunctionGenerator<EvaluatorTraits, 1>(expression, x, p,
                                                          codegen) {
        // Variables and parameters
        std::vector<SymbolicVector> in;
        in = {p};
        // Compute quadratic coefficients
        Symbol A, b, c;
        try {
            Symbol::quadratic_coeff(expression, x, A, b, c, true);
        } catch (std::exception &e) {
            throw std::runtime_error(
                "Expression provided is not quadratic in specified variable "
                "x!");
        }

        if constexpr (EvaluatorTraits::type == FunctionType::DENSE) {
            coefficients = Function(
                "quadratic_coefficients", in,
                {Symbol::densify(A), Symbol::densify(b), Symbol::densify(c)});
        } else {
            coefficients = Function("quadratic_coefficients", in,
                                    {A, b, Symbol::densify(c)});
        }

        // If function is to be code-generated, do so.
        if (codegen) {
            coefficients = bopt::casadi::codegen(coefficients);
        }
    }

   protected:
    void evalImpl(const InputVectorConstRef &x,
                  EvaluatorData &data) const override {
        this->f({x.data(), this->getParameters().data()}, {&data.y});
    }

    void evalGradientsImpl(const InputVectorConstRef &x, EvaluatorData &data,
                           bool compute_x, bool compute_p) const override {
        std::vector<Scalar *> out = {nullptr, nullptr};
        if constexpr (EvaluatorTraits::type == FunctionType::SPARSE) {
            if (compute_x) out[0] = data.gx.valuePtr();
            if (compute_p) out[1] = data.gp.valuePtr();
        } else {
            if (compute_x) out[0] = data.gx.data();
            if (compute_p) out[1] = data.gp.data();
        }

        this->g({x.data(), this->getParameters().data()}, out);
    }

    void evalHessiansImpl(const InputVectorConstRef &x, EvaluatorData &data,
                          bool compute_xx, bool compute_xp,
                          bool compute_pp) const override {
        std::vector<Scalar *> out = {nullptr, nullptr, nullptr};
        if constexpr (EvaluatorTraits::type == FunctionType::SPARSE) {
            if (compute_pp) out[2] = data.Hpp.valuePtr();
        } else {
            if (compute_pp) out[2] = data.Hpp.data();
        }

        this->H({x.data(), this->getParameters().data()}, out);
    }

    void evalCoefficientsImpl(Data &data) const {
        std::vector<Scalar *> out = {nullptr, nullptr, nullptr};
        if constexpr (EvaluatorTraits::type == FunctionType::SPARSE) {
            out[0] = data.A.valuePtr();
            out[1] = data.b.valuePtr();
        } else {
            out[0] = data.A.data();
            out[1] = data.b.data();
        }
        out[2] = &data.c;

        this->coefficients({this->getParameters().data()}, out);
    }

    void setDataSparsityImpl(Data &data) const override {
        if constexpr (EvaluatorTraits::type == FunctionType::SPARSE) {
            setupSparseEigenMatrix(data.gx, this->g.sparsity_out(0));
            setupSparseEigenMatrix(data.gp, this->g.sparsity_out(1));
            setupSparseEigenMatrix(data.Hxx, this->H.sparsity_out(0));
            setupSparseEigenMatrix(data.Hxp, this->H.sparsity_out(1));
            setupSparseEigenMatrix(data.Hpp, this->H.sparsity_out(2));
            setupSparseEigenMatrix(data.A, this->coefficients.sparsity_out(0));
            setupSparseEigenMatrix(data.b, this->coefficients.sparsity_out(1));
        }
    }

   private:
    Function coefficients;
};

template <typename Scalar>
using DenseQuadraticEvaluatorTpl =
    QuadraticEvaluatorTpl<DenseEvaluatorTraits<Scalar>>;
using DenseQuadraticEvaluator = DenseQuadraticEvaluatorTpl<Real>;

template <typename Scalar>
using SparseQuadraticEvaluatorTpl =
    QuadraticEvaluatorTpl<SparseEvaluatorTraits<Scalar>>;
using SparseQuadraticEvaluator = SparseQuadraticEvaluatorTpl<Real>;

}  // namespace casadi
}  // namespace bopt