#pragma once

#include "bopt/EvaluatorTraits.hpp"
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
template <typename ScalarType, int OutputSizeAtCompileTime = Eigen::Dynamic,
          SparsityType Sparsity = SparsityType::DENSE>
class FunctionGenerator {
    static constexpr bool IsOutputScalar = OutputSizeAtCompileTime == 1;
    using SX = ::casadi::SX;

    using Traits =
        EvaluatorTraits<ScalarType, OutputSizeAtCompileTime, Sparsity>;

    using InputVector = typename Traits::InputVectorType;

   public:
    FunctionGenerator(const SX &expression, const SX &x, const SX &p,
                      bool codegen = false) {
        std::vector<SX> in;
        in = {x, p};
        // Create function
        f = Function("f", in, {SX::densify(expression)});

        // Jacobian
        SX jacx = SX::jacobian(expression, x);
        SX jacp = SX::jacobian(expression, p);

        // Hessian
        SX l = SX::sym("l", expression.rows());
        SX hesxx =
            SX::tril(SX::jacobian(SX::gradient(SX::dot(l, expression), x), x));
        SX hesxp =
            SX::tril(SX::jacobian(SX::gradient(SX::dot(l, expression), x), p));
        SX hespp =
            SX::tril(SX::jacobian(SX::gradient(SX::dot(l, expression), p), p));

        if constexpr (Sparsity == SparsityType::DENSE) {
            jacx = SX::densify(jacx);
            jacp = SX::densify(jacp);

            hesxx = SX::densify(hesxx);
            hesxp = SX::densify(hesxp);
            hespp = SX::densify(hespp);
        }

        J = Function("jacobian", in, {jacx, jacp});
        in = {x, l, p};
        H = Function("hessian", in, {hesxx, hesxp, hespp});

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

    void compute(const Eigen::Ref<const InputVector> &x,
                 const Eigen::Ref<const InputVector> &p,
                 typename Traits::OutputType &y) const {
        std::vector<ScalarType *> out = {nullptr};
        out[0] = y.data();
        this->f({x.data(), p.data()}, out);
    }

    void computeJacobians(const Eigen::Ref<const InputVector> &x,
                          const Eigen::Ref<const InputVector> &p,
                          typename Traits::OutputJacobianType &Jx,
                          typename Traits::OutputJacobianType &Jp,
                          bool compute_x, bool compute_p) const {
        std::vector<ScalarType *> out = {nullptr, nullptr};
        if constexpr (Sparsity == SparsityType::SPARSE) {
            if (compute_x) out[0] = Jx.valuePtr();
            if (compute_p) out[1] = Jp.valuePtr();
        } else {
            if (compute_x) out[0] = Jx.data();
            if (compute_p) out[1] = Jp.data();
        }

        this->J({x.data(), p.data()}, out);
    }

    void computeHessians(const Eigen::Ref<const InputVector> &x,
                         const Eigen::Ref<const InputVector> &lambda,
                         const Eigen::Ref<const InputVector> &p,
                         typename Traits::OutputHessianType &Hxx,
                         typename Traits::OutputHessianType &Hxp,
                         typename Traits::OutputHessianType &Hpp,
                         bool compute_xx, bool compute_xp,
                         bool compute_pp) const {
        std::vector<ScalarType *> out = {nullptr, nullptr, nullptr};
        if constexpr (Sparsity == SparsityType::SPARSE) {
            if (compute_xx) out[0] = Hxx.valuePtr();
            if (compute_xp) out[1] = Hxp.valuePtr();
            if (compute_pp) out[2] = Hpp.valuePtr();
        } else {
            if (compute_xx) out[0] = Hxx.data();
            if (compute_xp) out[1] = Hxp.data();
            if (compute_pp) out[2] = Hpp.data();
        }

        this->H({x.data(), lambda.data(), p.data()}, out);
    }
};

template <typename ScalarType, SparsityType Sparsity>
class FunctionGenerator<ScalarType, 1, Sparsity> {
    using SX = ::casadi::SX;
    using Traits = EvaluatorTraits<ScalarType, 1, Sparsity>;
    using InputVector = typename Traits::InputVectorType;

   public:
    FunctionGenerator(const SX &expression, const SX &x, const SX &p,
                      bool codegen = false) {
        std::vector<SX> in;
        in = {x, p};
        // Create function
        f = Function("f", in, {SX::densify(expression)});

        // Jacobian
        SX grdx = SX::gradient(expression, x);
        SX grdp = SX::gradient(expression, p);

        // Hessian
        SX hesxx = SX::tril(SX::jacobian(SX::gradient(expression, x), x));
        SX hesxp = SX::tril(SX::jacobian(SX::gradient(expression, x), p));
        SX hespp = SX::tril(SX::jacobian(SX::gradient(expression, p), p));

        if constexpr (Sparsity == SparsityType::DENSE) {
            grdx = SX::densify(grdx);
            grdp = SX::densify(grdp);

            hesxx = SX::densify(hesxx);
            hesxp = SX::densify(hesxp);
            hespp = SX::densify(hespp);
        }

        g = Function("gradient", in, {grdx, grdp});
        H = Function("hessian", in, {hesxx, hesxp, hespp});

        // If function is to be code-generated, do so.
        if (codegen) {
            f = bopt::casadi::codegen(f);
            g = bopt::casadi::codegen(g);
            H = bopt::casadi::codegen(H);
        }
    }

   protected:
    /// @brief Casadi generated function
    Function f;
    /// @brief Casadi generated gradient function
    Function g;
    /// @brief Casadi generated Hessians function
    Function H;

    void compute(const Eigen::Ref<const InputVector> &x,
                 const Eigen::Ref<const InputVector> &p,
                 typename Traits::OutputType &y) const {
        std::vector<ScalarType *> out = {nullptr};
        out[0] = &y;
        this->f({x.data(), p.data()}, out);
    }

    void computeGradients(const Eigen::Ref<const InputVector> &x,
                          const Eigen::Ref<const InputVector> &p,
                          typename Traits::OutputGradientType &gx,
                          typename Traits::OutputGradientType &gp,
                          bool compute_x, bool compute_p) const {
        std::vector<ScalarType *> out = {nullptr, nullptr};
        if constexpr (Sparsity == SparsityType::SPARSE) {
            if (compute_x) out[0] = gx.valuePtr();
            if (compute_p) out[1] = gp.valuePtr();
        } else {
            if (compute_x) out[0] = gx.data();
            if (compute_p) out[1] = gp.data();
        }

        this->g({x.data(), p.data()}, out);
    }

    void computeHessians(const Eigen::Ref<const InputVector> &x,
                         const Eigen::Ref<const InputVector> &p,
                         typename Traits::OutputHessianType &Hxx,
                         typename Traits::OutputHessianType &Hxp,
                         typename Traits::OutputHessianType &Hpp,
                         bool compute_xx, bool compute_xp,
                         bool compute_pp) const {
        std::vector<ScalarType *> out = {nullptr, nullptr, nullptr};
        if constexpr (Sparsity == SparsityType::SPARSE) {
            if (compute_xx) out[0] = Hxx.valuePtr();
            if (compute_xp) out[1] = Hxp.valuePtr();
            if (compute_pp) out[2] = Hpp.valuePtr();
        } else {
            if (compute_xx) out[0] = Hxx.data();
            if (compute_xp) out[1] = Hxp.data();
            if (compute_pp) out[2] = Hpp.data();
        }

        this->H({x.data(), p.data()}, out);
    }
};

template <typename ScalarType, int OutputSizeAtCompileTime = Eigen::Dynamic,
          SparsityType Sparsity = SparsityType::DENSE>
class LinearFunctionGenerator {
    using SX = ::casadi::SX;
    using Traits =
        EvaluatorTraits<ScalarType, OutputSizeAtCompileTime, Sparsity>;
    using InputVector = typename Traits::InputVectorType;

   public:
    LinearFunctionGenerator(const SX &expression, const SX &x, const SX &p,
                            bool codegen = false) {
        // Variables and parameters
        std::vector<SX> in;
        in = {p};
        // Compute linear coefficients
        SX A, b;
        try {
            SX::linear_coeff(expression, x, A, b, true);
        } catch (std::exception &e) {
            throw std::runtime_error(
                "Expression provided is not linear in specified variable "
                "x!");
        }

        if constexpr (Sparsity == SparsityType::DENSE) {
            A = SX::densify(A);
        }

        b = SX::densify(b);

        coefficients = Function("coefficients", in, {A, b});

        if (codegen) {
            coefficients = bopt::casadi::codegen(coefficients);
        }
    }

   protected:
    void computeLinearCoefficients(const Eigen::Ref<const InputVector> &p,
                                   typename Traits::OutputJacobianType &A,
                                   typename Traits::OutputType &b) const {
        std::vector<ScalarType *> out = {nullptr, nullptr};
        if constexpr (Sparsity == SparsityType::SPARSE) {
            out[0] = A.valuePtr();
        } else {
            out[0] = A.data();
        }

        if constexpr (OutputSizeAtCompileTime == 1) {
            out[1] = &b;
        } else {
            out[1] = b.data();
        }

        this->coefficients({p.data()}, out);
    }

    Function coefficients;
};

template <typename ScalarType, SparsityType Sparsity>
class LinearFunctionGenerator<ScalarType, 1, Sparsity> {
    using SX = ::casadi::SX;
    using Traits = EvaluatorTraits<ScalarType, 1, Sparsity>;
    using InputVector = typename Traits::InputVectorType;

   public:
    LinearFunctionGenerator(const SX &expression, const SX &x, const SX &p,
                            bool codegen = false) {
        // Variables and parameters
        std::vector<SX> in;
        in = {p};
        // Compute linear coefficients
        SX a, b;
        try {
            SX::linear_coeff(expression, x, a, b, true);
        } catch (std::exception &e) {
            throw std::runtime_error(
                "Expression provided is not linear in specified variable "
                "x!");
        }

        if constexpr (Sparsity == SparsityType::DENSE) {
            a = SX::densify(a);
        }

        b = SX::densify(b);

        coefficients = Function("coefficients", in, {a, b});

        if (codegen) {
            coefficients = bopt::casadi::codegen(coefficients);
        }
    }

   protected:
    void computeLinearCoefficients(const Eigen::Ref<const InputVector> &p,
                                   typename Traits::OutputGradientType &a,
                                   typename Traits::OutputType &b) const {
        std::vector<ScalarType *> out = {nullptr, nullptr};
        if constexpr (Sparsity == SparsityType::SPARSE) {
            out[0] = a.valuePtr();
        } else {
            out[0] = a.data();
        }

        out[1] = &b;

        this->coefficients({p.data()}, out);
    }

    Function coefficients;
};

template <typename ScalarType, SparsityType Sparsity>
class QuadraticFunctionGenerator {
   public:
    using SX = ::casadi::SX;
    using Traits = EvaluatorTraits<ScalarType, 1, Sparsity>;
    using InputVector = typename Traits::InputVectorType;

    QuadraticFunctionGenerator(const SX &expression, const SX &x, const SX &p,
                               bool codegen = false) {
        std::vector<SX> in;
        in = {p};
        // Compute linear coefficients
        SX A, b, c;
        try {
            SX::quadratic_coeff(expression, x, A, b, c, true);
        } catch (std::exception &e) {
            throw std::runtime_error(
                "Expression provided is not quadratic in specified variable "
                "x!");
        }

        A = SX::tril(A);

        if constexpr (Sparsity == SparsityType::DENSE) {
            A = SX::densify(A);
            b = SX::densify(b);
        }

        c = SX::densify(c);

        coefficients = Function("coefficients", in, {A, b, c});

        if (codegen) {
            coefficients = bopt::casadi::codegen(coefficients);
        }
    }

   protected:
    void computeQuadraticCoefficients(const Eigen::Ref<const InputVector> &p,
                                      typename Traits::OutputHessianType &A,
                                      typename Traits::OutputGradientType &b,
                                      typename Traits::OutputType &c) const {
        std::vector<ScalarType *> out = {nullptr, nullptr, nullptr};
        if constexpr (Sparsity == SparsityType::SPARSE) {
            out[0] = A.valuePtr();
            out[1] = b.valuePtr();
        } else {
            out[0] = A.data();
            out[1] = b.data();
        }

        out[2] = &c;

        this->coefficients({p.data()}, out);
    }

    Function coefficients;
};

}  // namespace internal
}  // namespace casadi
}  // namespace bopt