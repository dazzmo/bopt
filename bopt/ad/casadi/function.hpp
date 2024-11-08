#ifndef bopt_CASADI_FUNCTION_HPP
#define bopt_CASADI_FUNCTION_HPP

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "bopt/ad/casadi/eigen.hpp"
#include "bopt/constraints.hpp"
#include "bopt/costs.hpp"
#include "bopt/function.hpp"
#include "bopt/logging.hpp"
#include "bopt/types.hpp"

namespace bopt {
namespace casadi {

// using sym_t = ::casadi::SX;

// template <std::size_t _InputSize, std::size_t _OutputSize>
// class FunctionWrapper {
//    public:
//     using UniquePtr = std::unique_ptr<FunctionWrapper>;
//     using SharedPtr = std::shared_ptr<FunctionWrapper>;

//     FunctionWrapper() = default;
//     FunctionWrapper(const ::casadi::Function &f, const bool &codegen = false) {
//         *this = f;
//     }

//     ~FunctionWrapper() {
//         // Release memory for casadi function
//         if (!f_.is_null()) f_.release(mem_);
//     }

//     FunctionWrapper<_InputSize, _OutputSize> &operator=(
//         const FunctionWrapper &other) {
//         *this = other.f_;
//         return *this;
//     }

//     FunctionWrapper<_InputSize, _OutputSize> &operator=(::casadi::Function f) {
//         if (f.is_null()) {
//             return *this;
//         }

//         f_ = f;

//         // Checkout memory object for function
//         mem_ = f_.checkout();

//         // Resize work vectors
//         in_data_ptr_.assign(f_.n_in(), nullptr);
//         out_data_ptr_.assign(f_.n_out(), nullptr);

//         iw_.assign(f_.sz_iw(), 0);
//         dw_.assign(f_.sz_w(), 0.0);

//         return *this;
//     }

//     void call(
//         const std::array<Eigen::Ref<const Eigen::VectorXd>, _InputSize> &in,
//         const std::array<OptionalEigenMatrix<Eigen::MatrixXd>, _OutputSize>
//             &out) {
//         for (std::size_t i = 0; i < _InputSize; ++i) {
//             in_data_ptr_[i] = in[i].data();
//         }

//         for (std::size_t i = 0; i < _OutputSize; ++i) {
//             if (out[i]) {
//                 out_data_ptr_[i] = const_cast<double *>(out[i]->data());
//             } else {
//                 out_data_ptr_[i] = nullptr;
//             }
//         }

//         // Call the function
//         f_(in_data_ptr_.data(), out_data_ptr_.data(), iw_.data(), dw_.data(),
//            mem_);
//     }

//    private:
//     // Data input vector for casadi function
//     std::vector<const double *> in_data_ptr_;
//     // Data output pointers for casadi function
//     std::vector<double *> out_data_ptr_;

//     // Memory allocated for function evaluation
//     int mem_;

//     // Integer working vector
//     std::vector<casadi_int> iw_;
//     // Double working vector
//     std::vector<double> dw_;

//     // Underlying function
//     ::casadi::Function f_;
// };

// /**
//  * @brief Generic constraint function
//  *
//  */
// class Constraint : public bopt::Constraint {
//    public:
//     Constraint(const sym_t &ex, const sym_t &x, const sym_t &p = sym_t())
//         : bopt::Constraint(ex.size1(), x.size1(), p.size1()),
//           function_(nullptr),
//           jacobian_(nullptr),
//           hessian_(nullptr) {
//         // Compute Jacobian
//         sym_t jac = sym_t::jacobian(ex, x);

//         VLOG(10) << "Jacobian : " << jac;

//         // Function
//         function_ = std::make_unique<FunctionWrapper<2, 1>>(
//             ::casadi::Function("function", {x, p}, {densify(ex)}));
//         // Jacobian
//         jacobian_ = std::make_unique<FunctionWrapper<2, 1>>(
//             ::casadi::Function("jacobian", {x, p}, {densify(jac)}));

//         // Hessian
//         sym_t l = sym_t::sym("l", ex.size1());
//         sym_t lc = sym_t::mtimes(l.T(), ex);
//         sym_t hes = sym_t::hessian(lc, x);

//         hessian_ = std::make_unique<FunctionWrapper<3, 1>>(
//             ::casadi::Function("hessian", {x, p, l}, {densify(tril(hes))}));
//     }

//     VectorX evaluate(
//         const Base::InputType &x, const Base::InputType &p,
//         Base::OptionalDerivativeType jac = nullptr) const override {
//         Base::InputType out(this->sz_out());
//         function_->call({x, p}, {out});
//         if (jac) jacobian_->call({x, p}, {jac});
//         return out;
//     }

//     /**
//      * @brief Compute the lower triangular component of the Hessian matrix of
//      * the expression \f$ \lambda^T c(x, p) \f$.
//      *
//      * @param x
//      * @param lam
//      * @param hes
//      */
//     void hessian(const Base::InputType &x, const Base::InputType &p,
//                  const Base::InputType &lam,
//                  Base::OptionalHessianType hes = nullptr) const override {
//         if (hes) {
//             hessian_->call({x, p, lam}, {hes});
//         }
//     }

//    private:
//     FunctionWrapper<2, 1>::UniquePtr function_;
//     FunctionWrapper<2, 1>::UniquePtr jacobian_;
//     FunctionWrapper<3, 1>::UniquePtr hessian_;
// };

// class LinearConstraint : public bopt::LinearConstraint {
//    public:
//     // Override method
//     LinearConstraint(const sym_t &ex, const sym_t &x, const sym_t &p = sym_t())
//         : bopt::LinearConstraint(ex.size1(), x.size1(), p.size1()),
//           coeffs_(nullptr) {
//         // Create coefficients
//         sym_t A, b;
//         sym_t::linear_coeff(ex, x, A, b, true);
//         // Create function
//         coeffs_ = std::make_unique<FunctionWrapper<1, 2>>(
//             ::casadi::Function("linear_coeffs", {p}, {densify(A), densify(b)}));
//     }

//     void coeffs(const VectorX &p, Base::OptionalDerivativeType A = nullptr,
//                 OptionalEigenMatrix<VectorX> b = nullptr) const override {
//         coeffs_->call({p}, {A, b});
//     }

//    private:
//     FunctionWrapper<1, 2>::UniquePtr coeffs_;
// };

// class Cost : public bopt::Cost {
//    public:
//     // Override method
//     Cost(const sym_t &ex, const sym_t &x, const sym_t &p = sym_t())
//         : bopt::Cost(x.size1(), p.size1()),
//           function_(nullptr),
//           gradient_(nullptr),
//           hessian_(nullptr) {
//         VLOG(10) << "ex: ";
//         VLOG(10) << ex;
//         // Compute Jacobian
//         sym_t grd = sym_t::jacobian(ex, x);
//         VLOG(10) << "jac: ";
//         VLOG(10) << grd;
//         // Compute Hessian
//         sym_t hes = sym_t::hessian(ex, x);
//         VLOG(10) << "hes: ";
//         VLOG(10) << hes;
//         // Create function
//         function_ = std::make_unique<FunctionWrapper<2, 1>>(
//             ::casadi::Function("function", {x, p}, {densify(ex)}));
//         gradient_ = std::make_unique<FunctionWrapper<2, 1>>(
//             ::casadi::Function("jacobian", {x, p}, {densify(grd)}));
//         hessian_ = std::make_unique<FunctionWrapper<2, 1>>(
//             ::casadi::Function("hessian", {x, p}, {densify(tril(hes))}));
//     }

//     Base::OutputType evaluate(
//         const Base::InputType &x, const Base::InputType &p,
//         Base::OptionalDerivativeType grd = nullptr) const override {
//         Base::OutputType out;
//         function_->call({x, p}, {out});
//         if (grd) gradient_->call({x, p}, {grd});
//         return out;
//     }

//     /**
//      * @brief Computes the lower triangular component of the Hessian matrix
//      *
//      * @param x
//      * @param lam
//      * @param hes
//      */
//     void hessian(const Base::InputType &x, const Base::InputType &p,
//                  const Scalar &lam,
//                  Base::OptionalHessianType hes) const override {
//         if (hes) {
//             hessian_->call({x, p}, {hes});
//             (*hes) *= lam;
//         }
//     }

//    private:
//     FunctionWrapper<2, 1>::UniquePtr function_;
//     FunctionWrapper<2, 1>::UniquePtr gradient_;
//     FunctionWrapper<2, 1>::UniquePtr hessian_;
// };

// class LinearCost : public bopt::LinearCost {
//    public:
//     // Override method
//     LinearCost(const sym_t &ex, const sym_t &x, const sym_t &p = sym_t())
//         : bopt::LinearCost(x.size1(), p.size1()), coeffs_(nullptr) {
//         // Create coefficients
//         sym_t c, b;
//         sym_t::linear_coeff(ex, x, c, b, true);
//         // Create function
//         coeffs_ = std::make_unique<FunctionWrapper<1, 2>>(
//             ::casadi::Function("linear_coeffs", {p}, {densify(c), densify(b)}));
//     }

//     void coeffs(const VectorX &p, Base::OptionalDerivativeType c,
//                 double &b) const override {
//         coeffs_->call({p}, {c, b});
//     }

//    private:
//     FunctionWrapper<1, 2>::UniquePtr coeffs_;
// };

// class QuadraticCost : public bopt::QuadraticCost {
//    public:
//     // Override method
//     QuadraticCost(const sym_t &ex, const sym_t &x, const sym_t &p = sym_t())
//         : bopt::QuadraticCost(x.size1(), p.size1()), coeffs_(nullptr) {
//         // Create coefficients
//         sym_t A, b, c;
//         sym_t::quadratic_coeff(ex, x, A, b, c, true);
//         // Create function
//         // todo - check compile time that these number of outputs are correct
//         // for todo - the function
//         VLOG(10) << "A = " << tril(A);
//         VLOG(10) << "b = " << b;
//         VLOG(10) << "c = " << c;
//         coeffs_ = std::make_unique<FunctionWrapper<1, 3>>(::casadi::Function(
//             "quadratic_coeffs", {p},
//             {sym_t::densify(tril(A)), sym_t::densify(b), sym_t::densify(c)}));
//     }

//     void coeffs(const VectorX &p, Base::OptionalHessianType A,
//                 Base::OptionalDerivativeType b, double &c) const override {
//         // todo - consider whether these could be separated
//         coeffs_->call({p}, {A, b, c});
//     }

//    private:
//     FunctionWrapper<1, 3>::UniquePtr coeffs_;
// };

}  // namespace casadi
}  // namespace bopt

#endif /* bopt_CASADI_FUNCTION_HPP */
