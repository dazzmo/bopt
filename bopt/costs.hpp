#pragma once

#include <memory>

#include "bopt/evaluator.hpp"
#include "bopt/logging.hpp"

namespace bopt {

/**
 * @brief Cost function y = fₚ(x) ∈ ℝ
 *
 */
class Cost : public EvaluatorBase {
   public:
    /**
     * @brief Sets the name of the cost.
     *
     * @return const std::string&
     */
    const std::string &name() const { return name_; }
    void setName(const std::string &name) { name_ = name; }

    /**
     * @brief Scaling factor for the objective.
     *
     * @return const double&
     */
    const double &scaling_factor() const { return scaling_factor_; }
    void setScalingFactor(const double &factor) { scaling_factor_ = factor; }

   protected:
    Cost(const Index &n_inputs, const std::string &name = "")
        : EvaluatorBase(n_inputs, 1), name_(name), scaling_factor_(1.0) {}

   private:
    std::string name_;
    double scaling_factor_;
};

std::ostream &operator<<(std::ostream &os, const Cost &c);

/**
 * @brief Linear cost of the form fₚ(x) = aₚᵀx + bₚ
 *
 */
class LinearCost : public Cost {
   public:
    /**
     * @brief Evaluates the coefficient vector aₚ for the cost fₚ(x) = aₚᵀx + bₚ
     *
     * @param a
     */
    void evala(Eigen::Ref<VectorXd> a) { return evalaImpl(a); }

    /**
     * @brief Evaluates the constant value bₚ for the cost fₚ(x) = aₚᵀx + bₚ
     *
     * @param b
     */
    void evalb(double &b) { return evalbImpl(b); }

    bool a_has_nz_only() const { return a_has_nz_only_; }

    const std::optional<SparsityPattern> &a_sparsity_pattern() const {
        return a_sparsity_pattern_;
    }

    void setaSparsityPattern(const SparsityPattern &pattern) {
        a_sparsity_pattern_ = pattern;
    }

   protected:
    LinearCost(const Index &dim_input)
        : Cost(dim_input),
          a_has_nz_only_(false),
          a_sparsity_pattern_(std::nullopt) {
        setName("linear_cost");
    }

    /**
     * @brief Indicate whether the evaluation of aₚ only includes the non-zero
     * elements
     *
     * @param flag
     */
    void setANonZeroOnly(bool flag) { a_has_nz_only_ = flag; }

    void evalJacobianImpl(const Eigen::Ref<const VectorXd> &x,
                          Eigen::Ref<MatrixXd> jacobian) override {
        evala(jacobian);
    }

    virtual void evalaImpl(Eigen::Ref<VectorXd> a) {}
    virtual void evalbImpl(double &b) {}

   private:
    bool a_has_nz_only_;
    std::optional<SparsityPattern> a_sparsity_pattern_;
};

/**
 * @brief Quadratic cost of the form fₚ(x) = xᵀ Aₚ x + bₚᵀ x + cₚ
 *
 */
class QuadraticCost : public Cost {
   public:
    /**
     * @brief Hessian type Aₚ for the quadratic cost.
     *
     */
    enum class HessianType {
        kPositiveDefinite,
        kPositiveSemiDefinite,
        Indefinite
    };

    /**
     * @brief Evaluates the coefficient matrix Aₚ for the cost fₚ(x) = xᵀ Aₚ x +
     * bₚᵀ x + cₚ
     *
     * @param a
     */
    void evalA(Eigen::Ref<MatrixXd> A) { return evalAImpl(A); }

    /**
     * @brief Evaluates the vector coeffcient vector bₚ for the cost fₚ(x) = xᵀ
     * Aₚ x + bₚᵀ x + cₚ
     *
     * @param b
     */
    void evalb(Eigen::Ref<VectorXd> b) { return evalbImpl(b); }

    /**
     * @brief Evaluates the constant cₚ for the cost fₚ(x) = xᵀ Aₚ x + bₚᵀ x +
     * cₚ
     *
     * @param c
     */
    void evalc(double &c) { return evalcImpl(c); }

    bool A_has_nz_only() const { return A_has_nz_only_; }
    bool b_has_nz_only() const { return b_has_nz_only_; }

    const std::optional<SparsityPattern> &A_sparsity_pattern() const {
        return A_sparsity_pattern_;
    }

    const std::optional<SparsityPattern> &b_sparsity_pattern() const {
        return b_sparsity_pattern_;
    }

    void setASparsityPattern(const SparsityPattern &pattern) {
        A_sparsity_pattern_ = pattern;
    }
    void setbSparsityPattern(const SparsityPattern &pattern) {
        b_sparsity_pattern_ = pattern;
    }

   protected:
    QuadraticCost(const Index &dim_input)
        : Cost(dim_input),
          A_has_nz_only_(false),
          A_sparsity_pattern_(std::nullopt),
          b_has_nz_only_(false),
          b_sparsity_pattern_(std::nullopt) {
        setName("quadratic_cost");
    }

    virtual void evalAImpl(Eigen::Ref<MatrixXd> A) {}
    virtual void evalbImpl(Eigen::Ref<VectorXd> b) {}
    virtual void evalcImpl(double &c) {}

    /**
     * @brief Indicate whether the evaluation of Aₚ only includes the non-zero
     * elements
     *
     * @param flag
     */
    void setANonZeroOnly(bool flag) { A_has_nz_only_ = flag; }

    /**
     * @brief Indicate whether the evaluation of bₚ only includes the non-zero
     * elements
     *
     * @param flag
     */
    void setbNonZeroOnly(bool flag) { b_has_nz_only_ = flag; }

   private:
    bool A_has_nz_only_;
    std::optional<SparsityPattern> A_sparsity_pattern_;

    bool b_has_nz_only_;
    std::optional<SparsityPattern> b_sparsity_pattern_;
};

// class LeastSquaresCost : public QuadraticCost {
//    public:
//     LeastSquaresCost(const std::shared_ptr<LinearCost> &linear_cost) {}

//    protected:
//     void evalImpl(const Eigen::Ref<const VectorXd> &x, double &out) {
//         // linear_cost_->eval(x, out);
//         // setA(linear_cost_->a().transpose() * linear_cost_->a());
//     }

//    private:
// };

}  // namespace bopt
