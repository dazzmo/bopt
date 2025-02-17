#pragma once

#include <memory>

#include "bopt/evaluator.hpp"
#include "bopt/logging.hpp"

namespace bopt {

template <typename Scalar>
struct CostDataTpl;

/**
 *
 * @brief Cost function y = fₚ(x) ∈ ℝ
 *
 */
template <typename Scalar>
class CostTpl {
   public:
    using CostData = CostDataTpl<Scalar>;

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

    /**
     * @brief Evaluates the expression y = fₚ(x) using variables x and
     * parameters p (set through \ref CostTpl::setParameters()).
     *
     * @param x The input vector (dim_input() x 1)
     * @param data
     */
    void eval(const Eigen::Ref<const VectorXd> &x, CostData &data) const {
        BOPT_ASSERT(x.rows() == dim_input());
        checkVector(x);
        evalImpl(x, data);
    }

    /**
     * @brief Computes the jacobians of the expression f.
     *
     * @param x
     * @param data
     * @param compute_x Compute ∂f/∂x
     * @param compute_p Compute ∂f/∂p
     */
    void evalGradients(const Eigen::Ref<const VectorXd> &x, CostData &data,
                       bool compute_x = true, bool compute_p = false) const {
        BOPT_ASSERT(x.rows() == dim_input());
        evalGradientsImpl(x, data, compute_x, compute_p);
    }

    /**
     * @brief Computes the sparse jacobians of the expression f.
     *
     * @param x
     * @param data
     * @param compute_x Compute ∂f/∂x
     * @param compute_p Compute ∂f/∂p
     */
    void evalSparseGradients(const Eigen::Ref<const VectorXd> &x,
                             CostData &data, bool compute_x = true,
                             bool compute_p = false) const {
        BOPT_ASSERT(x.rows() == dim_input());
        evalSparseGradientsImpl(x, data, compute_x, compute_p);
    }

    /**
     * @brief Computes the lower-triangular hessians of the vector-product of
     * the expression f.
     *
     * @param x
     * @param data
     * @param compute_xx Compute ∂²f/∂x²
     * @param compute_xp Compute ∂²f/∂x∂p
     * @param compute_pp Compute ∂²f/∂p²
     */
    void evalHessians(const Eigen::Ref<const VectorXd> &x, CostData &data,
                      bool compute_xx = true, bool compute_xp = false,
                      bool compute_pp = false) const {
        evalHessiansImpl(x, data, compute_xx, compute_xp, compute_pp);
    }

    /**
     * @brief Computes the sparse lower-triangular hessians of the
     * vector-product of the expression f.
     *
     * @param x
     * @param data
     * @param compute_xx Compute ∂²f/∂x²
     * @param compute_xp Compute ∂²f/∂x∂p
     * @param compute_pp Compute ∂²f/∂p²
     */
    void evalSparseHessians(const Eigen::Ref<const VectorXd> &x, CostData &data,
                            bool compute_xx = true, bool compute_xp = false,
                            bool compute_pp = false) const {
        evalSparseHessiansImpl(x, data, compute_xx, compute_xp, compute_pp);
    }

    /**
     * @brief Provides the sparsity patterns for the sparse
     * gradients ∂f/∂x and ∂f/∂p (if applicable).
     *
     */
    virtual void setGradientSparsityPatterns(CostData &data) const {
        if (ptr_) ptr_->setGradientSparsityPatterns(data);
    }

    /**
     * @brief Provides the sparsity patterns for the lower-triangular Hessians
     * ∂²f/∂x², ∂²f/∂x∂p, ∂²f/∂p² ∀λ (if applicable)
     *
     */
    virtual void setHessianSparsityPatterns(CostData &data) const {
        if (ptr_) ptr_->setHessianSparsityPatterns(data);
    }

    /**
     * @brief Dimension of the input variable vector, commonly denoted as x.
     *
     * @return const Index&
     */
    const Index &dim_input() const { return dim_input_; }

    /**
     * @brief Dimension of the tangent space for the input vector, typically
     * this is equal to dim_input().
     *
     * @return const Index&
     */
    const Index &dim_tangent_space() const { return dim_tangent_space_; }

    /**
     * @brief Dimension of the parameter vector p.
     *
     * @return const Index&
     */
    const Index &dim_parameter() const { return dim_parameter_; }

    const std::string &description() const { return description_; }

    void setDescription(const std::string &description) {
        description_ = description;
    }

    const VectorXd &parameters() const { return parameters_; }

    /**
     * @brief Set the parameter vector p (dim_parameter() x 1).
     *
     * @param p
     */
    void setParameters(const Eigen::Ref<const VectorXd> &p) {
        BOPT_ASSERT(p.size() == dim_parameter());
        parameters_ = p;
        if (ptr_) ptr_->setParameters(p);
    }

   protected:
    CostTpl(const Index &n_inputs, const std::string &description = "")
        : dim_input_(n_inputs),
          dim_tangent_space_(n_inputs),
          dim_parameter_(0),
          name_(""),
          scaling_factor_(1.0),
          parameters_(VectorXd::Zero(0)),
          description_(description),
          ptr_(nullptr) {}

    CostTpl(const std::shared_ptr<CostTpl<Scalar>> &ptr)
        : dim_input_(ptr->dim_input()),
          dim_tangent_space_(ptr->dim_input()),
          dim_parameter_(ptr->dim_parameter()),
          name_(ptr->name()),
          scaling_factor_(ptr->scaling_factor()),
          parameters_(ptr->parameters()),
          description_(ptr->description()),
          ptr_(ptr) {}

    /**
     * @brief Sets the dimension of the tangent space for the input variables.
     *
     * @param dim Dimension of the vector
     */
    void setTangentSpaceDimension(const Index &dim) {
        dim_tangent_space_ = dim;
    }

    /**
     * @brief Sets the dimension of the evaluator parameter vector.
     *
     * @param dim Dimension of the vector
     */
    void setParameterDimension(const Index &dim) { dim_parameter_ = dim; }

    /**
     * @brief Implementation of the evaluator
     *
     * @param x
     * @param out
     */
    virtual void evalImpl(const Eigen::Ref<const VectorXd> &x,
                          CostData &data) const {
        VLOG(10) << "In bopt::CostTpl::evalImpl";
        if (ptr_) ptr_->eval(x, data);
    }

    /**
     * \copydoc CostTpl::evalGradients(const Eigen::Ref<const
     * VectorXd>, CostData &)
     *
     */
    virtual void evalGradientsImpl(const Eigen::Ref<const VectorXd> &x,
                                   CostData &data, bool compute_x,
                                   bool compute_p) const {
        if (ptr_) ptr_->evalGradients(x, data, compute_x, compute_p);
    }

    /**
     * \copydoc CostTpl::evalSparseGradients(const Eigen::Ref<const
     * VectorXd>, CostData &)
     *
     */
    virtual void evalSparseGradientsImpl(const Eigen::Ref<const VectorXd> &x,
                                         CostData &data, bool compute_x,
                                         bool compute_p) const {
        if (ptr_) ptr_->evalSparseGradients(x, data, compute_x, compute_p);
    }
    /**
     * \copydoc CostTpl::evalHessians(const Eigen::Ref<const VectorXd>,
     * const Eigen::Ref<const VectorXd>, CostData &)
     *
     */
    virtual void evalHessiansImpl(const Eigen::Ref<const VectorXd> &x,
                                  CostData &data, bool compute_xx,
                                  bool compute_xp, bool compute_pp) const {
        if (ptr_)
            ptr_->evalHessians(x, data, compute_xx, compute_xp, compute_pp);
    }

    /**
     * \copydoc CostTpl::evalSparseHessians(const Eigen::Ref<const
     * VectorXd>, const Eigen::Ref<const VectorXd>, CostData &)
     *
     */
    virtual void evalSparseHessiansImpl(const Eigen::Ref<const VectorXd> &x,
                                        CostData &data, bool compute_xx,
                                        bool compute_xp,
                                        bool compute_pp) const {
        if (ptr_)
            ptr_->evalSparseHessians(x, data, compute_xx, compute_xp,
                                     compute_pp);
    }

   private:
    /// @brief Dimension of the input vector
    Index dim_input_;
    Index dim_tangent_space_;
    Index dim_parameter_;

    std::string name_;
    double scaling_factor_;

    VectorXd parameters_;
    std::string description_;

    std::shared_ptr<CostTpl> ptr_;
};

typedef CostTpl<double> Cost;

template <typename Scalar>
struct CostDataTpl {
    CostDataTpl(const CostTpl<Scalar> &e)
        : f(0.0),
          gx(VectorX<Scalar>::Zero(e.dim_tangent_space())),
          gp(VectorX<Scalar>::Zero(e.dim_parameter())),
          Hxx(MatrixX<Scalar>::Zero(e.dim_tangent_space(),
                                    e.dim_tangent_space())),
          Hxp(MatrixX<Scalar>::Zero(e.dim_tangent_space(), e.dim_parameter())),
          Hpp(MatrixX<Scalar>::Zero(e.dim_parameter(), e.dim_parameter())),
          gx_s(e.dim_tangent_space()),
          gp_s(e.dim_tangent_space()),
          Hxx_s(e.dim_tangent_space(), e.dim_tangent_space()),
          Hxp_s(e.dim_tangent_space(), e.dim_parameter()),
          Hpp_s(e.dim_parameter(), e.dim_parameter()) {
        e.setGradientSparsityPatterns(*this);
        e.setHessianSparsityPatterns(*this);
    }

    /// Cost value f
    Scalar f;

    /// Dense vector for gradient ∂f/∂x
    VectorX<Scalar> gx;
    /// Dense vector for gradient ∂f/∂p
    VectorX<Scalar> gp;

    /// Sparse matrix for ∂f/∂x
    SparseVector<Scalar> gx_s;
    /// Sparse matrix for ∂f/∂p
    SparseVector<Scalar> gp_s;

    /// Dense matrix for lower-triangular matrix ∂²f/∂x²
    MatrixX<Scalar> Hxx;
    /// Dense matrix for lower-triangular matrix ∂²f/∂x∂p
    MatrixX<Scalar> Hxp;
    /// Dense matrix for lower-triangular matrix ∂²f/∂p²
    MatrixX<Scalar> Hpp;

    /// Sparse matrix for lower-triangular matrix ∂²f/∂x²
    SparseMatrix<Scalar> Hxx_s;
    /// Sparse matrix for lower-triangular matrix ∂²f/∂x∂p
    SparseMatrix<Scalar> Hxp_s;
    /// Sparse matrix for lower-triangular matrix ∂²f/∂p²
    SparseMatrix<Scalar> Hpp_s;
};

typedef CostDataTpl<double> CostData;

template <typename Scalar>
std::ostream &operator<<(std::ostream &os, const CostTpl<Scalar> &c) {
    os << "cost:\n";
    os << "name: " << c.name() << '\n';
    os << "scaling factor: " << c.scaling_factor() << '\n';
    os << "description: " << c.description();
    return os;
}

template <typename Scalar>
struct LinearCostDataTpl;

/**
 * @brief Linear cost of the form fₚ(x) = aₚᵀx + bₚ
 *
 */
template <typename Scalar>
class LinearCostTpl : public CostTpl<Scalar> {
   public:
    using LinearCostData = LinearCostDataTpl<Scalar>;

    /**
     * @brief Evaluates the vector coeffcient vector bₚ for the cost fₚ(x) = aₚ
     * x + bₚ
     *
     * @param a Coefficient vector aₚ
     * @param b Constant bₚ
     */
    void evalCoefficients(LinearCostData &data) const {
        evalCoefficientsImpl(data);
    }

    void evalSparseCoefficients(LinearCostData &data) const {
        evalSparseCoefficientsImpl(data);
    }

    void setCoefficientSparsityPatterns(LinearCostData &data) const {
        setCoefficientSparsityPatternsImpl(data);
    }

   protected:
    LinearCostTpl(const Index &dim_input) : CostTpl<Scalar>(dim_input) {
        this->setName("linear_cost");
    }

    LinearCostTpl(const std::shared_ptr<CostTpl<Scalar>> &cost)
        : CostTpl<Scalar>(cost) {
        this->setName("linear_cost");
    }

    virtual void evalCoefficientsImpl(LinearCostData &data) const {}

    virtual void evalSparseCoefficientsImpl(LinearCostData &data) const {}

    virtual void setCoefficientSparsityPatternsImpl(
        LinearCostData &data) const {}

   private:
};

typedef LinearCostTpl<double> LinearCost;

template <typename Scalar>
struct LinearCostDataTpl : public CostDataTpl<Scalar> {
    LinearCostDataTpl(const LinearCostTpl<Scalar> &c)
        : CostDataTpl<Scalar>(c),
          a(VectorX<Scalar>::Zero(c.dim_input())),
          b(0),
          a_s(SparseVector<Scalar>(c.dim_input())) {
        c.setCoefficientSparsityPatterns(*this);
    }

    /// Dense coefficient vector a
    VectorX<Scalar> a;
    /// Constant term b
    Scalar b;

    /// Sparse coefficient vector a
    SparseVector<Scalar> a_s;
};

typedef LinearCostDataTpl<double> LinearCostData;

/**
 * @brief Types of hessians
 *
 */
enum class HessianType { kPositiveDefinite, kPositiveSemiDefinite, Indefinite };

template <typename Scalar>
struct QuadraticCostDataTpl;

/**
 * @brief Quadratic cost of the form fₚ(x) = (1/2) xᵀ Aₚ x + bₚᵀ x + cₚ
 *
 */
template <typename Scalar>
class QuadraticCostTpl : public CostTpl<Scalar> {
   public:
    using QuadraticCostData = QuadraticCostDataTpl<Scalar>;

    /**
     * @brief Evaluates the vector coeffcient vector bₚ for the cost fₚ(x) =
     * (1/2) xᵀ Aₚ x + bₚᵀ x + cₚ
     *
     * @param A Lower triangular matrix Aₚ
     * @param b Vector bₚ
     * @param c Constant cₚ
     */
    void evalCoefficients(QuadraticCostData &data) const {
        evalCoefficientsImpl(data);
    }

    void evalSparseCoefficients(QuadraticCostData &data) const {
        evalSparseCoefficientsImpl(data);
    }

    void setCoefficientSparsityPatterns(QuadraticCostData &data) const {
        setCoefficientSparsityPatternsImpl(data);
    }

   protected:
    QuadraticCostTpl<Scalar>(const Index &dim_input)
        : CostTpl<Scalar>(dim_input) {
        this->setName("quadratic_cost");
    }

    QuadraticCostTpl<Scalar>(const std::shared_ptr<CostTpl<Scalar>> &cost)
        : CostTpl<Scalar>(cost) {
        this->setName("quadratic_cost");
    }

    virtual void evalCoefficientsImpl(QuadraticCostData &data) const {}

    virtual void evalSparseCoefficientsImpl(QuadraticCostData &data) const {}

    virtual void setCoefficientSparsityPatternsImpl(
        QuadraticCostData &data) const {}

   private:
};

typedef QuadraticCostTpl<double> QuadraticCost;

template <typename Scalar>
struct QuadraticCostDataTpl : public CostDataTpl<Scalar> {
    QuadraticCostDataTpl(const QuadraticCostTpl<Scalar> &c)
        : CostDataTpl<Scalar>(c),
          A(MatrixX<Scalar>::Zero(c.dim_input(), c.dim_input())),
          b(VectorX<Scalar>::Zero(c.dim_input())),
          c(0),
          A_s(SparseMatrix<Scalar>(c.dim_input(), c.dim_input())),
          b_s(SparseVector<Scalar>(c.dim_input())) {
        c.setCoefficientSparsityPatterns(*this);
    }

    /// Dense coefficient matrix A (lower triangular)
    MatrixX<Scalar> A;
    /// Dense coefficient vector b
    VectorX<Scalar> b;
    /// Constant term c
    Scalar c;

    /// Sparse coefficient matrix A (lower triangular)
    SparseMatrix<Scalar> A_s;
    /// Sparse coefficient vector b
    SparseVector<Scalar> b_s;
};

typedef QuadraticCostDataTpl<double> QuadraticCostData;

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
