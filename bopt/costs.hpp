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
template <typename FunctionTraits>
class CostTpl {
   public:
    using Scalar = typename FunctionTraits::Scalar;

    using InputVector = typename FunctionTraits::InputVector;
    using InputVectorConstRef = typename FunctionTraits::InputVectorConstRef;

    using Data = CostDataTpl<FunctionTraits>;

    /**
     * @brief Sets the name of the cost.
     *
     * @return const std::string&
     */
    const std::string &name() const { return name_; }
    void setName(const std::string &name) { name_ = name; }

    std::shared_ptr<Data> createData() const {
        return std::shared_ptr<Data>(this->createDataImpl());
    }

    /**
     * @brief Scaling factor for the objectivc.
     *
     * @return const double&
     */
    const double &scaling_factor() const { return scaling_factor_; }
    void setScalingFactor(const double &factor) { scaling_factor_ = factor; }

    /**
     * @brief Evaluates the expression y = fₚ(x) using variables x and
     * parameters p (set through \ref CostTpl::setParameters()).
     *
     * @param x The input vector (getInputDimension() x 1)
     * @param data
     */
    void eval(const InputVectorConstRef &x, Data &data) const {
        assert(x.rows() == getInputDimension());
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
    void evalGradients(const InputVectorConstRef &x, Data &data,
                       bool compute_x = true, bool compute_p = false) const {
        assert(x.rows() == getInputDimension());
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
    void evalSparseGradients(const InputVectorConstRef &x, Data &data,
                             bool compute_x = true,
                             bool compute_p = false) const {
        assert(x.rows() == getInputDimension());
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
    void evalHessians(const InputVectorConstRef &x, Data &data,
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
    void evalSparseHessians(const InputVectorConstRef &x, Data &data,
                            bool compute_xx = true, bool compute_xp = false,
                            bool compute_pp = false) const {
        evalSparseHessiansImpl(x, data, compute_xx, compute_xp, compute_pp);
    }

    /**
     * @brief Dimension of the input variable vector, commonly denoted as x.
     *
     * @return const Index&
     */
    const Index &getInputDimension() const { return dim_input_; }

    /**
     * @brief Dimension of the tangent space for the input vector, typically
     * this is equal to getInputDimension().
     *
     * @return const Index&
     */
    const Index &getInputTangentSpaceDimension() const {
        return dim_tangent_space_;
    }

    /**
     * @brief Dimension of the parameter vector p.
     *
     * @return const Index&
     */
    const Index &getNumberOfParameters() const { return num_parameters_; }

    const std::string &description() const { return description_; }

    void setDescription(const std::string &description) {
        description_ = description;
    }

    const InputVector &parameters() const { return parameters_; }

    /**
     * @brief Set the parameter vector p (getNumberOfParameters() x 1).
     *
     * @param p
     */
    void setParameters(const InputVectorConstRef &p) {
        assert(p.size() == getNumberOfParameters());
        parameters_ = p;
        if (ptr_) ptr_->setParameters(p);
    }

   protected:
    CostTpl(const Index &n_inputs, const std::string &description = "")
        : dim_input_(n_inputs),
          dim_tangent_space_(n_inputs),
          num_parameters_(0),
          name_(""),
          scaling_factor_(1.0),
          parameters_(InputVector::Zero(0)),
          description_(description),
          ptr_(nullptr) {}

    CostTpl(const std::shared_ptr<CostTpl<FunctionTraits>> &ptr)
        : dim_input_(ptr->getInputDimension()),
          dim_tangent_space_(ptr->getInputDimension()),
          num_parameters_(ptr->getNumberOfParameters()),
          name_(ptr->name()),
          scaling_factor_(ptr->scaling_factor()),
          parameters_(ptr->parameters()),
          description_(ptr->description()),
          ptr_(ptr) {}

    virtual Data *createDataImpl() const {
        auto data = new Data(*this);
        return data;
    }

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
    void setParameterDimension(const Index &dim) { num_parameters_ = dim; }

    /**
     * @brief Implementation of the evaluator
     *
     * @param x
     * @param out
     */
    virtual void evalImpl(const InputVectorConstRef &x, Data &data) const {
        VLOG(10) << "In bopt::CostTpl::evalImpl";
        if (ptr_) ptr_->eval(x, data);
    }

    /**
     * \copydoc CostTpl::evalGradients(const Eigen::Ref<const
     * VectorXd>, Data &)
     *
     */
    virtual void evalGradientsImpl(const InputVectorConstRef &x, Data &data,
                                   bool compute_x, bool compute_p) const {
        if (ptr_) ptr_->evalGradients(x, data, compute_x, compute_p);
    }

    /**
     * \copydoc CostTpl::evalHessians(const Eigen::Ref<const VectorXd>,
     * const Eigen::Ref<const VectorXd>, Data &)
     *
     */
    virtual void evalHessiansImpl(const InputVectorConstRef &x, Data &data,
                                  bool compute_xx, bool compute_xp,
                                  bool compute_pp) const {
        if (ptr_)
            ptr_->evalHessians(x, data, compute_xx, compute_xp, compute_pp);
    }

   private:
    /// @brief Dimension of the input vector
    Index dim_input_;
    Index dim_tangent_space_;
    Index num_parameters_;

    std::string name_;
    double scaling_factor_;

    InputVector parameters_;
    std::string description_;

    std::shared_ptr<CostTpl> ptr_;
};

template <typename Scalar>
using DenseCostTpl = CostTpl<DenseFunctionTraits<Scalar>>;

template <typename Scalar>
using SparseCostTpl = CostTpl<SparseFunctionTraits<Scalar>>;

template <typename FunctionTraits>
struct CostDataTpl {
    using Scalar = typename FunctionTraits::Scalar;

    using Vector = typename FunctionTraits::OutputVector;
    using Matrix = typename FunctionTraits::OutputMatrix;

    CostDataTpl(const CostTpl<FunctionTraits> &c) {
        if constexpr (FunctionTraits::type == "Sparse") {
            // Sparse: allocate sparse objects properly
            gx.resize(c.getInputTangentSpaceDimension());
            gp.resize(c.getNumberOfParameters());
            Hxx.resize(c.getInputTangentSpaceDimension(),
                       c.getInputTangentSpaceDimension());
            Hxp.resize(c.getInputTangentSpaceDimension(),
                       c.getNumberOfParameters());
            Hpp.resize(c.getNumberOfParameters(), c.getNumberOfParameters());
        } else {
            // Dense
            gx = Vector::Zero(c.getInputTangentSpaceDimension());
            gp = Vector::Zero(c.getNumberOfParameters());
            Hxx = Matrix::Zero(c.getInputTangentSpaceDimension(),
                               c.getInputTangentSpaceDimension());
            Hxp = Matrix::Zero(c.getInputTangentSpaceDimension(),
                               c.getNumberOfParameters());
            Hpp = Matrix::Zero(c.getNumberOfParameters(),
                               c.getNumberOfParameters());
        }
    }

    /// Evaluator output vector y
    Scalar y;

    /// Gradient for ∂y/∂x
    Vector gx;
    /// Gradient for ∂y/∂p
    Vector gp;

    /// Matrix for lower-triangular hessian ∂²(λᵀy)/∂x²
    Matrix Hxx;
    /// Matrix for lower-triangular hessian ∂²(λᵀy)/∂x∂p
    Matrix Hxp;
    /// Matrix for lower-triangular hessian ∂²(λᵀy)/∂p²
    Matrix Hpp;
};

template <typename FunctionTraits>
std::ostream &operator<<(std::ostream &os, const CostTpl<FunctionTraits> &c) {
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
template <typename FunctionTraits>
class LinearCostTpl : public CostTpl<FunctionTraits> {
   public:
    using Base = CostTpl<FunctionTraits>;
    using EvaluatorData = typename Base::Data;

    using Data = LinearCostDataTpl<FunctionTraits>;

    std::shared_ptr<Data> createData() const {
        return std::shared_ptr<Data>(this->createDataImpl());
    }

    /**
     * @brief Evaluates the vector coeffcient vector bₚ for the cost fₚ(x) = aₚ
     * x + bₚ
     *
     * @param a Coefficient vector aₚ
     * @param b Constant bₚ
     */
    void evalCoefficients(Data &data) const { evalCoefficientsImpl(data); }

   protected:
    LinearCostTpl(const Index &dim_input) : CostTpl<FunctionTraits>(dim_input) {
        this->setName("linear_cost");
    }

    LinearCostTpl(const std::shared_ptr<CostTpl<FunctionTraits>> &cost)
        : CostTpl<FunctionTraits>(cost) {
        this->setName("linear_cost");
    }

    virtual void evalCoefficientsImpl(Data &data) const {}

    Data *createDataImpl() const override {
        auto data = new Data(*this);
        return data;
    }

   private:
};

template <typename Scalar>
using DenseLinearCostTpl = LinearCostTpl<DenseFunctionTraits<Scalar>>;

template <typename Scalar>
using SparseLinearCostTpl = LinearCostTpl<SparseFunctionTraits<Scalar>>;

/**
 * @brief Contains the data associated with a linear cost
 *
 * @tparam FunctionTraits
 */
template <typename FunctionTraits>
struct LinearCostDataTpl : public CostDataTpl<FunctionTraits> {
    using Scalar = typename FunctionTraits::Scalar;

    using Vector = typename FunctionTraits::OutputVector;
    using Matrix = typename FunctionTraits::OutputMatrix;

    LinearCostDataTpl(const LinearCostTpl<FunctionTraits> &c)
        : CostDataTpl<FunctionTraits>(c) {
        if constexpr (FunctionTraits::type == "Sparse") {
            // Sparse: allocate sparse objects properly
            a.resize(c.getInputDimension());
        } else {
            // Dense
            a = Vector::Zero(c.getInputDimension());
        }
    }

    /// Dense coefficient vector a
    Vector a;
    /// Constant term b
    Scalar b;
};

/**
 * @brief Types of hessians
 *
 */
enum class HessianType { kPositiveDefinite, kPositiveSemiDefinite, Indefinite };

template <typename FunctionTraits>
struct QuadraticCostDataTpl;

/**
 * @brief Quadratic cost of the form fₚ(x) = (1/2) xᵀ Aₚ x + bₚᵀ x + cₚ
 *
 */
template <typename FunctionTraits>
class QuadraticCostTpl : public CostTpl<FunctionTraits> {
   public:
    using Data = QuadraticCostDataTpl<FunctionTraits>;

    std::shared_ptr<Data> createData() const {
        return std::shared_ptr<Data>(this->createDataImpl());
    }

    /**
     * @brief Evaluates the vector coeffcient vector bₚ for the cost fₚ(x) =
     * (1/2) xᵀ Aₚ x + bₚᵀ x + cₚ
     *
     * @param A Lower triangular matrix Aₚ
     * @param b Vector bₚ
     * @param c Constant cₚ
     */
    void evalCoefficients(Data &data) const { evalCoefficientsImpl(data); }

   protected:
    QuadraticCostTpl<FunctionTraits>(const Index &dim_input)
        : CostTpl<FunctionTraits>(dim_input) {
        this->setName("quadratic_cost");
    }

    QuadraticCostTpl<FunctionTraits>(
        const std::shared_ptr<CostTpl<FunctionTraits>> &cost)
        : CostTpl<FunctionTraits>(cost) {
        this->setName("quadratic_cost");
    }

    virtual Data *createDataImpl() const { return new Data(*this); }

    virtual void evalCoefficientsImpl(Data &data) const {}

   private:
};

typedef QuadraticCostTpl<double> QuadraticCost;

template <typename FunctionTraits>
struct QuadraticCostDataTpl : public CostDataTpl<FunctionTraits> {
    using Scalar = typename FunctionTraits::Scalar;

    using Vector = typename FunctionTraits::OutputVector;
    using Matrix = typename FunctionTraits::OutputMatrix;

    QuadraticCostDataTpl(const QuadraticCostTpl<FunctionTraits> &c)
        : CostDataTpl<FunctionTraits>(c) {
        if constexpr (FunctionTraits::type == "Sparse") {
            // Sparse: allocate sparse objects properly
            A.resize(c.getInputDimension(), c.getInputDimension());
            b.resize(c.getInputDimension(), c.getInputDimension());
        } else {
            // Dense
            A = Matrix::Zero(c.getInputDimension(), c.getInputDimension());
            b = Vector::Zero(c.getInputDimension());
        }
    }

    Matrix A;
    Vector b;
    Scalar c;
};

}  // namespace bopt
