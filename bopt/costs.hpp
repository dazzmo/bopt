#pragma once

#include <memory>

#include "bopt/function.hpp"
#include "bopt/types.hpp"

namespace bopt {

template <typename Cost>
struct cost_traits {
    bool has_gradient;
    bool has_hessian;
};

template <class T>
class cost : public EvaluatorBase<T> {
   public:
    typedef typename T value_type;

   public:
    id_type id;
    std::string name;

    virtual bopt_int jac(const value_type **arg, value_type *res) { return 0; }

    virtual bopt_int hes(const value_type **arg, const value_type **lam,
                         value_type *res) {
        return 0;
    }

    virtual index_type jac_info(evaluator_out_info<cost> &info) { return 0; }

    virtual index_type hes_info(evaluator_out_info<cost> &info) { return 0; }

   private:
};

/**
 * @brief Linear cost of the form \f$ c(x, p) = a^T(p) x + b(p) \in \mathbb{R}
 * \f$.
 *
 */
class LinearCost : public cost {
   public:
    using UniquePtr = std::unique_ptr<LinearCost>;
    using SharedPtr = std::shared_ptr<LinearCost>;

    LinearCost() = default;

    virtual bopt_int a(const double **arg, double *res) { return 0; }
    virtual bopt_int a_info(evaluator_out_info<LinearCost> &info) { return 0; }

    virtual bopt_int b(const double **arg, double *res) { return 0; }
    virtual bopt_int b_info(evaluator_out_info<LinearCost> &info) { return 0; }

   protected:
};

/**
 * @brief A cost of the form 0.5 x^T A x + b^T x + c
 *
 */
class QuadraticCost : public cost {
   public:
    using UniquePtr = std::unique_ptr<QuadraticCost>;
    using SharedPtr = std::shared_ptr<QuadraticCost>;

    virtual bopt_int A(const double **arg, double *res) { return 0; }
    virtual bopt_int A_info(evaluator_out_info<QuadraticCost> &info) { return 0; }

    virtual bopt_int b(const double **arg, double *res) { return 0; }
    virtual bopt_int b_info(evaluator_out_info<QuadraticCost> &info) { return 0; }

    virtual bopt_int c(const double **arg, double *res) { return 0; }
    virtual bopt_int c_info(evaluator_out_info<QuadraticCost> &info) { return 0; }

   protected:
};

}  // namespace bopt
