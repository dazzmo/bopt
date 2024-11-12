#pragma once

#include <memory>

#include "bopt/types.hpp"

namespace bopt {

template <typename T>
struct cost_traits {
    typedef typename T::ptr_type ptr_type;
    typedef typename T::value_type value_type;
    typedef typename T::integer_type integer_type;
    typedef typename T::input_vector input_vector;
};

template <typename T>
struct cost_attributes {
    // bool has_gradient(const T &cost) const {return cost}
};

template <class T, class I = std::size_t>
class cost : public evaluator<T> {
   public:
    typedef T value_type;
    typedef int integer_type;
    typedef I id_type;
    typedef std::shared_ptr<cost<T, I>> ptr_type;
    typedef std::vector<T> input_vector;

   public:
    id_type id;
    std::string name;

    virtual index_type jac(const value_type **arg, value_type *res) {
        return 0;
    }

    virtual index_type hes(const value_type **arg, const value_type **lam,
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
template <typename T>
class linear_cost : public cost<T> {
   public:
    using UniquePtr = std::unique_ptr<linear_cost>;
    using SharedPtr = std::shared_ptr<linear_cost>;

    typedef typename cost<T>::value_type value_type;
    typedef typename cost<T>::integer_type integer_type;

    linear_cost() = default;

    virtual integer_type a(const value_type **arg, value_type *res) {
        return 0;
    }
    virtual integer_type a_info(evaluator_out_info<linear_cost> &info) {
        return 0;
    }

    virtual integer_type b(const value_type **arg, value_type *res) {
        return 0;
    }
    virtual integer_type b_info(evaluator_out_info<linear_cost> &info) {
        return 0;
    }

   protected:
};

/**
 * @brief A cost of the form 0.5 x^T A x + b^T x + c
 *
 */
template <typename T>
class quadratic_cost : public cost<T> {
   public:
    using UniquePtr = std::unique_ptr<quadratic_cost>;
    using SharedPtr = std::shared_ptr<quadratic_cost>;

    typedef typename cost<T>::value_type value_type;
    typedef typename cost<T>::integer_type integer_type;

    virtual integer_type A(const double **arg, double *res) { return 0; }
    virtual integer_type A_info(evaluator_out_info<quadratic_cost> &info) {
        return 0;
    }

    virtual integer_type b(const double **arg, double *res) { return 0; }
    virtual integer_type b_info(evaluator_out_info<quadratic_cost> &info) {
        return 0;
    }

    virtual integer_type c(const double **arg, double *res) { return 0; }
    virtual integer_type c_info(evaluator_out_info<quadratic_cost> &info) {
        return 0;
    }

   protected:
};

}  // namespace bopt
