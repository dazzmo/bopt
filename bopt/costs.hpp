#pragma once

#include <memory>

#include "bopt/types.hpp"

namespace bopt {

template <typename T>
struct cost_traits {
    typedef typename T::ptr_type ptr_type;
    typedef typename T::value_type value_type;
    typedef typename T::index_type index_type;
    typedef typename T::integer_type integer_type;

    typedef typename T::shared_ptr shared_ptr;
};

template <typename T>
struct cost_attributes {
    // bool has_gradient(const T &cost) const {return cost}
};

template <class ValueType, class IntegerType = int,
          class IndexType = std::size_t>
class cost : public evaluator<ValueType, IntegerType, IndexType> {
   public:
    typedef evaluator<ValueType, IntegerType, IndexType> Base;

    typedef typename Base::value_type value_type;
    typedef typename Base::index_type index_type;
    typedef typename Base::integer_type integer_type;

    typedef IndexType id_type;

   public:
    id_type id;
    std::string name;

    virtual integer_type jac(const value_type **arg, value_type *res) {
        return 0;
    }

    virtual integer_type hes(const value_type **arg, const value_type **lam,
                             value_type *res) {
        return 0;
    }

    virtual integer_type jac_info(evaluator_out_info<cost> &info) { return 0; }

    virtual integer_type hes_info(evaluator_out_info<cost> &info) { return 0; }

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
    typedef typename cost<T>::index_type index_type;
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
