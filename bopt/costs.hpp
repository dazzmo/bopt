#pragma once

#include <memory>

#include "bopt/types.hpp"
#include "bopt/evaluator.hpp"

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
    typedef evaluator<ValueType, IntegerType, IndexType> evaluator_t;

    typedef typename evaluator_t::value_type value_type;
    typedef typename evaluator_t::index_type index_type;
    typedef typename evaluator_t::integer_type integer_type;
    typedef typename evaluator_t::out_info_t out_info_t;
    typedef typename evaluator_t::out_data_t out_data_t;

    typedef std::shared_ptr<cost> shared_ptr;

    typedef IndexType id_type;

   public:
    id_type id;
    std::string name;

    virtual integer_type jac(const value_type **arg, value_type *res) {
        return 0;
    }

    virtual integer_type hes(const value_type **arg, value_type *res) {
        return 0;
    }

    virtual integer_type jac_info(out_info_t &info) { return 0; }

    virtual integer_type hes_info(out_info_t &info) { return 0; }

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
    typedef typename cost<T>::value_type value_type;
    typedef typename cost<T>::index_type index_type;
    typedef typename cost<T>::integer_type integer_type;

    typedef typename cost<T>::evaluator_t evaluator_t;

    typedef typename evaluator_t::out_info_t out_info_t;
    typedef typename evaluator_t::out_data_t out_data_t;

    typedef std::shared_ptr<linear_cost> shared_ptr;

    linear_cost() = default;

    virtual integer_type a(const value_type **arg, value_type *res) {
        return 0;
    }
    virtual integer_type a_info(out_info_t &info) { return 0; }

    virtual integer_type b(const value_type **arg, value_type *res) {
        return 0;
    }
    virtual integer_type b_info(out_info_t &info) { return 0; }

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

    typedef typename cost<T>::evaluator_t evaluator_t;

    typedef typename evaluator_t::out_info_t out_info_t;
    typedef typename evaluator_t::out_data_t out_data_t;

    typedef std::shared_ptr<quadratic_cost> shared_ptr;

    virtual integer_type A(const double **arg, double *res) { return 0; }
    virtual integer_type A_info(out_info_t &info) { return 0; }

    virtual integer_type b(const double **arg, double *res) { return 0; }
    virtual integer_type b_info(out_info_t &info) { return 0; }

    virtual integer_type c(const double **arg, double *res) { return 0; }
    virtual integer_type c_info(out_info_t &info) { return 0; }

   protected:
};

}  // namespace bopt
