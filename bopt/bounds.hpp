#pragma once

#include <Eigen/Core>
#include <limits>

#include "bopt/common.hpp"
#include "bopt/types.hpp"

namespace bopt {

template <typename T>
struct bound_traits {
    typedef T value_type;
};

template <typename T>
struct bound_attributes {};

struct bound_type {
    enum type {
        /**
         * @brief bounds of the form x = 0
         *
         */
        Equality,
        /**
         * @brief bounds of the form x >= 0
         *
         */
        Positive,
        /**
         * @brief bounds of the form x <= 0
         *
         */
        Negative,
        /**
         * @brief bounds of the form x > 0
         *
         */
        StrictlyPositive,
        /**
         * @brief bounds of the form x < 0
         *
         */
        StrictlyNegative,
        /**
         * @brief bounds of the form -inf < x < inf
         *
         */
        Unbounded,
        /**
         * @brief Custom defined upper and lower bounds
         *
         */
        Custom
    };
};

template <typename T>
struct bound_element {
    typedef T value_type;

    bound_element() : lower(value_type(0)), upper(value_type(0)) {}

    bound_element(const value_type &lower, const value_type &upper)
        : upper(upper), lower(lower) {}

    bound_element(const bound_type::type &type)
        : lower(value_type(0)), upper(value_type(0)) {
        set(type);
    }

    value_type lower;
    value_type upper;

    void set(const value_type &lb, const value_type &ub) {
        bopt::set(lower, lb);
        bopt::set(upper, ub);
    }

    void set(const bound_type::type &type) {
        constexpr value_type inf = std::numeric_limits<value_type>::infinity();
        constexpr value_type eps = std::numeric_limits<value_type>::epsilon();
        switch (type) {
            case bound_type::Equality: {
                bopt::set(lower, 0.0);
                bopt::set(upper, 0.0);
                break;
            }

            case bound_type::Positive: {
                bopt::set(lower, 0.0);
                bopt::set(upper, inf);
                break;
            }

            case bound_type::Negative: {
                bopt::set(lower, -inf);
                bopt::set(upper, inf);
                break;
            }

            case bound_type::StrictlyPositive: {
                bopt::set(upper, inf);
                bopt::set(lower, eps);
                break;
            }

            case bound_type::StrictlyNegative: {
                bopt::set(upper, -eps);
                bopt::set(lower, -inf);
                break;
            }

            case bound_type::Unbounded: {
                bopt::set(upper, inf);
                bopt::set(lower, -inf);
                break;
            }

            default: {
                bopt::set(upper, inf);
                bopt::set(lower, -inf);
                break;
            }
        }
    }
};

template <typename T, typename I = std::size_t,
          template <class> class VectorType = std::vector>
class vector_bounds {
   public:
    typedef typename bound_traits<T>::value_type value_type;
    typedef I index_type;
    typedef bound_element<value_type> bound_element_type;

    VectorType<bound_element_type> m_values;

    bound_element_type &operator[](const index_type &idx) {
        return m_values[idx];
    }

    vector_bounds() = default;

    vector_bounds(const index_type &n,
                  const bound_type::type &type = bound_type::Equality)
        : m_values(
              VectorType<bound_element_type>(n, bound_element_type(type))) {}

    void set(const bound_type::type &type) {
        for (bound_element_type &b : m_values) {
            b.set(type);
        }
    }
};

template <typename T>
inline bool is_satisfied(
    const std::vector<T> &values, const vector_bounds<T> &b,
    const typename bound_traits<T>::value_type &eps =
        std::numeric_limits<typename bound_traits<T>::value_type>::epsilon()) {
    for (typename bound_traits<T>::index_type i = 0; i < values.size(); ++i) {
        if (values[i] - b[i].lower < -eps || values[i] - b[i].upper > eps)
            return false;
    }
    return true;
}

}  // namespace bopt