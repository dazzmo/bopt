#pragma once

#include <Eigen/Core>
#include <limits>

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
    typename T value_type;
    bound_element(const value_type &lower, const value_type &upper)
        : upper(upper), lower(lower) {}

    bound_element(const bound_type::type &type) : lower(0.0), upper(0.0) {
        set(type);
    }

    value_type lower;
    value_type upper;

    void set(const bound_type::type &type) {
        constexpr value_type inf = std::numeric_limits<value_type>::infinity();
        constexpr value_type eps = std::numeric_limits<value_type>::epsilon();
        switch (type) {
            case bound_type::Equality: {
                set(lower, 0.0);
                set(upper, 0.0);
                break;
            }

            case bound_type::Positive: {
                set(lower, 0.0);
                set(upper, inf);
                break;
            }

            case bound_type::Negative: {
                set(lower, -inf);
                set(upper, inf);
                break;
            }

            case bound_type::StrictlyPositive: {
                set(upper, inf);
                set(lower, eps);
                break;
            }

            case bound_type::StrictlyNegative: {
                set(upper, -eps);
                set(lower, -inf);
                break;
            }

            case bound_type::Unbounded: {
                set(upper, inf);
                set(lower, -inf);
                break;
            }

            default: {
                set(upper, inf);
                set(lower, -inf);
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

    VectorType<bound_element<value_type>> m_values;

    bound_element &operator[](const index_type &idx) { return m_values[idx]; }

    vector_bounds(const index_type &n,
                  const bound_type::type &type = bound_type::Equality)
        : m_values(VectorType<bound_element<value_type>>(
              n, bound_element<value_type>(type))) {}

    void set(const bound_type::type &type) {
        for (bound_element &b : m_values) {
            b.set(type);
        }
    }
};

template <typename T>
const inline bool is_satisfied(
    const std::vector<T> &values, const vector_bounds<T> &b,
    const typename bound_traits<T>::value_type &eps =
        std::numeric_limits<typename bound_traits<T>::value_type>::epsilon()) {
    for (bound_traits<T>::index_type i = 0; i < values.size(); ++i) {
        if (values[i] - b[i].lower < -eps || values[i] - b[i].upper > eps)
            return false;
    }
    return true;
}

}  // namespace bopt