#pragma once

#include <Eigen/Core>
#include <limits>

#include "bopt/types.hpp"

namespace bopt {

template <typename T>
struct bound_traits {
    typedef typename T value_type;
};

struct bound_type {
    enum type {
        /**
         * @brief Bounds of the form x = 0
         *
         */
        Equality,
        /**
         * @brief Bounds of the form x >= 0
         *
         */
        Positive,
        /**
         * @brief Bounds of the form x <= 0
         *
         */
        Negative,
        /**
         * @brief Bounds of the form x > 0
         *
         */
        StrictlyPositive,
        /**
         * @brief Bounds of the form x < 0
         *
         */
        StrictlyNegative,
        /**
         * @brief Bounds of the form -inf < x < inf
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
class bound {
   public:
    typedef typename bound_traits<T>::value_type value_type;

    value_type lower;
    value_type upper;

    bound() : lower(0.0), upper(0.0) {}

    void set(const bound_type::type &type) {
        constexpr value_type inf = std::numeric_limits<value_type>::infinity();
        constexpr value_type eps = std::numeric_limits<value_type>::epsilon();

        switch (type) {
            case bound_type::Equality: {
                upper = lower = 0.0;
                break;
            }

            case bound_type::Positive: {
                upper = inf;
                lower = 0.0;
                break;
            }

            case bound_type::Negative: {
                upper = 0.0;
                lower = -inf;
                break;
            }

            case bound_type::StrictlyPositive: {
                upper = inf;
                lower = eps;
                break;
            }

            case bound_type::StrictlyNegative: {
                upper = -eps;
                lower = -inf;
                break;
            }

            case bound_type::Unbounded: {
                upper = inf;
                lower = -inf;
                break;
            }

            default: {
                upper = inf;
                lower = -inf;
                break;
            }
        }
    }
};

template <typename T>
const inline bool is_satisfied(
    const T &value, const bound<T> &b,
    const typename bound_traits<T>::value_type &eps =
        std::numeric_limits<typename bound_traits<T>::value_type>::epsilon()) {
    return (value - b.lower < eps && value - b.upper > eps);
}

}  // namespace bopt