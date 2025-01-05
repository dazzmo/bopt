#pragma once

#include <Eigen/Core>
#include <limits>

#include "bopt/common.hpp"
#include "bopt/types.hpp"

namespace bopt {
namespace bounds {

enum class type {
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

template <typename ValueType>
void set_bound_limits(const type &type,
                      Eigen::Ref<Eigen::VectorX<ValueType>> lower_bound,
                      Eigen::Ref<Eigen::VectorX<ValueType>> upper_bound) {
    constexpr ValueType inf = std::numeric_limits<ValueType>::infinity();
    constexpr ValueType eps = std::numeric_limits<ValueType>::epsilon();

    switch (type) {
        case bounds::type::Equality: {
            lower_bound.setConstant(0.0);
            upper_bound.setConstant(0.0);
            break;
        }

        case bounds::type::Positive: {
            lower_bound.setConstant(0.0);
            upper_bound.setConstant(inf);
            break;
        }

        case bounds::type::Negative: {
            lower_bound.setConstant(-inf);
            upper_bound.setConstant(inf);
            break;
        }

        case bounds::type::StrictlyPositive: {
            upper_bound.setConstant(inf);
            lower_bound.setConstant(eps);
            break;
        }

        case bounds::type::StrictlyNegative: {
            upper_bound.setConstant(-eps);
            lower_bound.setConstant(-inf);
            break;
        }

        case bounds::type::Unbounded: {
            upper_bound.setConstant(inf);
            lower_bound.setConstant(-inf);
            break;
        }

        default: {
            upper_bound.setConstant(inf);
            lower_bound.setConstant(-inf);
            break;
        }
    }
}

template <typename ValueType>
inline bool is_satisfied(
    const Eigen::VectorX<ValueType> &values,
    const Eigen::VectorX<ValueType> &lower,
    const Eigen::VectorX<ValueType> &upper,
    const ValueType &eps = std::numeric_limits<ValueType>::epsilon()) {
    for (bopt_index i = 0; i < values.size(); ++i) {
        if (values[i] - lower[i] < -eps || values[i] - upper[i] > eps)
            return false;
    }
    return true;
}

}  // namespace bounds
}  // namespace bopt