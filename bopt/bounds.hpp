/**
 * @file bounds.h
 * @author Damian Abood (damian.abood@sydney.edu.au)
 * @brief Bound types used for conventional constrained optimisation
 * @version 0.1
 * @date 2024-05-09
 *
 *
 */
#pragma once

#include <Eigen/Core>
#include <limits>

#include "bopt/types.hpp"

namespace bopt {

enum class BoundType {
    /**
     * @brief Bounds of the form x = 0
     *
     */
    EQUALITY,
    /**
     * @brief Bounds of the form x >= 0
     *
     */
    POSITIVE,
    /**
     * @brief Bounds of the form x <= 0
     *
     */
    NEGATIVE,
    /**
     * @brief Bounds of the form x > 0
     *
     */
    STRICTLY_POSITIVE,
    /**
     * @brief Bounds of the form x < 0
     *
     */
    STRICTLY_NEGATIVE,
    /**
     * @brief Bounds of the form -inf < x < inf
     *
     */
    UNBOUNDED,
    /**
     * @brief Custom defined upper and lower bounds
     *
     */
    CUSTOM
};

void setBoundsFromType(const std::size_t &n, double* lb, double* ub, const BoundType &type);

}  // namespace bopt