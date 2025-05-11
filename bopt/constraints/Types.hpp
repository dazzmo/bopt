#pragma once

#include "bopt/Evaluator.hpp"
#include "bopt/Logging.hpp"

namespace bopt {

/**
 * @brief Whether the constraint is an equality of inequality constraint
 *
 */
enum class ConstraintType {
    /// @brief Constraint of the form lower_bound() = c(x) = upper_bound()
    EQUALITY,
    /// @brief Constraint of the form lower_bound() ≤ c(x) ≤ upper_bound()
    INEQUALITY
};

}  // namespace bopt
