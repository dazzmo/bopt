#include "bopt/bounds.hpp"
namespace bopt {

void setBoundsFromType(const std::size_t &n, double *lb, double *ub,
                       const BoundType &type) {
    constexpr double inf = std::numeric_limits<double>::infinity();
    constexpr double eps = std::numeric_limits<double>::epsilon();

    for (std::size_t i = 0; i < n; ++i) {
        switch (type) {
            case BoundType::EQUALITY: {
                ub[i] = lb[i] = 0.0;
                break;
            }

            case BoundType::POSITIVE: {
                ub[i] = inf;
                lb[i] = 0.0;
                break;
            }

            case BoundType::NEGATIVE: {
                ub[i] = 0.0;
                lb[i] = -inf;
                break;
            }

            case BoundType::STRICTLY_POSITIVE: {
                ub[i] = inf;
                lb[i] = eps;
                break;
            }

            case BoundType::STRICTLY_NEGATIVE: {
                ub[i] = -eps;
                lb[i] = -inf;
                break;
            }

            case BoundType::UNBOUNDED: {
                ub[i] = inf;
                lb[i] = -inf;
                break;
            }

            default: {
                ub[i] = inf;
                lb[i] = -inf;
                break;
            }
        }
    }
}

}  // namespace bopt
