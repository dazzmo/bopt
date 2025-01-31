#include "bopt/common.hpp"

namespace bopt {

bool checkVector(const Eigen::Ref<const Eigen::VectorXd> &x) {
    return x.allFinite() && !x.hasNaN() && x.size();
}

bool checkMatrix(const Eigen::Ref<const Eigen::MatrixXd> &M) {
    return M.allFinite() && !M.hasNaN() && M.size();
}

}  // namespace bopt
