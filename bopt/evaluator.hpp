#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>

#include "bopt/common.hpp"
#include "bopt/logging.hpp"
#include "bopt/types.hpp"

namespace bopt {

struct evaluator {
    enum class return_status {
        Success = 0,      // Successful evaluation
        NotImplemented,   // Evaluator not implemented
        InvalidArgument,  // An invalid argument was provided
        InvalidOutput,    // An invalid output was provided
        Failure           // Unknown failure
    };
};

template <typename ValueType>
struct evaluator_traits {
    using value_t = ValueType;

    using dense_vector_t = Eigen::VectorX<value_t>;
    using sparse_vector_t = Eigen::SparseVector<value_t>;

    using dense_matrix_t = Eigen::MatrixX<value_t>;
    using sparse_matrix_t = Eigen::SparseMatrix<value_t>;
};

/**
 * @brief Evaluator class that computes an output given an input.
 *
 */
template <typename ValueType>
class evaluator_tpl {
   public:
    using value_t = typename evaluator_traits<ValueType>::value_t;

    using dense_vector_t = typename evaluator_traits<ValueType>::dense_vector_t;
    using sparse_vector_t =
        typename evaluator_traits<ValueType>::sparse_vector_t;

    using dense_matrix_t = typename evaluator_traits<ValueType>::dense_matrix_t;
    using sparse_matrix_t =
        typename evaluator_traits<ValueType>::sparse_matrix_t;

   public:
    evaluator_tpl() : sz_in_(0), sz_out_(0) {
        LOG(INFO) << "evaluator_tpl default constructor";
    }
    evaluator_tpl(const bopt_index &sz_in, const bopt_index &sz_out)
        : sz_in_(sz_in), sz_out_(sz_out) {}

    virtual ~evaluator_tpl() = default;

    const bopt_index &sz_in() const { return sz_in_; }
    const bopt_index &sz_out() const { return sz_out_; }

    inline bool check_input(const dense_vector_t &x) const {
        return !x.hasNaN() || x.allFinite();
    }

   protected:
    // Input size
    bopt_index sz_in_;
    // Output size
    bopt_index sz_out_;

    static_assert(std::is_arithmetic<ValueType>::value,
                  "ValueType for evaluator_tpl "
                  "must be a numerical type.");
};

}  // namespace bopt
