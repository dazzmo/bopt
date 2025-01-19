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
 * @brief Evaluator class that computes an output given an input. Also provides
 * the option for a parameterised evaluator of the form \f$ y = f_p(x) \f$
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
    evaluator_tpl()
        : sz_in_(0), sz_out_(0), sz_p_(0), p_(dense_vector_t::Zero(0)) {}

    evaluator_tpl(const bopt_index &sz_in, const bopt_index &sz_out,
                  const bopt_index &sz_p = 0)
        : sz_in_(sz_in),
          sz_out_(sz_out),
          sz_p_(sz_p),
          p_(dense_vector_t::Zero(sz_p)) {}

    virtual ~evaluator_tpl() = default;

    const bopt_index &sz_in() const { return sz_in_; }
    const bopt_index &sz_p() const { return sz_p_; }
    const bopt_index &sz_out() const { return sz_out_; }

    /**
     * @brief Parameter vector for an evaluator \f$ y = f_p(x) \f$ (of size
     * sz_p() x 1)
     *
     * @return const dense_vector_t&
     */
    const dense_vector_t &parameters() const { return p_; }

    void set_parameters(const Eigen::Ref<dense_vector_t> &p) {
        DBGASSERT(p.size() == sz_p());
        p_ = p;
    }

    inline bool check_vector(const dense_vector_t &x) const {
        return !x.hasNaN() || x.allFinite();
    }

   protected:
    // Input size
    bopt_index sz_in_;
    // Output size
    bopt_index sz_out_;
    // Parameter size
    bopt_index sz_p_;

    dense_vector_t p_;

    static_assert(std::is_arithmetic<ValueType>::value,
                  "ValueType for evaluator_tpl "
                  "must be a numerical type.");
};

template <typename DenseType, typename SparseType>
struct dense_sparse_buffer_tpl {
    DenseType dense;
    SparseType sparse;
};

}  // namespace bopt
