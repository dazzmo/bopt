#pragma once

#include <vector>

#include "bopt/common.hpp"

namespace bopt {

struct evaluator_matrix_type {
    enum type { Dense, Sparse };
};

template <typename Evaluator>
struct evaluator_traits {
    typedef typename Evaluator::value_type value_type;
    typedef typename Evaluator::index_type index_type;
    typedef typename Evaluator::integer_type integer_type;
};

template <typename Evaluator>
struct evaluator_attributes {
    typedef typename Evaluator::index_type index_type;

    static constexpr index_type &n_in(const Evaluator &evaluator) {
        return evaluator.n_in();
    }
};

template <typename EvaluatorInfo>
struct evaluator_out_info_traits {
    typedef typename EvaluatorInfo::value_type value_type;
    typedef typename EvaluatorInfo::index_type index_type;
    typedef typename EvaluatorInfo::integer_type integer_type;
};

/**
 * @brief Evaluator output information, including output dimensions, sparsity
 * pattern, density and number of non-zero elements.
 *
 * @tparam Evaluator
 */
template <typename Evaluator>
struct evaluator_out_info {
    typedef typename evaluator_traits<Evaluator>::index_type index_type;
    typedef typename evaluator_traits<Evaluator>::value_type value_type;
    typedef typename evaluator_traits<Evaluator>::integer_type integer_type;

    typedef evaluator_matrix_type::type matrix_type;

    // Whether the output is dense or sparse
    matrix_type type = matrix_type::Dense;

    // Number of rows in output matrix
    index_type m = 0;
    // Number of columns in output matrix
    index_type n = 0;
    // Number of non-zero entries in output matrix
    index_type nnz = 0;
    // Sparsity pattern in CCS format (with size and nnz elements included)
    index_type *sparsity_out = nullptr;
    // Values
    value_type *values = nullptr;
};

/**
 * @brief Column Compressed Storage representation
 *
 * @tparam EvaluatorInfo
 */
template <typename EvaluatorInfo>
struct ccs_traits {
    typedef typename evaluator_out_info_traits<EvaluatorInfo>::index_type
        index_type;

    static constexpr index_type *indices(const EvaluatorInfo &info) {
        return (info.sparsity_out + 2 + info.n + 1);
    }

    static constexpr index_type *indptrs(const EvaluatorInfo &info) {
        return (info.sparsity_out + 2);
    }
};

template <typename Evaluator>
struct evaluator_out_data {
    typedef typename evaluator_traits<Evaluator>::value_type value_type;
    evaluator_out_data(const evaluator_out_info<Evaluator> &info) {
        values.assign(info.nnz, value_type(0));
    }
    std::vector<value_type> values;
};

template <typename Evaluator>
struct evaluator_out_data_ptr {
    typedef typename evaluator_traits<Evaluator>::value_type value_type;
    value_type *values = nullptr;
};

/**
 * @brief Evaluator class that computes an output given an input.
 *
 */
template <typename ValueType, typename IntegerType = int,
          typename IndexType = std::size_t,
          template <class> class VectorType = std::vector>
class evaluator {
   public:
    typedef ValueType value_type;
    typedef IndexType index_type;
    typedef IntegerType integer_type;
    typedef evaluator_out_info<evaluator> out_info_t;
    typedef evaluator_out_data<evaluator> out_data_t;

   public:
    virtual integer_type operator()(const value_type **arg,
                                    value_type *ret) = 0;

    virtual integer_type info(out_info_t &info) { return 0; }

    // Output data buffer
    VectorType<value_type> buffer;

    /**
     * @brief Number of inputs for the evaluator.
     *
     * @return const index_type& Number of inputs required by the evaluator.
     */
    const index_type &n_in() const { return n_in_; }

   protected:
    // Number of input vectors for the function
    index_type n_in_;
    // Number of rows in output matrix
    index_type out_n;
    // Number of columns in output matrix
    index_type out_m;
    // Number of non-zero entries in output matrix
    index_type out_nnz;

    VectorType<index_type> sparsity_out;
};

}  // namespace bopt
