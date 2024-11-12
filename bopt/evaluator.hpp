#pragma once

#include <vector>

#include "bopt/common.hpp"

namespace bopt {

template <typename Evaluator>
struct evaluator_traits {
    typedef typename Evaluator::value_type value_type;
    typedef typename Evaluator::index_type index_type;
};

template <typename Evaluator>
struct evaluator_attributes {
    typedef typename Evaluator::index_type index_type;

    index_type n_in(const Evaluator &evaluator) const {
        return evaluator.n_in();
    }
};

// template <typename Evaluator>
// struct evaluator_input_generator {
//     typedef typename Evaluator::value_type value_type;

//     std::vector<const value_type *> operator()(
//         std::array<value_type *> &input) {
//         return std::vector<const value_type *>(input);
//     }
// };

struct evaluator_matrix_type {
    enum type { Dense, Sparse };
};

template <typename Evaluator>
struct evaluator_out_info {
    typedef typename evaluator_traits<Evaluator>::index_type index_type;
    typedef typename evaluator_traits<Evaluator>::value_type value_type;
    typedef evaluator_matrix_type::type matrix_type;

    // Whether the output is dense or sparse
    matrix_type type = matrix_type::Dense;

    // Number of rows in output matrix
    index_type m;
    // Number of columns in output matrix
    index_type n;
    // Number of non-zero entries in output matrix
    index_type nnz;
    // Sparsity pattern in CCS format (with size and nnz elements included)
    index_type *sparsity_out;
    // Values
    value_type *values;
};

template <typename Evaluator>
struct evaluator_out_data {
    typedef typename evaluator_traits<Evaluator>::value_type value_type;
    value_type *values;
};

/**
 * @brief Evaluator class that computes an output given an input.
 *
 */
template <typename T, typename I = std::size_t,
          template <class> class VectorType = std::vector>
class evaluator {
   public:
    typedef T value_type;
    typedef I index_type;

   public:
    virtual index_type operator()(const value_type **arg, value_type *ret) = 0;

    virtual index_type info(evaluator_out_info<evaluator> &info) {
        return 0;
    }

    // Output data buffer
    VectorType<value_type> buffer;

   protected:
    // Number of input vectors for the function
    index_type n_in;
    // Number of rows in output matrix
    index_type out_n;
    // Number of columns in output matrix
    index_type out_m;
    // Number of non-zero entries in output matrix
    index_type out_nnz;

    VectorType<index_type> sparsity_out;
};

}  // namespace bopt
