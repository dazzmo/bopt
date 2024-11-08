#pragma once

#include <vector>

#include "bopt/common.hpp"

namespace bopt {

template <typename Evaluator>
struct evaluator_traits {
    typedef typename Evaluator::value_type value_type;
    typedef typename Evaluator::index_type index_type;
    typedef typename Evaluator::vector_type vector_type;
    typedef typename Evaluator::vector_iterator vector_iterator;
};

template <typename Evaluator>
struct SparsityInformation {
    typedef typename evaluator_traits<Evaluator>::index_type I;
    typedef typename evaluator_traits<Evaluator>::value_type D;
    typedef typename evaluator_traits<Evaluator>::vector_type vector_type;

    // Number of input vectors for the function
    I n;
    // Number of rows in output matrix
    I m;
    // Number of columns in output matrix
    I nnz;
    // Number of non-zero entries in output matrix
    vector_type<D> val;
    // Indices for the column or row
    vector_type<I> indices;
    // Pointers for each indexed column or row
    vector_type<I> indptr;
};

template <typename Evaluator>
struct evaluator_out_info {
    typedef typename evaluator_traits<Evaluator>::index_type index_type;

    // Number of input vectors for the function
    index_type n_in;
    // Number of rows in output matrix
    index_type out_n;
    // Number of columns in output matrix
    index_type out_m;
    // Number of non-zero entries in output matrix
    index_type out_nnz;

    index_type *indices;
    index_type *indptrs;
};

template <typename Evaluator>
struct evaluator_out_ref {
    typedef typename evaluator_traits<Evaluator>::index_type index_type;

    // Number of input vectors for the function
    index_type n_in;
    // Number of rows in output matrix
    index_type out_n;
    // Number of columns in output matrix
    index_type out_m;
    // Number of non-zero entries in output matrix
    index_type out_nnz;

    vector_type<index_type> sparsity_out;

    // Output data buffer
    vector_type<value_type> buffer;
};

/**
 * @brief Evaluator class that computes an output given an input.
 *
 */
template <typename T, typename I = std::size_t>
class EvaluatorBase {
   public:
    typedef typename T value_type;
    typedef typename I index_type;

    template <typename T>
    using vector_type = std::vector<T>;

    template <typename T>
    using vector_iterator = std::vector<T>::iterator;

   public:
    virtual index_type operator()(const value_type **arg) = 0;

    virtual index_type info(evaluator_info<EvaluatorBase> &info) { return 0; }

   protected:
    // Number of input vectors for the function
    index_type n_in;
    // Number of rows in output matrix
    index_type out_n;
    // Number of columns in output matrix
    index_type out_m;
    // Number of non-zero entries in output matrix
    index_type out_nnz;

    vector_type<index_type> sparsity_out;

    // Output data buffer
    vector_type<value_type> buffer;
};

}  // namespace bopt
