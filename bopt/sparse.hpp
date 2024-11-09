#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <boost/numeric/ublas/storage.hpp>
#include <vector>

#include "bopt/evaluator.hpp"
#include "bopt/logging.hpp"

namespace bopt {

namespace ublas = boost::numeric::ublas;

/**
 * @brief Sets a block within a matrix container using the values defined by an
 * Evaluator output. Row and column mappings must be provided to ensure correct
 * insertion. Matrix insertion is also required.
 *
 * @tparam MatrixContainer
 * @tparam Evaluator
 * @tparam IndexVectorType
 * @tparam MatrixInserterFunction
 * @param matrix
 * @param block_info
 * @param block_data
 * @param row_indices
 * @param col_indices
 * @param inserter
 */
template <class MatrixContainer, class Evaluator, typename IndexVectorType,
          class MatrixInserterFunction>
void setBlock(MatrixContainer &matrix,
              const evaluator_out_info<Evaluator> &block_info,
              const evaluator_out_data<Evaluator> &block_data,
              const IndexVectorType &row_indices,
              const IndexVectorType &col_indices,
              const MatrixInserterFunction &inserter) {
    // Typedefs
    typedef MatrixContainer matrix_type;
    typedef typename evaluator_out_info<Evaluator>::index_type index_type;

    assert(row_indices.size() == block_info.m &&
           col_indices.size() == block_info.n &&
           "Indices provided are not same size as provided block");

    if (block_info.type == evaluator_matrix_type::type::Dense) {
        // Populate matrix in a dense manner
        for (index_type col = 0; col < block_info.m; ++col) {
            for (index_type row = 0; row < block_info.n; ++row) {
                // Assume column-major
                inserter(matrix, row_indices[row], col_indices[col],
                         block_data.values[col * block_info.m + row]);
            }
        }

    } else {
        // Access elements of the evaluator block
        index_type *colind = (block_info.sparsity_out + 2);
        index_type *indices =
            (block_info.sparsity_out + (2 + block_info.m + 1));

        for (index_type col = 0; col < block_info.n; ++col) {
            index_type start = colind[col];
            index_type end = colind[col + 1];

            for (index_type row = start; row < end; ++row) {
                // Get index of entry
                index_type idx_r = indices[row];
                index_type idx_c = indices[col];

                // Add entry to full Jacobian
                inserter(matrix, row_indices[idx_r], col_indices[idx_c],
                         block_data.values[row]);
            }
        }
    }
}

}  // namespace bopt