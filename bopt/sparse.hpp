#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <boost/numeric/ublas/storage.hpp>
#include <vector>

#include "bopt/evaluator.hpp"
#include "bopt/logging.hpp"

namespace bopt {

namespace ublas = boost::numeric::ublas;

/**
 * @brief Inserts a block of data from an evaluator output into a matrix, using
 * specified row and column mappings.
 *
 * This function sets a block within a given matrix container by inserting data
 * derived from an `EvaluatorOutData` output. It supports both dense and sparse
 * block types, handling each accordingly. The insertion leverages custom row
 * and column indices for correct positioning within the larger matrix, and an
 * inserter function is used to perform the insertion of values.
 *
 * @tparam MatrixContainer Type of the matrix container, representing the larger
 * matrix structure.
 * @tparam EvaluatorOutInfo Type containing metadata about the evaluator output,
 * including dimensions and block type (dense or sparse).
 * @tparam EvaluatorOutData Type representing the data output from the
 * evaluator, containing the values to be inserted.
 * @tparam IndexVectorType Container type for row and column index mappings.
 * @tparam MatrixInserterFunction Type of the function or callable responsible
 * for inserting values into the matrix.
 *
 * @param matrix Reference to the matrix container where the block will be
 * inserted.
 * @param block_info Metadata about the block, including its dimensions (m x n)
 * and type (dense or sparse).
 * @param block_data Output data from the evaluator, providing the values to
 * insert into the block.
 * @param row_indices Vector of row indices mapping the block to specific rows
 * in the matrix container.
 * @param col_indices Vector of column indices mapping the block to specific
 * columns in the matrix container.
 * @param inserter Function or callable that inserts values into `matrix` at
 * specified row and column locations.
 *
 * @details
 * - For dense blocks, values are inserted in a column-major order.
 * - For sparse blocks, only non-zero values are inserted, as specified by the
 * sparse structure of `block_info`.
 * - `block_info` must include the block's size and type, which determines
 * whether the dense or sparse insertion logic is used.
 *
 * @pre `row_indices.size() == block_info.m` and `col_indices.size() ==
 * block_info.n`
 * @warning Ensure `row_indices` and `col_indices` correctly correspond to the
 * intended block location in the matrix.
 */
template <class MatrixContainer, class EvaluatorOutInfo, class EvaluatorOutData,
          typename IndexVectorType, class MatrixInserterFunction>
void set_block(MatrixContainer &matrix, const EvaluatorOutInfo &block_info,
               const EvaluatorOutData &block_data,
               const IndexVectorType &row_indices,
               const IndexVectorType &col_indices,
               const MatrixInserterFunction &inserter) {
    // Typedefs
    typedef MatrixContainer matrix_type;
    typedef typename EvaluatorOutInfo::index_type index_type;

    assert(row_indices.size() == block_info.m &&
           col_indices.size() == block_info.n &&
           "Indices provided are not same size as provided block");

    if (block_info.type == evaluator_matrix_type::type::Dense) {
        VLOG(10) << "Dense";
        // Populate matrix in a dense manner
        for (index_type col = 0; col < block_info.n; ++col) {
            for (index_type row = 0; row < block_info.m; ++row) {
                VLOG(10) << "Accessing (" << row_indices[row] << ", "
                         << col_indices[col] << ") to insert "
                         << block_data.values[col * block_info.m + row];
                // Assume column-major
                inserter(matrix, row_indices[row], col_indices[col],
                         block_data.values[col * block_info.m + row]);
            }
        }

    } else {
        VLOG(10) << "Sparse";

        // Access elements of the evaluator block
        index_type *colind = ccs_traits<EvaluatorOutInfo>::indptrs(block_info);
        index_type *indices = ccs_traits<EvaluatorOutInfo>::indices(block_info);

        for (index_type col = 0; col < block_info.n; ++col) {
            index_type start = colind[col];
            index_type end = colind[col + 1];

            for (index_type row = start; row < end; ++row) {
                // Get index of entry
                index_type idx_r = indices[row];
                index_type idx_c = col;
                VLOG(10) << "Accessing (" << row_indices[idx_r] << ", "
                         << col_indices[idx_c] << ") to insert "
                         << block_data.values[row];

                // Add entry to full Jacobian
                inserter(matrix, row_indices[idx_r], col_indices[idx_c],
                         block_data.values[idx_r]);
            }
        }
    }
}

}  // namespace bopt