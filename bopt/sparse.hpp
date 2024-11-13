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
        LOG(INFO) << "Dense";
        // Populate matrix in a dense manner
        for (index_type col = 0; col < block_info.n; ++col) {
            for (index_type row = 0; row < block_info.m; ++row) {
                LOG(INFO) << "Accessing (" << row_indices[row] << ", "
                          << col_indices[col] << ")";
                // Assume column-major
                inserter(matrix, row_indices[row], col_indices[col],
                         block_data.values[col * block_info.m + row]);
            }
        }

    } else {
        LOG(INFO) << "Sparse";

        // Access elements of the evaluator block
        index_type *colind = ccs_traits<EvaluatorOutInfo>::indices(block_info);
        index_type *indices = ccs_traits<EvaluatorOutInfo>::indptrs(block_info);

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