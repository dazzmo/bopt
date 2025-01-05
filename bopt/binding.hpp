#ifndef OPTIMISATION_BINDING_H
#define OPTIMISATION_BINDING_H

#include <memory>

#include "bopt/common.hpp"
#include "bopt/evaluator.hpp"
#include "bopt/logging.hpp"
#include "bopt/variable.hpp"

namespace bopt {

template <typename IndexType>
struct variable_indices {
    variable_indices(const std::vector<IndexType> &indices) {
        IndexType pre = indices[0];
        for (const auto &i : indices) {
            if (pre - i != IndexType(1)) {
                is_block = false;
            }
            pre = i;
        }
        is_block = true;
    }

    bool is_block;
    std::vector<IndexType> indices;
};

/**
 * @brief Class to bind an evaluator-based object to a sequence of input
 * variables
 *
 * @tparam T
 */
template <class EvaluatorType>
class binding {
   public:
    typedef typename std::shared_ptr<EvaluatorType> evaluator_shared_ptr;

    /**
     * @brief Bind an evaluator object to a set of input variables, with
     * indexing dictated by a VariableIndexMap
     *
     * @param ptr
     * @param in
     * @param index_map
     */
    binding(const std::shared_ptr<EvaluatorType> &ptr,
            const std::vector<index_vector> &input_indices)
        : input_indices({}), evaluator_(ptr) {
        // Computes the indices within the map that the mapping relates to
        // assert(evaluator_attributes<evaluator_t>::n_in(*obj) ==
        // input_indices.size() &&
        //    "Incorrect number of input index vectors for evaluator
        //    binding");
        this->input_indices = input_indices;
    }

    /**
     * @brief Cast a binding of type Other to a binding of type Evaluator, if
     * convertible.
     *
     * @tparam Other
     * @param b
     */
    template <typename Other>
    binding(const binding<Other> &b,
            typename std::enable_if_t<std::is_convertible_v<
                typename binding<Other>::evaluator_shared_ptr,
                typename binding<Evaluator>::evaluator_shared_ptr>> * = nullptr)
        : binding(static_cast<evaluator_shared_ptr>(b.get()), b.input_indices) {
        // Maintain the same binding id
        id = b.id;
    }

    /**
     * @brief ID of the binding
     *
     */
    id_type id;

    evaluator_shared_ptr get() const { return evaluator_; }

    variable_indices<Eigen::Index> input_indices;

   private:
    evaluator_shared_ptr evaluator_;
};

/**
 * @brief Creates a vector with the values specified from the indices provided.
 *
 * @tparam ValueType The types of values.
 * @tparam IndexType The type of indices.
 * @param values Value vector to take the values from.
 * @param indices Vector of indices from values to insert into the new vector.
 * @return std::vector<ValueType> Vector of values from values specified by
 * indices.
 */
template <class ValueType, class IndexType>
std::vector<ValueType> create_indexed_view(
    const std::vector<ValueType> &values,
    const std::vector<IndexType> &indices) {
    typedef typename std::vector<IndexType>::const_iterator index_iterator;

    std::vector<ValueType> res;
    res.reserve(indices.size());

    for (index_iterator it = indices.begin(); it != indices.end(); ++it) {
        res.push_back(values[*it]);
    }

    return res;
}

}  // namespace bopt

#endif /* OPTIMISATION_BINDING_H */
