#ifndef OPTIMISATION_BINDING_H
#define OPTIMISATION_BINDING_H

#include <memory>

#include "bopt/common.hpp"
#include "bopt/evaluator.hpp"
#include "bopt/logging.hpp"
#include "bopt/variable.hpp"

namespace bopt {

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

    evaluator_shared_ptr get() const { return evaluator_; }

    variable_indices<Eigen::Index> input_indices;

   private:
    evaluator_shared_ptr evaluator_;
};

}  // namespace bopt

#endif /* OPTIMISATION_BINDING_H */
