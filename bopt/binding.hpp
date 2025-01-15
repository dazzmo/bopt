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

    binding() : evaluator_(nullptr), indices_(nullptr) {}

    ~binding() = default;

    /**
     * @brief Bind an evaluator object to a set of input variables, with
     * indexing dictated by a VariableIndexMap
     *
     * @param ptr
     * @param indices Indices of the variables bound to the evaluator
     */
    binding(const std::shared_ptr<EvaluatorType> &ptr,
            const std::vector<Eigen::Index> &indices)
        : evaluator_(ptr), indices_(nullptr) {
        DBGASSERT(ptr->sz_in() == indices.size());
        this->indices_ = std::make_shared<variable_indices>(indices);
    }

    /**
     * @brief Cast a binding of type Other to a binding of type Evaluator, if
     * convertible.
     *
     * @tparam Other
     * @param b
     */
    template <typename Other>
    binding(
        const binding<Other> &b,
        typename std::enable_if_t<std::is_convertible_v<
            typename binding<Other>::evaluator_shared_ptr,
            typename binding<EvaluatorType>::evaluator_shared_ptr>> * = nullptr)
        : binding(static_cast<evaluator_shared_ptr>(b.get()),
                  b.indices().indices()) {}

    evaluator_shared_ptr get() const {
        DBGASSERT(evaluator_ && "Empty binding has no object bound to it");
        return evaluator_;
    }

    const variable_indices &indices() const {
        DBGASSERT(indices_ && "Empty binding has no indices");
        return *indices_;
    }

   private:
    evaluator_shared_ptr evaluator_;
    // todo - see about memory management here
    std::shared_ptr<variable_indices> indices_;
};

}  // namespace bopt

#endif /* OPTIMISATION_BINDING_H */
