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
template <typename EvaluatorType>
class Binding {
    using Evaluator = EvaluatorType;
    using Data = typename EvaluatorType::Data;

   public:
    typedef typename std::shared_ptr<EvaluatorType> evaluator_shared_ptr;

    Binding() : evaluator_(nullptr), indices_(nullptr) {}

    ~Binding() = default;

    /**
     * @brief Bind an evaluator object to a set of input variables, with
     * indexing dictated by a VariableIndexMap
     *
     * @param ptr
     * @param indices Indices of the variables bound to the evaluator
     */
    Binding(const std::shared_ptr<Evaluator> &ptr,
            const std::shared_ptr<Data> &data,
            const std::vector<Eigen::Index> &indices)
        : evaluator_(ptr), data_(data), indices_(nullptr) {
        BOPT_ASSERT(ptr->dim_input() == indices.size());
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
    Binding(const Binding<Other> &b,
            typename std::enable_if_t<std::is_convertible_v<
                typename Binding<Other>::evaluator_shared_ptr,
                typename Binding<Evaluator>::evaluator_shared_ptr>> * = nullptr)
        : Binding(static_cast<evaluator_shared_ptr>(b.get()),
                  b.indices().indices()) {}

    evaluator_shared_ptr get() const {
        // DBGASSERT(evaluator_ && "Empty binding has no object bound to it");
        return evaluator_;
    }

    const variable_indices &indices() const {
        // DBGASSERT(indices_ && "Empty binding has no indices");
        return *indices_;
    }

    /**
     * @brief The data class associated with the computing functions of the
     * bound evaluator.
     *
     * @return std::shared_ptr<Data>
     */
    std::shared_ptr<Data> &data() { return data_; }

   private:
    std::shared_ptr<Data> data_;

    evaluator_shared_ptr evaluator_;
    // todo - see about memory management here
    std::shared_ptr<variable_indices> indices_;
};

}  // namespace bopt

#endif /* OPTIMISATION_BINDING_H */
