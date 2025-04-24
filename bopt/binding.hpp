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
 * variables as well as the data used to evaluate it.
 *
 * @tparam T
 */
template <typename EvaluatorType>
class Binding {
   public:
    using Evaluator = EvaluatorType;
    /// @brief Evaluator data used to compute the main components of the
    /// function
    using Data = typename EvaluatorType::Data;

    using EvaluatorPtr = std::shared_ptr<Evaluator>;
    using DataPtr = std::shared_ptr<Data>;

   public:
    Binding() : evaluator_(nullptr), data_(nullptr), indices_(nullptr) {}

    ~Binding() = default;

    /**
     * @brief Bind an evaluator object to a set of input variables, with
     * indexing dictated by a VariableIndexMap
     *
     * @param ptr
     * @param indices Indices of the variables bound to the evaluator
     */
    Binding(const std::shared_ptr<Evaluator> &ptr, const DataPtr &data,
            const std::vector<Eigen::Index> &indices)
        : evaluator_(ptr), data_(data), indices_(nullptr) {
        BOPT_ASSERT(ptr->getInputDimension() == indices.size());
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
                typename Binding<Other>::EvaluatorPtr,
                typename Binding<Evaluator>::EvaluatorPtr>> * = nullptr,
            typename std::enable_if_t<
                std::is_convertible_v<typename Binding<Other>::DataPtr,
                                      typename Binding<Evaluator>::DataPtr>> * =
                nullptr)
        : Binding(static_cast<EvaluatorPtr>(b.get()),
                  static_cast<DataPtr>(b.data()), b.indices().indices()) {}

    EvaluatorPtr get() const {
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
     * @return DataPtr
     */
    DataPtr &data() { return data_; }
    const DataPtr &data() const { return data_; }

   private:
    EvaluatorPtr evaluator_;
    DataPtr data_;
    // todo - see about memory management here
    std::shared_ptr<variable_indices> indices_;
};

}  // namespace bopt

#endif /* OPTIMISATION_BINDING_H */
