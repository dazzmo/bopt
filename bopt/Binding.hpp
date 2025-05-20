#ifndef OPTIMISATION_BINDING_H
#define OPTIMISATION_BINDING_H

#include <memory>

#include "bopt/Common.hpp"
#include "bopt/Evaluator.hpp"
#include "bopt/Logging.hpp"
#include "bopt/Variable.hpp"

namespace bopt {

/**
 * @brief Class to bind an evaluator-based object to a set of input
 * variables as well as the data used to evaluate it.
 *
 * @tparam EvaluatorType
 */
template <typename EvaluatorType>
class Binding {
   public:
    using Evaluator = EvaluatorType;
    using EvaluatorPtr = std::shared_ptr<Evaluator>;

    /// @brief Evaluator data used to compute the main components of the
    /// function
    using Data = typename EvaluatorType::Data;
    using DataPtr = std::shared_ptr<Data>;

   public:
    Binding() : evaluator_(nullptr), data_(nullptr), index_manager_(nullptr) {}

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
        : evaluator_(ptr), data_(data), index_manager_(nullptr) {
        assert(ptr->numInputs() == indices.size());
        this->index_manager_ = std::make_shared<VariableIndexManager>(indices);
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
                  static_cast<DataPtr>(b.getData()),
                  b.getIndexManager().getIndices()) {}

    EvaluatorPtr get() const { return evaluator_; }

    /**
     * @brief Returns the index manager for the variables associated with the
     * binding.
     *
     * @return const VariableIndexManager&
     */
    const VariableIndexManager &getIndexManager() const {
        return *index_manager_;
    }

    /**
     * @brief The data class associated with the computing functions of the
     * bound evaluator.
     *
     * @return DataPtr
     */
    DataPtr &getData() { return data_; }
    const DataPtr &getData() const { return data_; }

   private:
    EvaluatorPtr evaluator_;
    DataPtr data_;
    std::shared_ptr<VariableIndexManager> index_manager_;
};

}  // namespace bopt

#endif /* OPTIMISATION_BINDING_H */
