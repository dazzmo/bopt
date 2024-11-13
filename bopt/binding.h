#ifndef OPTIMISATION_BINDING_H
#define OPTIMISATION_BINDING_H

#include <memory>

#include "bopt/common.hpp"
#include "bopt/evaluator.hpp"
#include "bopt/logging.hpp"
#include "bopt/variable.hpp"

namespace bopt {

template <typename T>
struct binding_traits {
    typedef typename T::type type;
    typedef typename T::id_type id_type;
    typedef typename T::index_vector index_vector;
    typedef typename T::evaluator_type evaluator_type;
    typedef typename T::evaluator_unique_ptr evaluator_unique_ptr;
    typedef typename T::evaluator_shared_ptr evaluator_shared_ptr;
};

template <typename Binding>
struct binding_attributes {
    typename binding_traits<Binding>::id_type id(Binding &binding) const {
        return binding.id();
    }

    constexpr typename binding_traits<Binding>::evaluator_type object(
        Binding &binding) const {
        return *binding.get();
    }
};

/**
 * @brief Class to bind an evaluator-based object to a sequence of input
 * variables
 *
 * @tparam T
 */
template <class Evaluator, class I = std::size_t>
class binding {
   public:
    typedef typename evaluator_traits<Evaluator>::value_type value_type;
    typedef typename evaluator_traits<Evaluator>::index_type index_type;
    typedef I id_type;

    typedef Evaluator evaluator_type;

    typedef typename std::unique_ptr<evaluator_type> evaluator_unique_ptr;
    typedef typename std::shared_ptr<evaluator_type> evaluator_shared_ptr;

    typedef std::vector<I> index_vector;

    /**
     * @brief Bind an evaluator object to a set of input variables, with
     * indexing dictated by a VariableIndexMap
     *
     * @param obj
     * @param in
     * @param index_map
     */
    binding(const evaluator_shared_ptr &obj,
            const std::vector<index_vector> &input_indices)
        : input_indices({}), evaluator_(obj) {
        // Computes the indices within the map that the mapping relates to
        //   todo - fix this
        // assert(obj->n_in == input_indices.size() &&
        //        "Incorrect number of input index vectors for evaluator
        //        binding");
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

    std::vector<index_vector> input_indices;

   private:
    evaluator_shared_ptr evaluator_;
};

}  // namespace bopt

#endif /* OPTIMISATION_BINDING_H */
