#ifndef OPTIMISATION_BINDING_H
#define OPTIMISATION_BINDING_H

#include <memory>

#include "bopt/common.hpp"
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

template <typename binding>
struct binding_attributes {
    binding_traits<binding>::id_type id(binding &binding) const {
        return binding.id();
    }

    binding_traits<binding>::evaluator_type object(binding &binding) const {
        return *binding.get();
    }

    binding_traits<binding>::input_vector input_vector() const {
        return binding.input_vector();
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
            const std::vector<index_vector> &input_indices) {
        // Computes the indices within the map that the mapping relates to
        assert(evaluator_attributes<evaluator_type>::n_in(*obj) ==
                   input_indices.size() &&
               "Incorrect number of input index vectors for evaluator binding");

        for (index_type i = 0; i < obj->n_in; ++i) {
            input_indices.push_back(input_indices[i]);
        }
    }

    /**
     * @brief Cast a binding of type U to a binding of type T, if convertible.
     *
     * @tparam U
     * @param b
     */
    template <typename U>
    binding(const binding<U> &b,
            typename std::enable_if_t<std::is_convertible_v<
                typename binding<U>::evaluator_shared_ptr,
                typename binding<T>::evaluator_shared_ptr>> * = nullptr)
        : binding() {
        // Maintain the same binding id
        id = b.id;
        // Convert to new pointer type
        evaluator_ = static_cast<evaluator_shared_ptr>(b.evaluator_);
        // Copy vectors
        input_index = b.input_index;
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
