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
    typedef typename T::input_vector input_vector;
};

template <typename T>
struct binding_attributes {
    binding_traits<T>::id_type id(T &binding) const { return binding.id(); }

    binding_traits<T>::type object() const { return *T.get(); }

    binding_traits<T>::input_vector input_vector() const {
        return T.input_vector();
    }
};

template <typename Binding, typename IndexVectorType>
struct binding_index_map {
    typedef typename binding_traits<Binding>::type type;
    typedef typename binding_traits<Binding>::input_vector input_vector;

    template <typename VariableIndexMap>
    binding_index_map(const &Binding binding,
                      const VariableIndexMap &index_map) {
        // Locate vector locations for each input
        for (input_vector::iterator it =
                 binding_attributes<type>::input_vector(binding).begin();
             it != binding_attributes<type>::input_vector().end(); ++it) {
            input_index.push_back(index_map.find(it));
        }
    }

    IndexVectorType input_index;
};

/**
 * @brief Class to bind an evaluator-based object to a sequence of input
 * variables
 *
 * @tparam T
 */
template <class T>
class Binding {
   public:
    typedef typename T type;
    typedef std::size_t id_type;
    typedef typename std::shared_ptr<type> type_shared_ptr;
    typedef std::vector<variable_t *> input_vector;
    typedef std::vector<std::vector<std::size_t>> input_index_vector;

    /**
     * @brief Bind an evaluator object to a set of input variables, with
     * indexing dictated by a VariableIndexMap
     *
     * @param obj
     * @param in
     * @param index_map
     */
    Binding(const type_shared_ptr &obj, const std::vector<Variable> &in,
            const VariableIndexMap &index_map) {
        // Computes the indices within the map that the mapping relates to
        input_index = {};
        for (int i = 0; i < obj->n_in; ++i) {
            input_indices.push_back({});
            for (const Variable &vi : in) {
                input_index.push_back(
                    index_map.index_vector[index_map.index(vi)]);
            }
        }
    }

    /**
     * @brief Cast a binding of type U to a Binding of type T, if convertible.
     *
     * @tparam U
     * @param b
     */
    template <typename U>
    Binding(const Binding<U> &b,
            typename std::enable_if_t<
                std::is_convertible_v<typename Binding<U>::type_shared_ptr,
                                      typename Binding<T>::type_shared_ptr>> * =
                nullptr)
        : Binding() {
        // Maintain the same binding id
        id = b.id;
        // Convert to new pointer type
        obj_ = static_cast<type_shared_ptr>(b.obj_);
        // Copy vectors
        input_index = b.input_index;
    }

    id_type id;

    std::shared_ptr<T> get() const { return obj_; }

    input_index_vector input_index;

   private:
    std::shared_ptr<T> obj_;
};

}  // namespace bopt

#endif /* OPTIMISATION_BINDING_H */
