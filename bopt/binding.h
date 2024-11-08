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
    typedef std::vector<std::size_t> input_index_vector;

    Binding(const type_shared_ptr &obj, const std::vector<Variable> &in,
            const VariableIndexMap &index_map) {
        // Computes the indices within the map that the mapping relates to
        input_index = {};
        for (const Variable &vi : in) {
            input_index.push_back(index_map.index_vector[index_map.index(vi)]);
        }
    }

    Binding(const type_shared_ptr &obj, const input_vector &in)
        : obj_(obj), in(in) {}

    /**
     * @brief Cast a binding of type U to a Binding of type T, if convertible.
     *
     * @tparam U
     * @param b
     */
    template <typename U>
    Binding(const Binding<U> &b,
            typename std::enable_if_t<std::is_convertible_v<
                std::shared_ptr<U>, std::shared_ptr<T>>> * = nullptr)
        : Binding() {
        // Maintain the same binding id
        id = b.id;
        // Convert to new pointer type
        obj_ = static_cast<std::shared_ptr<T>>(b.obj_);
        // Copy vectors
        in = b.in;
    }

    id_type id;

    std::shared_ptr<T> get() const { return obj_; }

    input_index_vector input_index;

   private:
    std::shared_ptr<T> obj_;
};

}  // namespace bopt

#endif /* OPTIMISATION_BINDING_H */
