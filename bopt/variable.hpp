#ifndef SYMBOLIC_VARIABLE_H
#define SYMBOLIC_VARIABLE_H

#include <ostream>

#include "bopt/bounds.hpp"
#include "bopt/logging.hpp"
#include "bopt/types.hpp"

namespace bopt {

template <typename T>
struct variable_traits {
    typedef typename T::id_type id_type;
    typedef typename T::name_type name_type;
    typedef typename T::type type;
};

struct variable_type {
    enum type { Continuous, Discrete };
};

template <typename T>
struct variable_attributes {
    typedef typename variable_traits<T>::variable_type type_t;

    const type_t &type(const T &variable) const { return variable.type(); }
};

/**
 * @brief Class representation of a single variable.
 *
 */
class variable {
   public:
    typedef std::size_t id_type;
    typedef std::string name_type;
    typedef variable_type::type type;

    variable() = default;

    variable(const name_type &name) : name_(name) {
        static int next_id_ = id_type(0);
        id_ = next_id_++;
    }

    ~variable() = default;

    const id_type &id() const { return id_; }
    const name_type &name() const { return name_; }

    bool operator<(const variable &v) const { return id() < v.id(); }
    bool operator==(const variable &v) const { return id() == v.id(); }

   private:
    id_type id_;
    name_type name_;
    type type_ = type::Continuous;
};

// Operator overloading
std::ostream &operator<<(std::ostream &os, variable var);

}  // namespace bopt

#endif /* SYMBOLIC_VARIABLE_H */
