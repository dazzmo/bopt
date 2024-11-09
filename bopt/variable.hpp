#ifndef SYMBOLIC_VARIABLE_H
#define SYMBOLIC_VARIABLE_H

#include <boost/iterator/permutation_iterator.hpp>
#include <boost/property_map/property_map.hpp>
#include <ostream>

#include "bopt/bounds.hpp"
#include "bopt/logging.hpp"
#include "bopt/types.hpp"

namespace bopt {

template <typename T>
struct variable_property_traits {
    typedef typename T::id_type id_type;
    typedef typename T::name_type name_type;
};

struct variable_type {
    enum type { Continuous, Discrete };
};

template <typename T>
struct variable_attributes {
    typedef typename variable_type::type type;

    const type &type(const T &variable) const { return variable.type(); }
};

/**
 * @brief Class representation of a single variable.
 *
 */
class Variable {
   public:
    typedef std::size_t id_type;
    typedef std::string name_type;
    typedef variable_type::type variable_type;

    Variable() = default;

    Variable(const name_type &name) : name_(name) {
        static int next_id_ = id_type(0);
        id_ = next_id_++;
    }

    ~Variable() = default;

    const id_type &id() const { return id_; }
    const name_type &name() const { return name_; }

    bool operator<(const Variable &v) const { return id() < v.id(); }
    bool operator==(const Variable &v) const { return id() == v.id(); }

   private:
    id_type id_;
    name_type name_;
    variable_type type_ = variable_type::Continuous;
};

typedef typename variable_property_traits<Variable>::id_type variable_t;
typedef std::vector<Variable> variable_vector_t;

class VariableIndexMap {
   public:
    Index size() const { return index_.size(); }

    typedef std::vector<Variable> variable_vector_t;

    bool contains(const Variable &v) const {
        return variable_vector_idx.find(v.id()) != variable_vector_idx.end();
    }

    bool add(const Variable &var) {
        if (contains(var)) {
            LOG(ERROR) << "Variable " << var.name()
                       << " is already added to program!";
            return false;
        }
        // Add to index map
        variable_vector_idx.insert({var.id(), index_.size()});
        index_.push_back(index_.size());
        return true;
    }

    bool add(const variable_vector_t &v) {
        // Append to our map
        for (variable_vector_t::iterator it = v.begin(); it != v.end(); ++it)
            if (add(*it) == false) return false;
        return true;
    }

    // Mapping of variables to the index vector
    std::unordered_map<variable_id_type, index_type> variable_vector_idx;
    // Index of each variable within the optimisation vector
    std::std::vector<index_type> vector_idx;
};

// Operator overloading
std::ostream &operator<<(std::ostream &os, Variable var);

}  // namespace bopt

#endif /* SYMBOLIC_VARIABLE_H */
