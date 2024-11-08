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

/**
 * @brief Index map for a set of variables contained within a vector.
 *
 */
class VariableIndexMap {
   public:
    typedef variable_property_traits<Variable>::id_type id_type;
    typedef std::size_t index_type;

    using IndexMapType = std::unordered_map<id_type, Index>;

    Index size() const { return index_.size(); }

    bool add(const Variable &var) {
        if (contains(var)) {
            LOG(ERROR) << "Variable " << var.name()
                       << " is already added to program!";
            return false;
        }
        // Add to index map
        variable_index_map_.insert({var.id(), index_.size()});
        index_.push_back(index_.size());
        return true;
    }

    bool add(const std::vector<Variable> &var) {
        // Append to our map
        for (std::size_t i = 0; i < var.size(); ++i) {
            if (add(var[i]) == false) return false;
        }
        return true;
    }

    // todo - void remove();

    bool contains(const Variable &v) const {
        return variable_index_map_.find(v.id()) != variable_index_map_.end();
    }

    /**
     * @brief Returns the index of the variable for the associated index
     * mapping.
     *
     * @param v
     * @return const Index&
     */
    const Index &index(const Variable &v) const {
        auto it = variable_index_map_.find(v.id());
        assert(it != variable_index_map_.end() &&
               "Variable does not exist in index map!");
        return index_[it->second];
    }

    std::vector<Index> indices(const std::vector<Variable> &v) const {
        std::vector<Index> indices;
        indices.reserve(v.size());
        for (std::size_t i = 0; i < v.size(); ++i) {
            indices.push_back(index(v[i]));
        }
        // Return vector of indices
        return indices;
    }

   private:
    // Location of each vertex in the index map
    std::unordered_map<id_type, index_type> variable_index_map_;
    // Index in the current vector
    std::vector<index_type> index_;
};

// Operator overloading
std::ostream &operator<<(std::ostream &os, Variable var);

}  // namespace bopt

#endif /* SYMBOLIC_VARIABLE_H */
