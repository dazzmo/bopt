#pragma once

#include <ostream>


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
    enum type { Continuous, Discrete, Binary };
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

    variable() : name_("") { id_ = next_id(); }
    variable(const name_type &name) : name_(name) { id_ = next_id(); }

    ~variable() = default;

    const id_type &id() const { return id_; }

    const name_type &name() const { return name_; }

    bool operator<(const variable &v) const { return id() < v.id(); }
    bool operator==(const variable &v) const { return id() == v.id(); }

   private:
    id_type id_;
    name_type name_;
    type type_ = type::Continuous;

    id_type next_id() {
        static int next_id_ = id_type(0);
        return next_id_++;
    }
};

typedef Eigen::VectorX<variable> variable_vector;

/**
 * @brief Create a vector of variables, all with the same name and indexed with
 * their position in the vector.
 *
 * @param name Name of the variables within the vector.
 * @param sz Size of the vector to create.
 * @return variable_vector
 */
variable_vector create_variable_vector(const std::string &name,
                                       const Eigen::Index &sz);

// Operator overloading
std::ostream &operator<<(std::ostream &os, const variable &var);

/**
 * @brief Class which contains indices for variables. Also provides indication
 * if the variables are contained within a block, for efficient block methods to
 * expoit.
 *
 * @tparam IndexType
 */
template <typename IndexType>
class variable_indices_tpl {
   public:
    variable_indices_tpl(const std::vector<IndexType> &indices)
        : is_block_(false), indices_(indices) {
        set_indices(indices);
    }

    void set_indices(const std::vector<IndexType> &indices) {
        indices_ = indices;
        for (std::size_t i = 1; i < indices.size(); ++i) {
            if (indices[i] - indices[i - 1] != IndexType(1)) {
                is_block_ = false;
                return;
            }
        }
        is_block_ = true;
    }

    const std::vector<IndexType> &indices() const { return indices_; }
    bool is_block() const { return is_block_; }

   private:
    bool is_block_;
    std::vector<IndexType> indices_;
};

typedef variable_indices_tpl<Eigen::Index> variable_indices;

std::ostream &operator<<(std::ostream &os, const variable_indices &var);

}  // namespace bopt
