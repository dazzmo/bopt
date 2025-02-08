#pragma once

#include <ostream>

#include "bopt/logging.hpp"
#include "bopt/types.hpp"

namespace bopt {

/**
 * @brief Class representation of a single variable.
 *
 */
class Variable {
   public:
    using Id = std::size_t;

    enum class Type { Continuous, Discrete, Binary };

    Variable() : name_("") { id_ = next_id(); }
    Variable(const std::string &name) : name_(name) { id_ = next_id(); }

    ~Variable() = default;

    const Id &id() const { return id_; }

    const std::string &name() const { return name_; }

    bool operator<(const Variable &v) const { return id() < v.id(); }
    bool operator==(const Variable &v) const { return id() == v.id(); }

   private:
    Id id_;
    std::string name_;
    Type type_ = Type::Continuous;

    Id next_id() {
        static int next_id_ = Id(0);
        return next_id_++;
    }
};

typedef VectorX<Variable> VariableVector;

/**
 * @brief Create a vector of variables, all with the same name and indexed with
 * their position in the vector.
 *
 * @param name Name of the variables within the vector.
 * @param sz Size of the vector to create.
 * @return variable_vector
 */
VariableVector createVariableVector(const std::string &name,
                                    const Eigen::Index &sz);

// Operator overloading
std::ostream &operator<<(std::ostream &os, const Variable &var);

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
