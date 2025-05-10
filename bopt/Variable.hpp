#pragma once

#include <ostream>

#include "bopt/Logging.hpp"
#include "bopt/MathTypes.hpp"
#include "bopt/Types.hpp"

namespace bopt {

/**
 * @brief Class representation of a single variable.
 *
 */
class Variable {
   public:
    using Id = Size;

    enum class Type { Continuous, Discrete, Binary };

    Variable() : name_("") { id_ = getNextId(); }
    Variable(const String &name, const Type &type = Type::Continuous)
        : name_(name), id_(Id(0)), type_(type) {
        id_ = getNextId();
    }

    ~Variable() = default;

    const Id &getId() const { return id_; }

    const String &name() const { return name_; }

    bool operator<(const Variable &v) const { return getId() < v.getId(); }
    bool operator==(const Variable &v) const { return getId() == v.getId(); }

   private:
    Id id_;
    String name_;
    Type type_{Type::Continuous};

    Id getNextId() {
        static Id next_id = Id(0);
        return next_id++;
    }
};

std::ostream &operator<<(std::ostream &os, const Variable &v);

using VariableVector = typename MathTypes<Variable>::VectorX;
using VariableMatrix = typename MathTypes<Variable>::MatrixX;

/**
 * @brief Create a vector of variables, all with the same name and indexed with
 * their position in the vector.
 *
 * @param name Name of the variables within the vector.
 * @param sz Size of the vector to create.
 * @return variable_vector
 */
VariableVector createVariableVector(const String &name, const Size &sz);

/**
 * @brief Class which contains indices for variables. Also provides indication
 * if the variables are contained within a block, for efficient block methods to
 * expoit.
 *
 */
class VariableIndexManager {
   public:
    VariableIndexManager(const std::vector<Index> &indices)
        : is_sequential_(false), indices_(indices) {
        setIndices(indices);
    }

    void setIndices(const std::vector<Index> &indices) {
        indices_ = indices;
        for (Size i = 1; i < indices.size(); ++i) {
            if (indices[i] - indices[i - 1] != Index(1)) {
                is_sequential_ = false;
                return;
            }
        }
        is_sequential_ = true;
    }

    const std::vector<Index> &getIndices() const { return indices_; }
    /**
     * @brief Whether the provided indices are all sequential
     *
     * @return true
     * @return false
     */
    bool isSequential() const { return is_sequential_; }

   private:
    bool is_sequential_{false};
    std::vector<Index> indices_{};
};

std::ostream &operator<<(std::ostream &os, const VariableIndexManager &v);

}  // namespace bopt
