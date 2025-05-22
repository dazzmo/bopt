#pragma once

#include <Eigen/Sparse>
#include <casadi/casadi.hpp>

#include "bopt/Types.hpp"

namespace bopt {
namespace casadi {

#ifndef BOPT_CASADI_CODEGEN_DIRECTORY
#define BOPT_CASADI_CODEGEN_DIRECTORY "./cg"
#endif

typedef ::casadi::SX Symbol;
typedef ::casadi::SX SymbolicVector;
typedef ::casadi::SX SymbolicMatrix;
typedef ::casadi::Function Function;

/**
 * @brief Generates a dynamically linkable library for the function f and loads
 * the binary into code. Returns a function which uses the library.
 *
 * @param f Function to perform code generation for
 * @return ::casadi::Function
 *
 */
Function codegen(const Function &f);

/**
 * @brief Setup an Eigen::SparseMatrix object with the correct sparsity pattern
 * specified by the casadi::Sparsity object
 *
 * @param out
 * @param sparsity
 */
void setupSparseEigenMatrix(Eigen::SparseMatrix<Real> &out,
                            const ::casadi::Sparsity &sparsity);

/**
 * @brief Setup an Eigen::SparseVector object with the correct sparsity pattern
 * specified by the casadi::Sparsity object
 *
 * @param out
 * @param sparsity
 */
void setupSparseEigenMatrix(Eigen::SparseVector<Real> &out,
                            const ::casadi::Sparsity &sparsity);

/**
 * @brief Create a casadi::Function with the provided inputs and outputs.
 * Optionally make all outputs dense with densify = true and allow the
 * function to use compiled binaries with codegen = true.
 *
 * @param name
 * @param in
 * @param out
 * @param densify
 * @param codegen
 * @return Function
 */
Function create_function(const std::string &name,
                         const std::vector<SymbolicVector> &in,
                         const std::vector<SymbolicVector> &out,
                         bool densify = false, bool codegen = false);

}  // namespace casadi
}  // namespace bopt
