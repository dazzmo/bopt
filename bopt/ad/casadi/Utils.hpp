#pragma once

#include <Eigen/Sparse>
#include <casadi/casadi.hpp>

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

void setupSparseEigenMatrix(Eigen::SparseMatrix<double> &out,
                        const ::casadi::Sparsity &sparsity);

void setupSparseEigenMatrix(Eigen::SparseVector<double> &out,
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
