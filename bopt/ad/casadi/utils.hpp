#pragma once

#include <Eigen/Sparse>
#include <casadi/casadi.hpp>

namespace bopt {
namespace casadi {

#ifndef BOPT_CASADI_CODEGEN_DIRECTORY
#define BOPT_CASADI_CODEGEN_DIRECTORY "./cg"
#endif

typedef ::casadi::SX sym_t;
typedef ::casadi::SX sym_vector_t;
typedef ::casadi::Function function_t;

/**
 * @brief Generates a dynamically linkable library for the function f and loads
 * the binary into code. Returns a function which uses the library.
 *
 * @param f Function to perform code generation for
 * @return ::casadi::Function
 *
 */
function_t codegen(const function_t &f);

void set_eigen_sparsity(Eigen::SparseMatrix<double> &out,
                        const ::casadi::Sparsity &sparsity);

void set_eigen_sparsity(Eigen::SparseVector<double> &out,
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
 * @return function_t
 */
function_t create_function(const std::string &name,
                           const std::vector<sym_t> &in,
                           const std::vector<sym_t> &out, bool densify = false,
                           bool codegen = false);

}  // namespace casadi
}  // namespace bopt
