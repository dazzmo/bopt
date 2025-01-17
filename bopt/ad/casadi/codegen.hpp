#ifdef BOPT_WITH_CASADI

#pragma once

#include <casadi/casadi.hpp>
#include <cassert>
#include <filesystem>

#include "bopt/logging.hpp"

#ifndef BOPT_CASADI_CODEGEN_DIRECTORY
#define BOPT_CASADI_CODEGEN_DIRECTORY "./cg"
#endif

namespace bopt {
namespace casadi {

/**
 * @brief Generates a dynamically linkable library for the function f and loads
 * the binary into code. Returns a function which uses the library.
 *
 * @param f Function to perform code generation for
 * @return ::casadi::Function
 *
 */
::casadi::Function codegen(const ::casadi::Function &f);

}  // namespace casadi
}  // namespace bopt

#endif