#ifndef UTILS_CODEGEN_H
#define UTILS_CODEGEN_H

#include <dlfcn.h>

#include <casadi/casadi.hpp>
#include <cassert>
#include <filesystem>

#include "bopt/dlib_handler.hpp"

namespace bopt {
namespace casadi {

/**
 * @brief Generates a dynamically linkable library for the function f and loads
 * the binary into code. Returns a function which uses the library.
 *
 * @param f Function to perform code generation for
 * @param dir Directory to store the binary
 *
 */
std::shared_ptr<bopt::dynamic_library_handler> codegen(
    const ::casadi::Function &f);

}  // namespace casadi
}  // namespace bopt

#endif /* UTILS_CODEGEN_H */
