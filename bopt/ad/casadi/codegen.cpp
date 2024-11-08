#include "bopt/ad/casadi/codegen.hpp"

namespace bopt {
namespace casadi {

void codegen(const ::casadi::Function &f, const std::string &dir) {
    // Get current path
    auto path = std::filesystem::current_path();
    // Change to new path
    try {
        std::filesystem::current_path(dir);
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        // Return to existing path
        std::filesystem::current_path(path);
        return;
    }

    // Create hash
    std::size_t hash = std::hash<std::string>()(f.serialize());
    // Create new name
    std::string lib_name = "cg_" + f.name() + "_" + std::to_string(hash);

    if (!std::filesystem::exists(lib_name + ".so")) {
        // If binary doesn't exist, create it
        f.generate(lib_name + ".c");
        int ret = system(("gcc -fPIC -shared -O3 -march=native " + lib_name +
                          ".c -o " + lib_name + ".so")
                             .c_str());
        if (ret) {
            assert("Could not compile code!");
        }
    }

    // Return back to normal path
    std::filesystem::current_path(path);
}

}  // namespace casadi
}  // namespace bopt
