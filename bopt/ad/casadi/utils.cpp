#include "bopt/ad/casadi/utils.hpp"

#include <filesystem>

namespace bopt {
namespace casadi {

function_t codegen(const function_t &f) {
    // Create a codegen file folder
    std::filesystem::create_directories(BOPT_CASADI_CODEGEN_DIRECTORY);

    // Get current path
    auto path = std::filesystem::current_path();
    // Change to new path
    try {
        std::filesystem::current_path(BOPT_CASADI_CODEGEN_DIRECTORY);
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        // Return to existing path
        std::filesystem::current_path(path);
        return function_t();
    }

    // Create hash
    std::size_t hash = std::hash<std::string>()(f.serialize());
    // Create new name
    std::string name = f.name() + "_" + std::to_string(hash);

    if (!std::filesystem::exists(name + ".so")) {
        // If binary doesn't exist, create it
        f.generate(name + ".c");
        int ret = system(("gcc -fPIC -shared -O3 -march=native " + name +
                          ".c -o " + name + ".so")
                             .c_str());
        if (ret) {
            assert("Could not compile code!");
        }
    }

    // Provide dynamic library handler for the provided shared library
    function_t ret = ::casadi::external(f.name(), name + ".so");

    // Return back to normal path
    std::filesystem::current_path(path);

    return ret;
}

void set_eigen_sparsity(Eigen::SparseMatrix<double> &out,
                        const ::casadi::Sparsity &sparsity) {
    // Use casadi information
    std::vector<casadi_int> output_row, output_col;
    sparsity.get_triplet(output_row, output_col);

    std::vector<Eigen::Triplet<double>> triplets;
    triplets.resize(sparsity.nnz());

    for (int k = 0; k < sparsity.nnz(); ++k)
        triplets[k] = Eigen::Triplet<double>(output_row[k], output_col[k]);

    out.resize(sparsity.rows(), sparsity.columns());
    out.setFromTriplets(triplets.begin(), triplets.end());
    out.makeCompressed();
}

void set_eigen_sparsity(Eigen::SparseVector<double> &out,
                        const ::casadi::Sparsity &sparsity) {
    // todo - make sure that sparsity pattern is a column vector
    // Use casadi information
    std::vector<casadi_int> output_row, output_col;
    sparsity.get_triplet(output_row, output_col);

    std::vector<Eigen::Triplet<double>> triplets;
    triplets.resize(sparsity.nnz());

    out.reserve(sparsity.nnz());

    // Loop over all non-zeros
    if (sparsity.rows() >= sparsity.columns()) {
        // Column vector
        for (int k = 0; k < sparsity.nnz(); ++k) out.insertBack(output_row[k]);
    } else {
        // Row vector
        for (int k = 0; k < sparsity.nnz(); ++k) out.insertBack(output_col[k]);
    }
}

function_t create_function(const std::string &name,
                           const std::vector<sym_t> &in,
                           const std::vector<sym_t> &out, bool densify,
                           bool codegen) {
    // Create vector of temporary outputs
    std::vector<sym_t> out_ = {};
    if (densify) {
        // If outputs are requested to be dense, make them dense
        for (const sym_t &out_i : out) {
            out_.push_back(sym_t::densify(out_i));
        }
    } else {
        out_ = out;
    }

    // Create function
    function_t f = function_t(name, in, out_);

    // If function is to be code-generated, do so.
    if (codegen) {
        return bopt::casadi::codegen(f);
    }

    return f;
}

}  // namespace casadi

}  // namespace bopt
