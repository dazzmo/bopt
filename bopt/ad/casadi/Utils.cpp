#include "bopt/ad/casadi/Utils.hpp"

#include <filesystem>

namespace bopt {
namespace casadi {

Function codegen(const Function &f) {
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
        return Function();
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
    Function function = ::casadi::external(f.name(), name + ".so");

    // Return back to normal path
    std::filesystem::current_path(path);

    return function;
}


void setupSparseEigenMatrix(Eigen::SparseMatrix<Real> &out,
                            const ::casadi::Sparsity &sparsity) {
    // Use casadi information
    std::vector<casadi_int> output_row, output_col;
    sparsity.get_triplet(output_row, output_col);

    std::vector<Eigen::Triplet<Real>> triplets;
    triplets.resize(sparsity.nnz());

    for (int k = 0; k < sparsity.nnz(); ++k)
        triplets[k] = Eigen::Triplet<Real>(output_row[k], output_col[k]);

    out.resize(sparsity.rows(), sparsity.columns());
    out.setFromTriplets(triplets.begin(), triplets.end());
    out.makeCompressed();
}

void setupSparseEigenMatrix(Eigen::SparseVector<Real> &out,
                            const ::casadi::Sparsity &sparsity) {
    // todo - make sure that sparsity pattern is a column vector
    // Use casadi information
    std::vector<casadi_int> output_row, output_col;
    sparsity.get_triplet(output_row, output_col);

    std::vector<Eigen::Triplet<Real>> triplets;
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

Function create_function(const std::string &name,
                         const std::vector<SymbolicVector> &in,
                         const std::vector<SymbolicVector> &out, bool densify,
                         bool codegen) {
    // Create vector of temporary outputs
    std::vector<SymbolicVector> out_ = {};
    if (densify) {
        // If outputs are requested to be dense, make them dense
        for (const SymbolicVector &out_i : out) {
            out_.push_back(SymbolicVector::densify(out_i));
        }
    } else {
        out_ = out;
    }

    // Create function
    Function f = Function(name, in, out_);

    // If function is to be code-generated, do so.
    if (codegen) {
        return bopt::casadi::codegen(f);
    }

    return f;
}

}  // namespace casadi

}  // namespace bopt
