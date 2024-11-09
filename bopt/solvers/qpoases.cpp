#include "bopt/solvers/qpoases.h"

#include "bopt/sparse.hpp"

namespace bopt {
namespace solvers {

qpOASESSparseSolverInstance::qpOASESSparseSolverInstance(
    MathematicalProgram& program)
    : SolverBase(program) {
    LOG(INFO) << "qpOASESSparseSolverInstance::qpOASESSparseSolverInstance";
    // Create problem
    int nx = program.numberOfDecisionVariables();
    int ng = program.numberOfConstraints();

    qp_dense_ = std::make_unique<qpOASES::SQProblem>(nx, ng);

    data_ = QPOASESProgramData(nx, ng);

    // Create matrix data
    // ! If it's a sparse program

    matrix_data_.A = getJacobianSparsityPattern(
        program.getLinearConstraints(), program.getDecisionVariableIndexMap());

    // Wrap data
    qpoases_matrix_data_.A = qpOASES::SparseMatrix(
        matrix_data_.A.n, matrix_data_.A.m, matrix_data_.A.indices,
        matrix_data_.A.indptr, matrix_data_.A.values);

    // Create variable bounds from bounding box constraints

    // getCurrentProgram().getBoundingBoxConstraints();

    // Compute locations of each
}

qpOASESSparseSolverInstance::~qpOASESSparseSolverInstance() = default;

void qpOASESSparseSolverInstance::reset() { info_.number_of_solves = 0; }

void qpOASESSparseSolverInstance::solve() {
    /** Linear costs **/
    LOG(INFO) << "linear costs";
    for (const Binding<LinearCost>& binding :
         getCurrentProgram().getLinearCosts()) {
        auto x_indices =
            getCurrentProgram().getDecisionVariableIndices(binding.in[0]);
        auto p_indices = getCurrentProgram().getParameterIndices(binding.in[1]);

        std::vector<double> pi;
        for (const auto& i : p_indices) {
            // Create vector of input
            pi.emplace_back(getCurrentProgram().p()[i]);
        }

        // Evaluate coefficients for the cost a^T x + b
        std::vector<double> a(binding.get()->a_nnz);
        double b;
        binding.get()->a(std::vector<const double*>({pi.data()}).data(),
                         {a.data()});
        binding.get()->b(std::vector<const double*>({pi.data()}).data(), {&b});

        // Insert at locations
        for (int i = 0; i < binding.get()->grd_nnz; ++i) {
            // Get index in the data for the gradient non-zeros
            // todo - see about speeding this up later
            // todo - need correct index for x and a
            matrix_data_.g.get_value(x_indices[i], 0) += a[i];
        }
    }

    // Lambda function
    auto inserter_plus = [](ublas::mapped_matrix<double>& m, std::size_t i,
                            std::size_t j, const double& v) { m(i, j) += v };
    // auto inserter_plus = []() {m(i, j) = v};

    /** Quadratic costs **/
    for (const Binding<QuadraticCost>& binding :
         getCurrentProgram().getQuadraticCosts()) {
        const auto& x = binding.input_index[0];
        const auto& p = binding.input_index[1];

        std::vector<double> pi;
        for (const auto& i : p) {
            // Create vector of input
            pi.emplace_back(getCurrentProgram().p()[i]);
        }

        // Evaluate coefficients for the cost a^T x + b
        evaluator_out_info<QuadraticCost> A_info, b_info;
        binding.get()->A_info(A);
        binding.get()->b_info(b);

        evaluator_out_data<QuadraticCost> A, b;
        binding.get()->A(
            evaluator_input_generator<QuadraticCost>({pi.data()}).data(),
            {A.values});
        binding.get()->b(
            evaluator_input_generator<QuadraticCost>({pi.data()}).data(),
            {b.values});

        setBlock(matrix_data_.H, A, x, x, inserter_plus);
        setBlock(matrix_data_.g, b, x, {0}, inserter_plus);
    }

    /** Linear constraints **/
    LOG(INFO) << "linear constraints";
    std::size_t cnt = 0;
    for (const Binding<LinearConstraint>& binding :
         getCurrentProgram().getLinearConstraints()) {
        const auto& x = binding.input_index;

        auto p_indices = getCurrentProgram().getParameterIndices(binding.in[1]);

        std::vector<double> pi;
        for (const auto& i : p_indices) {
            // Create vector of input
            pi.emplace_back(getCurrentProgram().p()[i]);
        }

        // Evaluate coefficients for the cost a^T x + b
        evaluator_out_info<LinearConstraint> A, b;
        binding.get()->A_info(A);
        binding.get()->b_info(b);

        std::vector<double> A(A_nnz), b(b_nnz);
        binding.get()->A(std::vector<const double*>({pi.data()}).data(),
                         {A.data()});
        binding.get()->b(std::vector<const double*>({pi.data()}).data(),
                         {b.data()});

        // Add to Jacobian

        // data_.ubA.middleRows(cnt, m) = binding.get()->ub - b;
        // data_.lbA.middleRows(cnt, m) = binding.get()->lb - b;

        // Increase constraint index
        cnt += A_n;
    }

    int nWSR = options_.nWSR;

    // Map created matrices to

    // Solve
    if (info_.number_of_solves > 0 && options_.perform_hotstart) {
        // Use previous solution to hot-start the program
        qp_->hotstart(H.data(), g_data.data(), A.data(), data_.lbx.data(),
                      data_.ubx.data(), data_.lbA.data(), data_.ubA.data(),
                      nWSR);
    } else {
        // Initialise the program and solve it
        qp_->init();
    }

    // Collect information
    info_.nWSR = nWSR;
    info_.status = qp_->getStatus();
    info_.iterations = 100;  // !

    info_.number_of_solves++;

    // Get results
    if (info_.status == qpOASES::QProblemStatus::QPS_SOLVED) {
        info_.success = true;
        qp_->getPrimalSolution(results_.x.data());
    }
};

}  // namespace solvers
}  // namespace bopt