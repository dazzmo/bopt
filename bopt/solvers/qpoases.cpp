#include "bopt/solvers/qpoases.h"

#include "bopt/sparse.hpp"

namespace bopt {
namespace solvers {

qpoases_solver_instance::qpoases_solver_instance(mathematical_program& program)
    : SolverBase(program) {
    LOG(INFO) << "qpoases_solver_instance::qpoases_solver_instance";
    // Create problem
    int nx = program.numberOfDecisionVariables();
    int ng = program.numberOfConstraints();

    qp_ = std::make_unique<qpOASES::SQProblem>(nx, ng);
}

qpoases_solver_instance::~qpoases_solver_instance() = default;

void qpoases_solver_instance::solve() {
    // Matrix inserter functions
    auto inserter_add_to = [](ublas::matrix<double>& m, std::size_t i,
                              std::size_t j, const double& v) { m(i, j) += v };

    auto inserter_set_to = [](ublas::matrix<double>& m, std::size_t i,
                              std::size_t j, const double& v) { m(i, j) = v };

    /** Linear costs **/
    LOG(INFO) << "linear costs";
    for (const Binding<LinearCost<double>>& binding :
         program().getLinearCosts()) {
        auto x_indices = binding.input_index[0];
        auto p_indices = binding.input_index[1];

        // todo - shorten this
        std::vector<double> pi;
        for (const auto& i : p_indices) {
            // Create vector of input
            pi.emplace_back(program().p()[i]);
        }

        evaluator_out_info<LinearCost<double>> a_info;
        evaluator_out_data<LinearCost<double>> a_data;
        binding.get()->a_info(a_info);

        // Evaluate coefficients for the cost a^T x + b
        binding.get()->a(std::vector<const double*>({pi.data()}).data(),
                         {a_data.values});

        setBlock(data.g, a_info, a_data, x_indices, {0}, inserter_add_to);
    }

    /** Quadratic costs **/
    for (const Binding<QuadraticCost>& binding :
         program().getQuadraticCosts()) {
        auto x_indices = binding.input_index[0];
        auto p_indices = binding.input_index[1];

        std::vector<double> pi;
        for (const auto& i : p_indices) {
            // Create vector of input
            pi.emplace_back(program().p()[i]);
        }

        // Evaluate coefficients for the cost a^T x + b
        evaluator_out_info<QuadraticCost> A_info, b_info;
        binding.get()->A_info(A_info);
        binding.get()->b_info(b_info);

        evaluator_out_data<QuadraticCost> A, b;
        binding.get()->A(
            evaluator_input_generator<QuadraticCost>({pi.data()}).data(),
            {A.values});
        binding.get()->b(
            evaluator_input_generator<QuadraticCost>({pi.data()}).data(),
            {b.values});

        setBlock(data.H, a_info, A_data, x_indices, inserter_add_to);
        setBlock(data.g, b_info, b_data, x_indices, {0}, inserter_add_to);
    }

    /** Linear constraints **/
    LOG(INFO) << "linear constraints";
    std::size_t cnt = 0;
    for (const Binding<LinearConstraint>& binding :
         program().getLinearConstraints()) {
        auto x_indices = binding.input_index[0];
        auto p_indices = binding.input_index[1];

        std::vector<double> pi;
        for (const auto& i : p_indices) {
            // Create vector of input
            pi.emplace_back(program().p()[i]);
        }

        // Evaluate coefficients for the cost a^T x + b
        evaluator_out_info<LinearConstraint> A_info, b_info;
        evaluator_out_data<LinearConstraint> A_data, b_data;
        binding.get()->A_info(A_info);
        binding.get()->b_info(b_info);

        binding.get()->A(std::vector<const double*>({pi.data()}).data(),
                         {A_data.values});
        binding.get()->b(std::vector<const double*>({pi.data()}).data(),
                         {b_data.values});

        // Create row indices
        std::vector<evaluator_traits<LinearConstraint>::index_type> row_indices;
        for (index_type i = 0; i < A_info.n; ++i) {
            row_indices.push_back(cnt + i);
        };

        setBlock(data.A, A_info, A_data, row_indices, x_indices,
                 inserter_set_to);

        // Add to each bound
        std::vector<double> lbA, ubA;

        for (index_type i = 0; i < b_info.n; ++i) {
            ubA[row_indices[i]] = binding.get()->bounds[i].upper - b.data[i];
            lbA[row_indices[i]] = binding.get()->bounds[i].lower - b.data[i];
        }

        // Increase constraint index
        cnt += A_info.n;
    }

    /** Bounding box constraints **/
    LOG(INFO) << "bounding box constraints";
    for (const Binding<BoundingBoxConstraint>& binding :
         program().getBoundingBoxConstraints()) {
        auto x_indices = binding.input_index[0];
        auto p_indices = binding.input_index[1];

        std::vector<double> pi;
        for (const auto& i : p_indices) {
            // Create vector of input
            pi.emplace_back(program().p()[i]);
        }

        // Evaluate coefficients for the cost a^T x + b
        evaluator_out_info<BoundingBoxConstraint> c_info;
        evaluator_out_data<BoundingBoxConstraint> c_data;
        binding.get()->info(info);

        // todo - bounds can be parameterised (e.g. lbx(p), ubx(p))
        binding.get()->bounds(std::vector<const double*>({pi.data()}).data(),
                              binding.get()->bounds.lower,
                              binding.get()->bounds.upper);

        // Create row indices
        std::vector<evaluator_traits<LinearConstraint>::index_type> row_indices;
        for (index_type i = 0; i < A_info.n; ++i) {
            row_indices.push_back(cnt + i);
        };

        for (index_type i = 0; i < info.n; ++i) {
            data.ubx[row_indices[i]] = binding.get()->bounds.upper[i];
            data.lbx[row_indices[i]] = binding.get()->bounds.lower[i];
        }
    }

    int nWSR = options_.nWSR;

    // Solve
    if (info_.number_of_solves > 0 && options_.perform_hotstart) {
        Profiler("qpoases_solver");
        // Use previous solution to hot-start the program
        qp_->hotstart(H.data(), g_data.data(), A.data(), data.lbx.data(),
                      data.ubx.data(), data.lbA.data(), data.ubA.data(), nWSR);
    } else {
        Profiler("qpoases_solver");
        // Initialise the program and solve it
        qp_->init(H.data(), g_data.data(), A.data(), data.lbx.data(),
                  data.ubx.data(), data.lbA.data(), data.ubA.data(), nWSR);
    }

    // Collect information
    info_.nWSR = nWSR;
    info_.status = qp_->getStatus();
    // info_.iterations = ;  // !

    info_.number_of_solves++;

    // Get results
    if (info_.status == qpOASES::QProblemStatus::QPS_SOLVED) {
        info_.success = true;
        qp_->getPrimalSolution(results_.x.data());
    }
};

void qpoases_solver_instance::reset() { info_.number_of_solves = 0; }

// qpOASESSparseSolverInstance::qpOASESSparseSolverInstance(
//     mathematical_program& program)
//     : SolverBase(program) {
//     LOG(INFO) << "qpOASESSparseSolverInstance::qpOASESSparseSolverInstance";
//     // Create problem
//     int nx = program.numberOfDecisionVariables();
//     int ng = program.numberOfConstraints();

//     qp_ = std::make_unique<qpOASES::SQProblem>(nx, ng);

//     auto inserter_set_to = [](ublas::mapped_matrix<double>& m, std::size_t i,
//                               std::size_t j, const double& v) { m(i, j) = v
//                               };

//     // Create sparsity patterns for the problem
//     ublas::mapped_matrix<double> constraint_jacobian;
//     getJacobianStructure(constraint_jacobian, program.getAllConstraints(),
//                          program.getDecisionVariableIndexMap(),
//                          inserter_set_to);

//     ublas::mapped_matrix<double> objective_hessian;
//     getHessianStructure(objective_hessian, program.getAllCosts(),
//                         program.getDecisionVariableIndexMap(),
//                         inserter_set_to);
//     getHessianStructure(objective_hessian, program.getAllConstraints(),
//                         program.getDecisionVariableIndexMap(),
//                         inserter_set_to);

//     // Wrap data
//     qpoases_matrix_data_.A = qpOASES::SparseMatrix(
//         matrix_data_.A.n, matrix_data_.A.m, matrix_data_.A.indices,
//         matrix_data_.A.indptr, matrix_data_.A.values);

//     // Create variable bounds from bounding box constraints

//     // program().getBoundingBoxConstraints();

//     // Compute locations of each
// }

// qpOASESSparseSolverInstance::~qpOASESSparseSolverInstance() = default;

// void qpOASESSparseSolverInstance::reset() { info_.number_of_solves = 0; }

// void qpOASESSparseSolverInstance::solve() {
//     // Matrix inserter functions
//     auto inserter_add_to = [](ublas::mapped_matrix<double>& m, std::size_t i,
//                               std::size_t j, const double& v) { m(i, j) += v
//                               };

//     auto inserter_set_to = [](ublas::mapped_matrix<double>& m, std::size_t i,
//                               std::size_t j, const double& v) { m(i, j) = v
//                               };

//     /** Linear costs **/
//     LOG(INFO) << "linear costs";
//     for (const Binding<LinearCost>& binding : program().getLinearCosts()) {
//         auto x_indices = program().getDecisionVariableIndices(binding.in[0]);
//         auto p_indices = program().getParameterIndices(binding.in[1]);

//         std::vector<double> pi;
//         for (const auto& i : p_indices) {
//             // Create vector of input
//             pi.emplace_back(program().p()[i]);
//         }

//         // Evaluate coefficients for the cost a^T x + b
//         std::vector<double> a(binding.get()->a_nnz);
//         double b;
//         binding.get()->a(std::vector<const double*>({pi.data()}).data(),
//                          {a.data()});
//         binding.get()->b(std::vector<const double*>({pi.data()}).data(),
//         {&b});

//         // setBlock(matrix_data_.H, a_info, a_data, x, inserter_add_to);
//         // setBlock(matrix_data_.g, b_info, b_data, x, {0}, inserter_add_to);
//     }

//     /** Quadratic costs **/
//     for (const Binding<QuadraticCost>& binding :
//          program().getQuadraticCosts()) {
//         const auto& x = binding.input_index[0];
//         const auto& p = binding.input_index[1];

//         std::vector<double> pi;
//         for (const auto& i : p) {
//             // Create vector of input
//             pi.emplace_back(program().p()[i]);
//         }

//         // Evaluate coefficients for the cost a^T x + b
//         evaluator_out_info<QuadraticCost> A_info, b_info;
//         binding.get()->A_info(A_info);
//         binding.get()->b_info(b_info);

//         evaluator_out_data<QuadraticCost> A, b;
//         binding.get()->A(
//             evaluator_input_generator<QuadraticCost>({pi.data()}).data(),
//             {A.values});
//         binding.get()->b(
//             evaluator_input_generator<QuadraticCost>({pi.data()}).data(),
//             {b.values});

//         setBlock(matrix_data_.H, A_info, A_data, x, x, inserter_add_to);
//         setBlock(matrix_data_.g, b_info, b_data, x, {0}, inserter_add_to);
//     }

//     /** Linear constraints **/
//     LOG(INFO) << "linear constraints";
//     std::size_t cnt = 0;
//     for (const Binding<LinearConstraint>& binding :
//          program().getLinearConstraints()) {
//         const auto& x = binding.input_index;

//         auto p_indices = program().getParameterIndices(binding.in[1]);

//         std::vector<double> pi;
//         for (const auto& i : p_indices) {
//             // Create vector of input
//             pi.emplace_back(program().p()[i]);
//         }

//         // Evaluate coefficients for the cost a^T x + b
//         evaluator_out_info<LinearConstraint> A, b;
//         binding.get()->A_info(A);
//         binding.get()->b_info(b);

//         std::vector<double> A(A_nnz), b(b_nnz);
//         binding.get()->A(std::vector<const double*>({pi.data()}).data(),
//                          {A.data()});
//         binding.get()->b(std::vector<const double*>({pi.data()}).data(),
//                          {b.data()});

//         setBlock(matrix_data_.A, A_info, A_data, x, x, inserter_add_to);

//         // Add to each bound

//         // Add to Jacobian

//         // data_.ubA.middleRows(cnt, m) = binding.get()->ub - b;
//         // data_.lbA.middleRows(cnt, m) = binding.get()->lb - b;

//         // Increase constraint index
//         cnt += A_n;
//     }

//     int nWSR = options_.nWSR;

//     // Map created matrices to

//     // Solve
//     if (info_.number_of_solves > 0 && options_.perform_hotstart) {
//         // Use previous solution to hot-start the program
//         qp_->hotstart(H.data(), g_data.data(), A.data(), data_.lbx.data(),
//                       data_.ubx.data(), data_.lbA.data(), data_.ubA.data(),
//                       nWSR);
//     } else {
//         // Initialise the program and solve it
//         qp_->init();
//     }

//     // Collect information
//     info_.nWSR = nWSR;
//     info_.status = qp_->getStatus();
//     info_.iterations = 100;  // !

//     info_.number_of_solves++;

//     // Get results
//     if (info_.status == qpOASES::QProblemStatus::QPS_SOLVED) {
//         info_.success = true;
//         qp_->getPrimalSolution(results_.x.data());
//     }
// };

}  // namespace solvers
}  // namespace bopt