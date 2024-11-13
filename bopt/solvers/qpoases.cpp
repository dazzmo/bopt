#include "bopt/solvers/qpoases.hpp"

#include "bopt/sparse.hpp"

namespace bopt {
namespace solvers {

qpoases_solver_instance::qpoases_solver_instance(
    mathematical_program<double>& program)
    : solver(program) {
    LOG(INFO) << "qpoases_solver_instance::qpoases_solver_instance";

    // Create problem
    int nx = program.n_variables();
    int ng = program.n_constraints();

    qp_ = std::make_unique<qpOASES::SQProblem>(nx, ng);

    // Create matrix data
    data.H.resize(nx, nx);
    data.g.resize(nx);

    data.A.resize(ng, nx);
    data.lbA.resize(ng);
    data.ubA.resize(ng);

    data.lbA.resize(nx);
    data.ubA.resize(nx);
}

qpoases_solver_instance::~qpoases_solver_instance() = default;

void qpoases_solver_instance::solve() {
    // Matrix inserter functions
    auto inserter_add_to = [](ublas::matrix<double>& m, std::size_t i,
                              std::size_t j, const double& v) { m(i, j) += v; };

    auto vector_add_to = [](std::vector<double>& m, std::size_t i,
                            std::size_t j, const double& v) { m[i] += v; };

    auto inserter_set_to = [](ublas::matrix<double>& m, std::size_t i,
                              std::size_t j, const double& v) { m(i, j) = v; };

    /** Linear costs **/
    LOG(INFO) << "linear costs";
    for (const binding<linear_cost<double>>& binding :
         program().linear_costs()) {
        auto x_indices = binding.input_indices[0];
        auto p_indices = binding.input_indices[1];

        // todo - shorten this
        std::vector<double> pi;
        for (const auto& i : p_indices) {
            // Create vector of input
            pi.emplace_back(program().p()[i]);
        }

        linear_cost<double>::out_info_t a_info;
        linear_cost<double>::out_data_t a_data(a_info);
        binding.get()->a_info(a_info);

        // Evaluate coefficients for the cost a^T x + b
        binding.get()->a(std::vector<const double*>({pi.data()}).data(),
                         {a_data.values.data()});

        set_block(data.g, a_info, a_data, x_indices, {0}, vector_add_to);
    }

    LOG(INFO) << "g = " << data.g;

    LOG(INFO) << "quadratic costs";
    /** Quadratic costs **/
    for (const binding<quadratic_cost<double>>& binding :
         program().quadratic_costs()) {
        auto x_indices = binding.input_indices[0];
        auto p_indices = binding.input_indices[1];

        std::vector<double> pi;
        for (const auto& i : p_indices) {
            // Create vector of input
            pi.emplace_back(program().p()[i]);
        }

        // Evaluate coefficients for the cost a^T x + b
        quadratic_cost<double>::out_info_t A_info, b_info;
        binding.get()->A_info(A_info);
        binding.get()->b_info(b_info);

        // evaluator_out_data<quadratic_cost<double>> A_data, b_data;
        // binding.get()->A(std::vector<const double*>({pi.data()}).data(),
        //                  {A_data.values});
        // binding.get()->b(std::vector<const double*>({pi.data()}).data(),
        //                  {b_data.values});

        // set_block(data.H, A_info, A_data, x_indices, x_indices,
        //           inserter_add_to);
        // set_block(data.g, b_info, b_data, x_indices, {0}, vector_add_to);
    }

    /** Linear constraints **/
    LOG(INFO) << "linear constraints";
    std::size_t cnt = 0;
    for (const binding<linear_constraint<double>>& binding :
         program().linear_constraints()) {
        LOG(INFO) << "x";
        auto x_indices = binding.input_indices[0];
        LOG(INFO) << "p";
        auto p_indices = binding.input_indices[1];

        std::vector<double> pi;
        for (const auto& i : p_indices) {
            // Create vector of input
            pi.emplace_back(program().p()[i]);
        }

        // Evaluate coefficients for the cost a^T x + b
        linear_constraint<double>::out_info_t A_info, b_info;
        linear_constraint<double>::out_data_t A_data(A_info), b_data(b_info);
        binding.get()->A_info(A_info);
        binding.get()->b_info(b_info);

        binding.get()->A(std::vector<const double*>({pi.data()}).data(),
                         {A_data.values.data()});
        binding.get()->b(std::vector<const double*>({pi.data()}).data(),
                         {b_data.values.data()});

        // Create row indices
        std::vector<evaluator_traits<linear_constraint<double>>::index_type>
            row_indices;
        for (index_type i = 0; i < A_info.n; ++i) {
            row_indices.push_back(cnt + i);
        };
        LOG(INFO) << "setting block";

        set_block(data.A, A_info, A_data, row_indices, x_indices,
                  inserter_set_to);

        LOG(INFO) << "block set";

        // Add to each bound

        for (index_type i = 0; i < b_info.n; ++i) {
            data.ubA[row_indices[i]] =
                binding.get()->bounds[i].upper - b_data.values[i];
            data.lbA[row_indices[i]] =
                binding.get()->bounds[i].lower - b_data.values[i];
        }

        LOG(INFO) << "bounds made";

        // Increase constraint index
        cnt += A_info.n;
    }

    /** Bounding box constraints **/
    LOG(INFO) << "bounding box constraints";
    for (const binding<bounding_box_constraint<double>>& binding :
         program().bounding_box_constraints()) {
        auto x_indices = binding.input_indices[0];
        auto p_indices = binding.input_indices[1];

        // todo - fix this
        // todo - xbu = std::min(xbu[i], bound.upper)
        // todo - xbl = std::max(xbl[i], bound.lower)
    }

    int nWSR = options_.nWSR;

    // Solve
    if (info_.number_of_solves > 0 && options_.perform_hotstart) {
        profiler("qpoases_solver");
        // Use previous solution to hot-start the program
        qp_->hotstart(data.H.data().begin(), data.g.data(),
                      data.A.data().begin(), data.lbx.data(), data.ubx.data(),
                      data.lbA.data(), data.ubA.data(), nWSR);
    } else {
        profiler("qpoases_solver");
        // Initialise the program and solve it
        qp_->init(data.H.data().begin(), data.g.data(), data.A.data().begin(),
                  data.lbx.data(), data.ubx.data(), data.lbA.data(),
                  data.ubA.data(), nWSR);
    }

    // Collect information
    info_.nWSR = nWSR;
    info_.status = qp_->getStatus();
    // info_.iterations = ;  // !

    info_.number_of_solves++;

    // Get results
    if (info_.status == qpOASES::QProblemStatus::QPS_SOLVED) {
        info_.success = true;
        // qp_->getPrimalSolution(results_.x.data());
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
//     for (const binding<linear_cost>& binding : program().getlinear_costs()) {
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

//         // set_block(matrix_data_.H, a_info, a_data, x, inserter_add_to);
//         // set_block(matrix_data_.g, b_info, b_data, x, {0},
//         inserter_add_to);
//     }

//     /** Quadratic costs **/
//     for (const binding<quadratic_cost<double>>& binding :
//          program().getquadratic_cost<double>s()) {
//         const auto& x = binding.input_indices[0];
//         const auto& p = binding.input_indices[1];

//         std::vector<double> pi;
//         for (const auto& i : p) {
//             // Create vector of input
//             pi.emplace_back(program().p()[i]);
//         }

//         // Evaluate coefficients for the cost a^T x + b
//         evaluator_out_info<quadratic_cost<double>> A_info, b_info;
//         binding.get()->A_info(A_info);
//         binding.get()->b_info(b_info);

//         evaluator_out_data<quadratic_cost<double>> A, b;
//         binding.get()->A(
//             evaluator_input_generator<quadratic_cost<double>>({pi.data()}).data(),
//             {A.values});
//         binding.get()->b(
//             evaluator_input_generator<quadratic_cost<double>>({pi.data()}).data(),
//             {b.values});

//         set_block(matrix_data_.H, A_info, A_data, x, x, inserter_add_to);
//         set_block(matrix_data_.g, b_info, b_data, x, {0}, inserter_add_to);
//     }

//     /** Linear constraints **/
//     LOG(INFO) << "linear constraints";
//     std::size_t cnt = 0;
//     for (const binding<linear_constraint<double>>& binding :
//          program().getlinear_constraint<double>s()) {
//         const auto& x = binding.input_indices;

//         auto p_indices = program().getParameterIndices(binding.in[1]);

//         std::vector<double> pi;
//         for (const auto& i : p_indices) {
//             // Create vector of input
//             pi.emplace_back(program().p()[i]);
//         }

//         // Evaluate coefficients for the cost a^T x + b
//         evaluator_out_info<linear_constraint<double>> A, b;
//         binding.get()->A_info(A);
//         binding.get()->b_info(b);

//         std::vector<double> A(A_nnz), b(b_nnz);
//         binding.get()->A(std::vector<const double*>({pi.data()}).data(),
//                          {A.data()});
//         binding.get()->b(std::vector<const double*>({pi.data()}).data(),
//                          {b.data()});

//         set_block(matrix_data_.A, A_info, A_data, x, x, inserter_add_to);

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