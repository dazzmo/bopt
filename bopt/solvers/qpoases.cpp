#include "bopt/solvers/qpoases.hpp"

#include "boost/numeric/ublas/io.hpp"
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
    data.H.clear();

    data.g.resize(nx);

    data.A.resize(ng, nx);
    data.A.clear();

    data.lbA.resize(ng);
    data.ubA.resize(ng);

    data.lbx.resize(nx);
    data.ubx.resize(nx);

    for (int i = 0; i < nx; ++i) {
        set(data.lbx, i, program.variable_bounds()[i].lower);
        set(data.ubx, i, program.variable_bounds()[i].upper);
    }
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
        LOG(INFO) << "linear cost";
        const auto& x_indices = binding.input_indices[0];
        // Create input vector
        std::vector<std::vector<double>> p_data;
        std::vector<const double*> in = {};
        for (std::size_t i = 1; i < binding.input_indices.size(); ++i) {
            const auto& p_indices = binding.input_indices[i];
            p_data.push_back(create_indexed_view(program().p(), p_indices));
            in.push_back(p_data.back().data());
        }

        linear_cost<double>::out_info_t a_info;
        linear_cost<double>::out_data_t a_data(a_info);
        binding.get()->a_info(a_info);

        // Evaluate coefficients for the cost a^T x + b
        binding.get()->a(in.data(), {a_data.values.data()});

        set_block(data.g, a_info, a_data, x_indices, {0}, vector_add_to);
    }

    LOG(INFO) << "quadratic costs";
    /** Quadratic costs **/
    for (const binding<quadratic_cost<double>>& binding :
         program().quadratic_costs()) {
        LOG(INFO) << "quadratic cost";
        const auto& x_indices = binding.input_indices[0];

        // Create input vector
        std::vector<std::vector<double>> p_data;
        std::vector<const double*> in = {};
        for (std::size_t i = 1; i < binding.input_indices.size(); ++i) {
            const auto& p_indices = binding.input_indices[i];
            p_data.push_back(create_indexed_view(program().p(), p_indices));
            in.push_back(p_data.back().data());
        }

        // Evaluate coefficients for the cost x^T A x + b^T x + c
        quadratic_cost<double>::out_info_t A_info, b_info;
        binding.get()->A_info(A_info);
        binding.get()->b_info(b_info);

        VLOG(10) << "A_info n = " << A_info.n << " m = " << A_info.m
                 << " nnz = " << A_info.nnz;
        VLOG(10) << "b_info n = " << b_info.n << " m = " << b_info.m
                 << " nnz = " << b_info.nnz;
        // Evaluate the coefficients
        quadratic_cost<double>::out_data_t A_data(A_info), b_data(b_info);
        LOG(INFO) << "A";
        binding.get()->A(in.data(), {A_data.values.data()});
        LOG(INFO) << "b";
        binding.get()->b(in.data(), {b_data.values.data()});

        LOG(INFO) << "Blocks";
        set_block(data.H, A_info, A_data, x_indices, x_indices,
                  inserter_add_to);
        set_block(data.g, b_info, b_data, x_indices, {0}, vector_add_to);
    }

    /** Linear constraints **/
    LOG(INFO) << "linear constraints";
    // todo - custom binding row orderings, much like in variables case
    std::size_t cnt = 0;
    for (const binding<linear_constraint<double>>& binding :
         program().linear_constraints()) {
        typedef evaluator_traits<linear_constraint<double>>::index_type
            index_type;

        const auto& x_indices = binding.input_indices[0];

        // Create input vector
        std::vector<std::vector<double>> p_data;
        std::vector<const double*> in = {};
        for (std::size_t i = 1; i < binding.input_indices.size(); ++i) {
            const auto& p_indices = binding.input_indices[i];
            p_data.push_back(create_indexed_view(program().p(), p_indices));
            VLOG(10) << "pi = " << p_data.back();
            in.push_back(p_data.back().data());
        }


        // Evaluate coefficients for the constraint  lbA < A x + b < ubA
        linear_constraint<double>::out_info_t A_info, b_info;
        binding.get()->A_info(A_info);
        binding.get()->b_info(b_info);
        // Evaluate the coefficients
        linear_constraint<double>::out_data_t A_data(A_info), b_data(b_info);
        VLOG(10) << "A_info n = " << A_info.n << " m = " << A_info.m
                 << " nnz = " << A_info.nnz;
        VLOG(10) << "b_info n = " << b_info.n << " m = " << b_info.m
                 << " nnz = " << b_info.nnz;
        LOG(INFO) << "A";
        LOG(INFO) << "A data = " << A_data.values;
        binding.get()->A(in.data(), {A_data.values.data()});
        LOG(INFO) << "b";
        binding.get()->b(in.data(), {b_data.values.data()});

        // Create row indices
        std::vector<index_type> row_indices;
        for (index_type i = 0; i < A_info.m; ++i) {
            row_indices.push_back(cnt + i);
        };

        set_block(data.A, A_info, A_data, row_indices, x_indices,
                  inserter_set_to);

        // Add to each bound
        for (index_type i = 0; i < b_info.m; ++i) {
            data.ubA[row_indices[i]] =
                binding.get()->bounds[i].upper - b_data.values[i];
            data.lbA[row_indices[i]] =
                binding.get()->bounds[i].lower - b_data.values[i];
        }

        // Increase constraint index
        cnt += A_info.m;
    }

    /** Bounding box constraints **/
    LOG(INFO) << "bounding box constraints";
    for (const binding<bounding_box_constraint<double>>& binding :
         program().bounding_box_constraints()) {
        const auto& x_indices = binding.input_indices[0];

        // Create input vector
        std::vector<std::vector<double>> p_data;
        std::vector<const double*> in = {};
        for (std::size_t i = 1; i < binding.input_indices.size(); ++i) {
            const auto& p_indices = binding.input_indices[i];
            p_data.push_back(create_indexed_view(program().p(), p_indices));
            in.push_back(p_data.back().data());
        }

        binding.get()->update_bounds(in.data());

        for (index_type i = 0; i < x_indices.size(); ++i) {
            data.ubx[x_indices[i]] = std::min(data.ubx[x_indices[i]],
                                              binding.get()->bounds[i].upper);
            data.lbx[x_indices[i]] = std::max(data.lbx[x_indices[i]],
                                              binding.get()->bounds[i].lower);
        }
    }

    int nWSR = options_.nWSR;

    qp_->setHessianType(qpOASES::HessianType::HST_POSDEF);

    VLOG(10) << data.H;
    VLOG(10) << data.g;
    VLOG(10) << data.A;
    VLOG(10) << data.lbA;
    VLOG(10) << data.ubA;

    VLOG(10) << data.lbx;
    VLOG(10) << data.ubx;

    // Solve
    if (info_.number_of_solves > 0 && options_.perform_hotstart) {
        profiler("qpoases_solver");
        // Use previous solution to hot-start the program
        // qpOASES::SymDenseMat(nx, nx, 0, data.H.data());
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

}  // namespace solvers
}  // namespace bopt