#include "bopt/solvers/qpoases.hpp"

namespace bopt {
namespace solvers {

qpoases_solver::qpoases_solver(MathematicalProgram& program) : solver(program) {
    LOG(INFO) << "qpoases_solver::qpoases_solver";

    // Create problem
    int nx = program.n_variables();
    int ng = program.n_constraints();

    qp_ = std::make_unique<qpOASES::SQProblem>(nx, ng);

    // Create cost and constraint data
    linear_cost_data_.reserve(program.n_costs());
    for (const auto& c : program.linearCosts()) {
        linear_cost_data_.push_back(LinearCostData(*c.get()));
    }
    quadratic_cost_data_.reserve(program.n_costs());
    for (const auto& c : program.quadraticCosts()) {
        quadratic_cost_data_.push_back(QuadraticCostData(*c.get()));
    }
    linear_constraint_data_.reserve(program.n_constraints());
    for (const auto& c : program.linearConstraints()) {
        linear_constraint_data_.push_back(LinearConstraintData(*c.get()));
    }

    // Create matrix data
    data.H.resize(nx, nx);
    data.H.setZero();

    data.g.resize(nx);
    data.g.setZero();

    data.A.resize(ng, nx);
    data.A.setZero();

    data.lbA.resize(ng);
    data.ubA.resize(ng);

    data.lbx.resize(nx);
    data.ubx.resize(nx);

    data.lbx = program.variableLowerBounds();
    data.ubx = program.variableUpperBounds();

    VLOG(10) << "lbx: " << data.lbx.transpose();
    VLOG(10) << "ubx: " << data.ubx.transpose();
}

qpoases_solver::~qpoases_solver() = default;

void qpoases_solver::solve(MathematicalProgram& program) {
    Eigen::MatrixXd tmp;

    /** Bounding box constraints **/
    {
        bopt::profiler profiler("qpoases: bounding box constraints");
        for (auto& binding : program.boundingBoxConstraints()) {
            // data.lbx(binding.indices().indices())
            //     << binding.get()->lowerBound();
            // data.ubx(binding.indices().indices())
            //     << binding.get()->upperBound();
        }
    }

    /** Linear costs **/
    {
        bopt::profiler profiler("qpoases: linear costs");
        VLOG(10) << "qpoases:linear costs";
        int i = 0;
        for (auto& binding : program.linearCosts()) {
            const auto& c = *binding.get();
            const auto& indices = binding.indices().indices();

            // Create vector
            LinearCostData& cdata = linear_cost_data_[i];
            if (cdata.a_s.nonZeros()) {
                c.evalSparseCoefficients(cdata);

            } else {
                c.evalCoefficients(cdata);
                data.g(indices) += cdata.a;
            }

            // Add coefficient vector
            i++;
        }
    }

    /** Quadratic costs **/
    {
        bopt::profiler profiler("qpoases: quadratic costs");
        VLOG(10) << "qpoases:quadratic costs";
        int i = 0;
        for (auto& binding : program.quadraticCosts()) {
            auto& c = *binding.get();
            const auto& indices = binding.indices().indices();

            // Create vector
            QuadraticCostData& cdata = quadratic_cost_data_[i];
            if (cdata.A_s.nonZeros()) {
                c.evalSparseCoefficients(cdata);
            } else {
                c.evalCoefficients(cdata);
                data.H(indices, indices) += cdata.A;
                data.g(indices) += cdata.b;
            }
            i++;
        }
    }

    /** Linear constraints **/
    {
        bopt::profiler profiler("qpoases: linear constraints");

        VLOG(10) << "qpoases:linear constraints";
        int row = 0;
        int i = 0;
        for (auto& binding : program.linearConstraints()) {
            auto& c = *binding.get();
            const auto& indices = binding.indices().indices();

            // Create vector
            LinearConstraintData& cdata = linear_constraint_data_[i];
            // todo - if sparse
            if (cdata.A_s.nonZeros()) {
                c.evalSparseCoefficients(cdata);
            } else {
                c.evalCoefficients(cdata);
                data.A.middleRows(row, c.dim_output()) = cdata.A;
            }

            // Evaluate bounds
            c.evalBounds(cdata);
            data.lbA.middleRows(row, c.dim_output()) = cdata.lb;
            data.ubA.middleRows(row, c.dim_output()) = cdata.ub;

            // Increment
            row += c.dim_output();
            i++;
        }
    }

    int nWSR = options_.nWSR;

    qp_->setHessianType(qpOASES::HessianType::HST_POSDEF);
    // todo - set this only once?
    qp_->setOptions(options_);

    VLOG(10) << "H: " << data.H;
    VLOG(10) << "g: " << data.g;
    VLOG(10) << "A: " << data.A;
    VLOG(10) << "lbA: " << data.lbA;
    VLOG(10) << "ubA: " << data.ubA;

    // Solve
    if (info_.number_of_solves > 0 && options_.perform_hotstart) {
        bopt::profiler profiler("qpoases: solve");
        // Use previous solution to hot-start the program
        // qpOASES::SymDenseMat(nx, nx, 0, data.H.data());
        qp_->hotstart(data.H.data(), data.g.data(), data.A.data(),
                      data.lbx.data(), data.ubx.data(), data.lbA.data(),
                      data.ubA.data(), nWSR);
    } else {
        bopt::profiler profiler("qpoases: solve");
        // Initialise the program and solve it
        qp_->init(data.H.data(), data.g.data(), data.A.data(), data.lbx.data(),
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
        qp_->getPrimalSolution(this->primal_solution().data());
        VLOG(10) << "primal_solution: " << this->primal_solution().transpose();
    }
};

void qpoases_solver::reset() { info_.number_of_solves = 0; }

}  // namespace solvers
}  // namespace bopt