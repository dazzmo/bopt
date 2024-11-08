
#include "bopt/ad/casadi/codegen.hpp"

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/ad/casadi/evaluator.hpp"
#include "bopt/ad/casadi/function.hpp"
#include "bopt/dlib_handler.hpp"
#include "bopt/logging.hpp"
#include "bopt/profiler.hpp"

using sym = ::casadi::SX;
using dm = ::casadi::DM;

TEST(CasadiFunction, FunctionWrapperScalar) {
    std::size_t n = 5;
    sym x = sym::sym("x", n);
    // Create symbolic constraint
    sym ex = sym::dot(x, x);

    auto fun = ::casadi::Function("function", {x}, {ex});

    // Generate code
    bopt::casadi::codegen(fun);

    // Create function to evaluate it
    std::shared_ptr<bopt::DynamicLibraryHandler> handle =
        std::make_shared<bopt::DynamicLibraryHandler>(
            "./cg_function_6817617485236275379.so");

    bopt::casadi::Evaluator test(handle, "function");

    Eigen::VectorXd y(5);
    y.setRandom();

    std::vector<const double *> in = {y.data()};
    test.eval(in.data());
}

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    testing::InitGoogleTest(&argc, argv);
    int status = RUN_ALL_TESTS();
    bopt::Profiler summary;
    return status;
}