#include "bopt/sparse.hpp"

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <list>

#include "boost/numeric/ublas/io.hpp"
#include "bopt/logging.hpp"
#include "bopt/profiler.hpp"

class TestEvaluator : public bopt::EvaluatorBase<double> {
   public:
    typedef typename bopt::EvaluatorBase<double> Base;

    TestEvaluator() {
        out_m = 2;
        out_n = 2;
        out_nnz = 2;

        sparsity_out.resize(2 + out_n + 1 + out_nnz);
        sparsity_out[0] = out_m;
        sparsity_out[1] = out_n;
        // Column index pointers
        sparsity_out[2] = 0;
        sparsity_out[3] = 1;
        // Number of non-zeros
        sparsity_out[4] = out_nnz;

        // Row indices
        sparsity_out[5] = 0;
        sparsity_out[6] = 1;

        buffer.assign(2, 0.0);
    }

    index_type operator()(const value_type **arg, value_type *res) override {
        buffer[0] = 1.0;
        buffer[1] = 2.0;

        res = buffer.data();

        return index_type(0);
    }

    index_type info(bopt::evaluator_out_info<TestEvaluator> &info) {
        info.m = out_m;
        info.n = out_n;

        info.type = bopt::evaluator_matrix_type::Sparse;

        info.nnz = out_nnz;

        info.sparsity_out = sparsity_out.data();

        info.values = buffer.data();
        return index_type(0);
    }
};

class TestEvaluatorDense : public bopt::EvaluatorBase<double> {
   public:
    typedef typename bopt::EvaluatorBase<double> Base;

    TestEvaluatorDense() {
        out_m = 2;
        out_n = 2;
        out_nnz = 4;

        buffer.assign(4, 0.0);
    }

    index_type operator()(const value_type **arg, value_type *res) override {
        buffer[0] = 1.0;
        buffer[1] = 2.0;
        buffer[2] = 3.0;
        buffer[3] = 4.0;

        res = buffer.data();

        return index_type(0);
    }

    index_type info(bopt::evaluator_out_info<TestEvaluatorDense> &info) {
        info.m = out_m;
        info.n = out_n;

        info.type = bopt::evaluator_matrix_type::Dense;

        info.nnz = out_nnz;

        info.values = buffer.data();
        return index_type(0);
    }
};

TEST(Evaluator, Initialise) {
    // TestEvaluator eval;
    TestEvaluatorDense eval;

    bopt::evaluator_out_info<TestEvaluatorDense> info;
    eval.info(info);

    // Evaluate the system
    bopt::evaluator_out_data<TestEvaluatorDense> data;

    for (int i = 0; i < 1000; ++i) {
        bopt::Profiler test("evaluate");
        eval(nullptr, eval.buffer.data());
    }

    data.values = eval.buffer.data();

    // Add to a sparse matrix

    auto inserter = [](auto &m, int i, int j, double v) { m(i, j) = v; };

    for (int i = 0; i < 1000; ++i) {
        bopt::Profiler test("setBlock");

        boost::numeric::ublas::mapped_matrix<double> m(4, 4);
        bopt::setBlock(m, info, data, std::vector<int>({1, 2}),
                       std::vector<int>({1, 2}), inserter);
    }

    // LOG(INFO) << m;

    EXPECT_EQ(info.m, 2);
    EXPECT_EQ(info.n, 2);
    EXPECT_EQ(info.nnz, 2);
}

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    testing::InitGoogleTest(&argc, argv);
    FLAGS_logtostderr = 1;
    int status = RUN_ALL_TESTS();
    bopt::Profiler summary;
    return status;
}