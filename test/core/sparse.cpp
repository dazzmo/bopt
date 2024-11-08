#include "bopt/sparse.hpp"

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/logging.hpp"
#include "bopt/profiler.hpp"

TEST(Sparse, IdentityCCS) {
    int n = 10;
    // Create basic identity matrix
    bopt::CompressedColumnStorageFormat mat(n, n);

    mat.insert(0, 1, 1.0);
    mat.insert(0, 2, 1.0);

    LOG(INFO) << mat.print().str();

    LOG(INFO) << "Values";
    std::stringstream ss;
    for (const int &v : mat.values) {
        ss << v << " ";
    }
    LOG(INFO) << ss.str();

    ss.str(std::string());

    LOG(INFO) << "Indices";
    for (const int &i : mat.indices) {
        ss << i << " ";
    }
    LOG(INFO) << ss.str();

    ss.str(std::string());

    LOG(INFO) << "Indice Pointers";
    for (const int &p : mat.indptr) {
        ss << p << " ";
    }
    LOG(INFO) << ss.str();

    // EXPECT_EQ(mat.values.size(), n);
    // EXPECT_EQ(mat.indices.size(), n);
    // EXPECT_EQ(mat.indptr.size(), n);
}

TEST(Sparse, BuildFromMatrices) {
    int n = 10000;
    int m = 100;

    for (int k = 0; k < 100; ++k) {
        bopt::Profiler a("Sparse Matrix Construction");
        // Create basic identity matrix
        bopt::CompressedColumnStorageFormat mat(n, n);
        bopt::CompressedColumnStorageFormat lil(m, m);

        for (int i = 0; i < m / 10; ++i) {
            for (int j = 0; j < m / 10; ++j) {
                int row = rand() % m;
                int col = rand() % m;

                lil.insert(row, col, 1.0);
            }
        }

        int r_offset = 0;
        int c_offset =  0;

        // Determine locations for each non-zero entry
        for (int col = 0; col < lil.m; ++col) {
            int start = lil.indptr[col];
            int end = lil.indptr[col + 1];

            for (int row = start; row < end; ++row) {
                // Get index of entry
                int idx = lil.indices[row];
                // Add entry to full Jacobian
                mat.insert(r_offset + idx, c_offset + col, lil.values[row]);
            }
        }
    }
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