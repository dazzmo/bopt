#pragma once

#include "bopt/MathTypes.hpp"

namespace bopt {

enum class FunctionType { DENSE, SPARSE };

template <typename ScalarType>
struct FunctionTraits {
    using Scalar = ScalarType;

    using DenseVector = typename MathTypes<Scalar>::VectorX;
    using InputVector = DenseVector;
    using InputVectorConstRef = Eigen::Ref<const InputVector>;
};

template <typename ScalarType>
struct SparseFunctionTraits : public FunctionTraits<ScalarType> {
    using Base = FunctionTraits<ScalarType>;

    using Scalar = typename Base::Scalar;
    using InputVector = typename Base::InputVector;
    using InputVectorConstRef = typename Base::InputVectorConstRef;

    using VectorType = typename MathTypes<Scalar>::SparseMatrix;
    using HessianType = typename MathTypes<Scalar>::SparseMatrix;

    static constexpr FunctionType type = FunctionType::SPARSE;
};

template <typename ScalarType>
struct DenseFunctionTraits : public FunctionTraits<ScalarType> {
    using Base = FunctionTraits<ScalarType>;

    using Scalar = typename Base::Scalar;
    using InputVector = typename Base::InputVector;
    using InputVectorConstRef = typename Base::InputVectorConstRef;

    using OutputType = Scalar;
    using VectorType = typename MathTypes<Scalar>::MatrixX;
    using HessianType = typename MathTypes<Scalar>::MatrixX;

    static constexpr FunctionType type = FunctionType::SPARSE;
};

}  // namespace bopt