#ifndef CONSTRAINTS_BASE_H
#define CONSTRAINTS_BASE_H

#include <Eigen/Core>
#include <memory>

#include "bopt/bounds.hpp"
#include "bopt/evaluator.hpp"
#include "bopt/function.hpp"

namespace bopt {

template <typename T>
struct constraint_traits : public evaluator_traits<T> {
    typedef typename T::input_vector input_vector;
    typedef typename T::id_type id_type;
};

template <typename T>
struct constraint_attributes {};

template <typename T>
struct constraint_index_map {};

template <typename T, typename I>
class constraint : public EvaluatorBase<T> {
   public:
    constraint() = default;

    typedef std::size_t id_type;

    typedef typename EvaluatorBase<T> Base;
    typedef typename Base::value_type value_type;

   public:
    id_type id;
    std::string name;

    virtual bopt_int jac(const value_type **arg, value_type *res) { return 0; }

    virtual bopt_int hes(const value_type **arg, const value_type **lam,
                         value_type *res) {
        return 0;
    }

    virtual index_type jac_info(index_type *n, index_type *m, index_type *nnz,
                                index_type *indices, index_type *indptr) {
        return 0;
    }

    virtual index_type hes_info(index_type *n, index_type *m, index_type *nnz,
                                index_type *indices, index_type *indptr) {
        return 0;
    }

    std::vector<bound<value_type>> bounds;

   private:
};

template <typename T>
class LinearConstraint : public constraint<T> {
   public:
    using UniquePtr = std::unique_ptr<LinearConstraint>;
    using SharedPtr = std::shared_ptr<LinearConstraint>;

    LinearConstraint() = default;

    virtual bopt_int A(const double **arg, double *res) { return 0; }

    virtual bopt_int A_info(bopt_int *n, bopt_int *m, bopt_int *nnz,
                            bopt_int *ind, bopt_int *inptr) {
        return 0;
    }

    virtual bopt_int b(const double **arg, double *res) { return 0; }

    virtual bopt_int b_info(bopt_int *n, bopt_int *m, bopt_int *nnz,
                            bopt_int *ind, bopt_int *inptr) {
        return 0;
    }

   private:
};

}  // namespace bopt

#endif /* CONSTRAINTS_BASE_H */
