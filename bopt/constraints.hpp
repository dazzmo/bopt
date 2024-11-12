#ifndef CONSTRAINTS_BASE_H
#define CONSTRAINTS_BASE_H

#include <Eigen/Core>
#include <memory>

#include "bopt/bounds.hpp"
#include "bopt/evaluator.hpp"

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

template <typename T>
class constraint : public evaluator<T> {
   public:
    constraint() = default;

    typedef std::size_t id_type;

    typedef evaluator<T> Base;
    typedef typename Base::value_type value_type;
    typedef typename Base::index_type index_type;

    constraint(const index_type &n) : bounds(n) {}

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

    bounds<value_type> bounds;

   private:
};

template <typename T>
class linear_constraint : public constraint<T> {
   public:
    using UniquePtr = std::unique_ptr<linear_constraint>;
    using SharedPtr = std::shared_ptr<linear_constraint>;

    linear_constraint() = default;

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

template <typename T>
class bounding_box_constraint : public constraint<T> {
   public:
    typedef constraint<T> Base;
    typedef typename Base::value_type value_type;
    typedef typename Base::index_type index_type;

    using UniquePtr = std::unique_ptr<bounding_box_constraint>;
    using SharedPtr = std::shared_ptr<bounding_box_constraint>;

    bounding_box_constraint() = default;
    bounding_box_constraint(const index_type &n, const std::vector<double> &lb,
                            const std::vector<double> &ub) {
        assert(lb.size() == ub.size() && lb.size() == this->out_n &&
               "Bound vector size mismatch");
    }

    /**
     * @brief Evaluates the bounds of the bounding box constraint, returning the
     * constraint
     *
     * @param arg
     * @param ret
     * @return index_type
     */
    index_type operator()(const value_type **arg, value_type *ret) override {
        // todo - bounds
        return index_type(0);
    }

   private:
};

}  // namespace bopt

#endif /* CONSTRAINTS_BASE_H */
