#ifndef CONSTRAINTS_BASE_H
#define CONSTRAINTS_BASE_H

#include <Eigen/Core>
#include <memory>

#include "bopt/logging.hpp"
#include "bopt/bounds.hpp"
#include "bopt/evaluator.hpp"

namespace bopt {

template <typename T>
struct constraint_traits : public evaluator_traits<T> {
    typedef typename T::id_type id_type;

    typedef typename T::shared_ptr shared_ptr;
    typedef typename T::unique_ptr unique_ptr;
};

template <typename T>
struct constraint_attributes {};

template <typename T>
struct constraint_index_map {};

template <typename T>
class constraint : public evaluator<T> {
   public:
    typedef std::size_t id_type;

    typedef std::shared_ptr<constraint> shared_ptr;
    typedef std::unique_ptr<constraint> unique_ptr;

    typedef evaluator<T> Base;
    typedef typename Base::value_type value_type;
    typedef typename Base::index_type index_type;
    typedef typename Base::integer_type integer_type;

    constraint() = default;

    constraint(const index_type &sz,
               const bound_type::type &type = bound_type::Unbounded)
        : bounds(sz, type) {}

   public:
    id_type id;
    std::string name;

    virtual integer_type jac(const value_type **arg, value_type *res) {
        return integer_type(0);
    }

    virtual integer_type hes(const value_type **arg, const value_type **lam,
                             value_type *res) {
        return integer_type(0);
    }

    virtual integer_type jac_info(evaluator_out_info<constraint> &info) {
        return integer_type(0);
    }

    virtual integer_type hes_info(evaluator_out_info<constraint> &info) {
        return integer_type(0);
    }

    vector_bounds<value_type> bounds;

   private:
};

template <typename T>
class linear_constraint : public constraint<T> {
   public:
    typedef std::shared_ptr<linear_constraint> shared_ptr;
    typedef std::unique_ptr<linear_constraint> unique_ptr;

    typedef constraint<T> Base;
    typedef typename Base::value_type value_type;
    typedef typename Base::index_type index_type;
    typedef typename Base::integer_type integer_type;

    linear_constraint() = default;
    linear_constraint(const index_type &sz,
                      const bound_type::type &type = bound_type::Unbounded)
        : constraint<T>(sz, type) {}

    virtual integer_type A(const double **arg, double *res) {
        return integer_type(0);
    }

    virtual integer_type A_info(evaluator_out_info<linear_constraint> &info) {
        LOG(INFO) << "In default class";
        return integer_type(0);
    }

    virtual integer_type b(const double **arg, double *res) {
        return integer_type(0);
    }

    virtual integer_type b_info(evaluator_out_info<linear_constraint> &info) {
        return integer_type(0);
    }

   private:
};

template <typename T>
class bounding_box_constraint : public constraint<T> {
   public:
    typedef std::shared_ptr<bounding_box_constraint> shared_ptr;
    typedef std::unique_ptr<bounding_box_constraint> unique_ptr;

    typedef constraint<T> Base;
    typedef typename Base::value_type value_type;
    typedef typename Base::index_type index_type;
    typedef typename Base::integer_type integer_type;

    bounding_box_constraint() = default;

    bounding_box_constraint(const index_type &sz,
                            const std::vector<value_type> &lb,
                            const std::vector<value_type> &ub)
        : constraint<T>(sz) {
        assert(lb.size() == ub.size() && lb.size() == sz &&
               "Bound vector size mismatch");
        for (index_type i = 0; i < sz; ++i) {
            this->bounds[i].set(lb[i], ub[i]);
        }
    }

    bounding_box_constraint(const index_type &sz, const bound_type::type &type)
        : constraint<T>(sz, type) {}

    /**
     * @brief Evaluates the bounds of the bounding box constraint, returning the
     * constraint
     *
     * @param arg
     * @param ret
     * @return index_type
     */
    integer_type operator()(const value_type **arg, value_type *ret) override {
        // todo - bounds
        return integer_type(0);
    }

   private:
};

}  // namespace bopt

#endif /* CONSTRAINTS_BASE_H */
