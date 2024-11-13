#ifndef CONSTRAINTS_BASE_H
#define CONSTRAINTS_BASE_H

#include <Eigen/Core>
#include <memory>

#include "bopt/bounds.hpp"
#include "bopt/evaluator.hpp"
#include "bopt/logging.hpp"

namespace bopt {

template <typename T>
struct constraint_traits {
    typedef typename T::id_type id_type;

    typedef typename T::shared_ptr shared_ptr;
    typedef typename T::unique_ptr unique_ptr;
};

template <typename T>
struct constraint_attributes {};

template <typename T>
class constraint : public evaluator<T> {
   public:
    typedef std::shared_ptr<constraint> shared_ptr;
    typedef std::unique_ptr<constraint> unique_ptr;

    typedef std::size_t id_type;

    typedef evaluator<T> evaluator_t;

    typedef typename evaluator_traits<evaluator_t>::value_type value_type;
    typedef typename evaluator_traits<evaluator_t>::index_type index_type;
    typedef typename evaluator_traits<evaluator_t>::integer_type integer_type;
    typedef typename evaluator_t::out_info_t out_info_t;
    typedef typename evaluator_t::out_data_t out_data_t;

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

    virtual integer_type hes(const value_type **arg, value_type *res) {
        return integer_type(0);
    }

    virtual integer_type jac_info(out_info_t &info) { return integer_type(0); }

    virtual integer_type hes_info(out_info_t &info) { return integer_type(0); }

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
    typedef typename Base::out_info_t out_info_t;
    typedef typename Base::out_data_t out_data_t;

    linear_constraint() = default;
    linear_constraint(const index_type &sz,
                      const bound_type::type &type = bound_type::Unbounded)
        : constraint<T>(sz, type) {}

    virtual integer_type A(const double **arg, double *res) {
        return integer_type(0);
    }

    virtual integer_type A_info(out_info_t &info) {
        LOG(INFO) << "In default class";
        return integer_type(0);
    }

    virtual integer_type b(const double **arg, double *res) {
        return integer_type(0);
    }

    virtual integer_type b_info(out_info_t &info) { return integer_type(0); }

   private:
};

template <typename T>
class bounding_box_constraint : public constraint<T> {
   public:
    typedef std::shared_ptr<bounding_box_constraint> shared_ptr;
    typedef std::unique_ptr<bounding_box_constraint> unique_ptr;

    typedef constraint<T> Base;
    typedef typename evaluator_traits<Base>::value_type value_type;
    typedef typename evaluator_traits<Base>::index_type index_type;
    typedef typename evaluator_traits<Base>::integer_type integer_type;
    typedef typename Base::out_info_t out_info_t;
    typedef typename Base::out_data_t out_data_t;

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
