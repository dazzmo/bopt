#pragma once
#include <memory>

#include "bopt/dlib_handler.hpp"
#include "bopt/evaluator.hpp"

namespace bopt {
namespace casadi {

/* Typedefs based on casadi format */
typedef long long int casadi_int;

typedef void (*signal_t)(void);

typedef casadi_int (*getint_t)(void);
typedef int (*work_t)(casadi_int *sz_arg, casadi_int *sz_res, casadi_int *sz_iw,
                      casadi_int *sz_w);

typedef const casadi_int *(*sparsity_t)(casadi_int ind);

typedef int (*eval_t)(const double **arg, double **res, casadi_int *iw,
                      double *w, int mem);

typedef int (*casadi_checkout_t)(void);
typedef void (*casadi_release_t)(int);

template <typename T>
class evaluator : public bopt::evaluator<T> {
   private:
    /// pointer to casadi codegen evaluation function
    eval_t f;
    /// casadi int work vector
    casadi_int *iw;
    /// casadi double work vector
    double *w;
    /// increase reference counter
    signal_t incref;
    /// decrease reference counter
    signal_t decref;
    /// input size
    int *input_size;
    /// release casadi memory
    casadi_release_t release;
    /// thread local mem id
    int mem;

    std::vector<double> work_vector_d;
    std::vector<double *> res_vec;
    std::vector<const double *> arg_vec;
    /// int work vector
    std::vector<casadi_int> work_vector_i;

   public:
    typedef bopt::evaluator<T> Base;
    typedef typename Base::value_type value_type;
    typedef typename Base::index_type index_type;
    typedef typename Base::integer_type integer_type;

    typedef typename bopt::evaluator_out_info<Base> evaluator_out_info;

    /**
     * @brief Construct a new evaluator object from a code-generated casadi
     * function
     *
     * @param handle
     * @param function_name
     */
    evaluator(const std::shared_ptr<dynamic_library_handler> &handle,
              const std::string &function_name)
        : handle_(handle) {
        void *handle_p = handle_->handle;

        /* Memory management -- increase reference counter */
        incref = (signal_t)dlsym(
            handle_p, (function_name + (std::string) "_incref").c_str());
        if (dlerror()) dlerror();  // No such function, reset error flags

        /* Memory management -- decrease reference counter */
        decref = (signal_t)dlsym(
            handle_p, (function_name + (std::string) "_decref").c_str());
        if (dlerror()) dlerror();  // No such function, reset error flags

        /* Thread-local memory management -- checkout memory */
        casadi_checkout_t checkout = (casadi_checkout_t)dlsym(
            handle_p, (function_name + (std::string) "_checkout").c_str());
        if (dlerror()) dlerror();  // No such function, reset error flags

        /* T memory management -- release memory */
        release = (casadi_release_t)dlsym(
            handle_p, (function_name + (std::string) "_release").c_str());
        if (dlerror()) dlerror();  // No such function, reset error flags

        /* Number of inputs */
        getint_t n_in_fcn = (getint_t)dlsym(
            handle_p, (function_name + (std::string) "_n_in").c_str());
        // if (dlerror()) return 1;
        this->n_in_ = n_in_fcn();

        /* Number of outputs */
        getint_t n_out_fcn = (getint_t)dlsym(
            handle_p, (function_name + (std::string) "_n_out").c_str());
        // if (dlerror()) return 1;
        casadi_int n_out = n_out_fcn();
        assert(n_out == 1);

        // Checkout thread-local memory (not thread-safe)
        // Note MAX_NUM_THREADS
        mem = checkout();

        /* Get sizes of the required work vectors */
        casadi_int sz_arg = this->n_in_, sz_res = n_out, sz_iw = 0, sz_w = 0;
        work_t work = (work_t)dlsym(
            handle_p, (function_name + (std::string) "_work").c_str());

        if (dlerror()) dlerror();  // No such function, reset error flags

        assert((work && work(&sz_arg, &sz_res, &sz_iw, &sz_w)) == 0);

        work_vector_d.resize(sz_w);
        work_vector_i.resize(sz_iw);
        res_vec.resize(sz_res);
        arg_vec.resize(sz_arg);
        w = work_vector_d.data();
        iw = work_vector_i.data();

        /* Input sparsities */

        /* Output sparsities */
        sparsity_t sp_out = (sparsity_t)dlsym(
            handle_p, (function_name + (std::string) "_sparsity_out").c_str());
        assert(dlerror() == 0);

        const casadi_int *sparsity_out_ci = sp_out(0);
        this->out_m = sparsity_out_ci[0];
        this->out_n = sparsity_out_ci[1];
        this->out_nnz = sparsity_out_ci[this->out_n + 2];

        this->sparsity_out.resize(2 + this->out_n + 1 + this->out_nnz, 0);
        this->sparsity_out.assign(
            sparsity_out_ci,
            sparsity_out_ci + 2 + this->out_n + 1 + this->out_nnz);

        /* Function for numerical evaluation */
        f = (eval_t)dlsym(handle_p, function_name.c_str());
        if (dlerror()) {
            printf("Failed to retrieve \"f\" function.\n");
        }

        this->buffer.resize(this->out_nnz, 0.0);
    }

    integer_type operator()(const value_type **arg, value_type *ret) override {
        w = work_vector_d.data();
        iw = work_vector_i.data();
        for (integer_type i = 0; i < this->n_in_; i++) {
            arg_vec[i] = arg[i];
        }
        // todo - currently only considering one output
        res_vec[0] = ret;
        if (f(arg_vec.data(), res_vec.data(), iw, w, mem))
            return integer_type(1);
        return integer_type(0);
    }

    integer_type info(evaluator_out_info &info) {
        info.m = this->out_m;
        info.n = this->out_n;
        info.nnz = this->out_nnz;
        info.sparsity_out = this->sparsity_out.data();
        info.type = evaluator_matrix_type::Sparse;
        return integer_type(0);
    }

    ~evaluator() {
        // Release thread-local (not thread-safe)
        release(mem);
        // Free memory (thread-safe)
        decref();
    }

   private:
    std::shared_ptr<dynamic_library_handler> handle_;
};

}  // namespace casadi
}  // namespace bopt
