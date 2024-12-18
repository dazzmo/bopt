#ifndef CORE_PROFILER_HPP
#define CORE_PROFILER_HPP

#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <string>

#include "bopt/logging.hpp"

namespace bopt {

using namespace boost::accumulators;

/**
 * @brief Profiling class that enables the timing of a specific component.
 * Records the time elapsed over the lifetime of the object and stores the
 * results in a global profiler that can be evaluated using the emptry
 * constructor.
 *
 */
class profiler {
 public:
  typedef std::chrono::steady_clock clock;
  typedef accumulator_set<double,
                          stats<tag::mean, tag::variance, tag::min, tag::max>>
      acc_t;
  /**
   * @brief Generates a report for all profilers
   *
   */
  profiler();

  /**
   * @brief Create a new scoped profiler
   *
   * @param name
   */
  profiler(const char* name);

  ~profiler();

 private:
  std::string name_;
  std::chrono::steady_clock::time_point start_;

  // Static map
  inline static std::map<std::string, acc_t> map_;
};

}  // namespace bopt

#endif /* CORE_PROFILER_HPP */
