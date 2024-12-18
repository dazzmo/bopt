#include "bopt/profiler.hpp"

namespace bopt {

using namespace boost::accumulators;

profiler::profiler() {
#if defined(BOPT_USE_PROFILING)
  LOG(INFO) << "Calls\tMean (secs)\tStdDev\tMin (sec)\tMax (secs)\n";
  for (std::map<std::string, acc_t>::iterator p = map_.begin(); p != map_.end();
       p++) {
    double av = mean(p->second);
    double stdev = sqrt(((double)variance(p->second)));
    double max = boost::accumulators::extract::max(p->second);
    double min = boost::accumulators::extract::min(p->second);
    LOG(INFO) << p->first.c_str() << '\t'
              << boost::accumulators::count(p->second) << '\t' << av << '\t'
              << stdev << '\t' << min << '\t' << max;
  }
#endif
}

profiler::profiler(const char* name) : name_(name) {
#if defined(BOPT_USE_PROFILING)
  // Record start time
  start_ = clock::now();
#endif
}

// todo - write to file

profiler::~profiler() {
#if defined(BOPT_USE_PROFILING)
  auto dur = clock::now() - start_;
  std::map<std::string, acc_t>::iterator p = map_.find(name_);
  if (p == map_.end()) {
    // Create new accumulator
    acc_t acc;
    std::pair<std::string, acc_t> pr(name_, acc);
    p = map_.insert(pr).first;
  }
  VLOG(10) << "Adding to accumulator";
  // TODO Check what the real time is (make it in seconds)
  (p->second)(dur.count() * 1e-9);
#endif
}

}  // namespace bopt