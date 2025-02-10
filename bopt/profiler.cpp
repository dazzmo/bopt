#include "bopt/profiler.hpp"

namespace bopt {

using namespace boost::accumulators;

profiler::profiler() {
#if defined(BOPT_USE_PROFILING)
    std::cout << "Calls\tMean (secs)\tStdDev\tMin (sec)\tMax (secs)\n";
    for (std::map<std::string, acc_t>::iterator p = map_.begin();
         p != map_.end(); p++) {
        double av = mean(p->second);
        double stdev = sqrt(((double)variance(p->second)));
        double max = boost::accumulators::extract::max(p->second);
        double min = boost::accumulators::extract::min(p->second);
        std::cout << p->first.c_str() << '\t'
                  << boost::accumulators::count(p->second) << '\t' << av << '\t'
                  << stdev << '\t' << min << '\t' << max << '\n';
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
    const std::chrono::duration<double> dur = clock::now() - start_;
    std::map<std::string, acc_t>::iterator p = map_.find(name_);
    if (p == map_.end()) {
        // Create new accumulator
        acc_t acc;
        std::pair<std::string, acc_t> pr(name_, acc);
        p = map_.insert(pr).first;
    }
    // TODO Check what the real time is (make it in seconds)
    (p->second)(dur.count());
#endif
}

}  // namespace bopt