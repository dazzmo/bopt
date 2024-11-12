#pragma once

#include <dlfcn.h>

#include <iostream>
#include <string>

namespace bopt {
class dynamic_library_handler {
   public:
    dynamic_library_handler(const std::string& filename) {
        handle = dlopen(&filename[0], RTLD_LAZY);
        if (handle == 0) {
            std::cout << "Cannot open dynamic library or shared object "
                      << filename << ": " << dlerror() << '\n';
        }
    }

    ~dynamic_library_handler() { dlclose(handle); }

    void* handle;

   private:
};
}  // namespace bopt
