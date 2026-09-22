#pragma once
#ifndef VISION_UTILS_TIMER_HPP
#define VISION_UTILS_TIMER_HPP

#include <chrono>
#include <string>

namespace vision::utils
{
  class Timer {
  public:
    Timer() { reset(); }
    void reset() { start_ = std::chrono::high_resolution_clock::now(); }
    double elapsed_ms() const {
      auto end = std::chrono::high_resolution_clock::now();
      return std::chrono::duration<double, std::milli>(end - start_).count();
    }
  private:
    std::chrono::high_resolution_clock::time_point start_;
  };
}

#endif
