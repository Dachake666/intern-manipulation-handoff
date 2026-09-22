#pragma once
#ifndef VISION_UTILS_STRINGX_H
#define VISION_UTILS_STRINGX_H

#include <string>
#include <vector>

namespace vision::utils
{
  std::vector<std::string> split(const std::string& str, const std::string& delim);
  std::string trim(const std::string& str);
}

#endif
