#pragma once
#ifndef VISION_UTILS_IO_H
#define VISION_UTILS_IO_H

#include <string>
#include <nlohmann/json.hpp>

namespace vision::utils
{
  bool read_json(const std::string& path, nlohmann::json& data);
  bool write_json(const std::string& path, const nlohmann::json& data);
  bool ensureDirectoryExistsRecursive(std::string file_path);
}

#endif
