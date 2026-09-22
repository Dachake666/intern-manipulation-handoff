#include "vision_utils/io.h"
#include "vision_utils/vision_log.h"
#include <fstream>
#include <iostream>

namespace vision::utils
{
  bool read_json(const std::string& path, nlohmann::json& data)
  {
    std::ifstream f(path);
    if (f.fail())
    {
      std::cout<<"json_open [{}] failed!"<< path<<std::endl;
      return false;
    }
    try
    {
      data = nlohmann::json::parse(f, nullptr, true, true);
    }
    catch (const nlohmann::json::exception& e)
    {
      std::cout<<"parse json failed: "<< e.what()<<std::endl;
      return false;
    }
    return true;
  }

  bool write_json(const std::string& path, const nlohmann::json& data)
  {
    std::ofstream f(path);
    f << std::setw(2) << data << std::endl;
    return true;
  }
  bool ensureDirectoryExistsRecursive(std::string file_path) {
    if (access(file_path.c_str(), F_OK) != 0) {
        mode_t mode = 0755;
        std::string cmd = "mkdir -p " + std::string(file_path);
        int result = system(cmd.c_str());
    }
    return 1;
  }

}
