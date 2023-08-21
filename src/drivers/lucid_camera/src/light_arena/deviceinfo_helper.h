#pragma once
#include "ArenaApi.h"
#include <memory>
#include <vector>

class DeviceInfoHelper {
public:
  static size_t get_index_of_serial(std::vector<Arena::DeviceInfo> device_infos,
                                    std::string serial);
  static std::string info(Arena::DeviceInfo device_info);
};
