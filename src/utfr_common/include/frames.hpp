/*

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

*
* file: topics.hpp
* auth: Kelvin Cui
* desc: ros2 topics list
*/
#pragma once

// System Requirements
#include <string>

namespace utfr_dv {
namespace frames {

const std::string kMap{"map"};
const std::string kBase{"base_link"};
const std::string kCamera{"left_camera"};
const std::string kRightCamera{"right_camera"};
const std::string kLidar{"scan"};

} // namespace frames
} // namespace utfr_dv