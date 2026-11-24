#pragma once

#include <array>

namespace m20 {

// 关节名称
inline constexpr std::array<const char *, 16> joint_names = {
    "fl_hipx", "fl_hipy", "fl_knee", "fl_wheel",
    "fr_hipx", "fr_hipy", "fr_knee", "fr_wheel",
    "hl_hipx", "hl_hipy", "hl_knee", "hl_wheel",
    "hr_hipx", "hr_hipy", "hr_knee", "hr_wheel",
};

// 关节固件原始方向和 SDK 方向是同向还是反向
inline constexpr std::array<float, 16> joint_direction = {
    1, 1, -1, 1, 1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1,
};

}
