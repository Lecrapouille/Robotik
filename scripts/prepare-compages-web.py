#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
# Copyright (c) 2020-2026 Quentin Quadrat
#
# Compages is cloned by `make download-external-libs`. A plain Emscripten
# toolchain (libc++ without a time-zone database, Clang 21) does not compile
# that tree as published. The edits below are applied in place, and skipped
# when a newer Compages already contains them.

import pathlib
import sys

root = pathlib.Path(sys.argv[1])


def replace_once(path: pathlib.Path, old: str, new: str, label: str) -> None:
    text = path.read_text()
    if new in text:
        return
    if old not in text:
        raise SystemExit(f"{label}: expected text not found in {path}")
    path.write_text(text.replace(old, new, 1))
    print(f"prepared {path.relative_to(root)} ({label})")


replace_once(
    root / "include/Compages/GPU/Core/Enums.hpp",
    "#include <cstdint>\n",
    "#include <cstddef>\n#include <cstdint>\n",
    "cstddef",
)

file_cpp = root / "src/Core/File.cpp"
file_text = file_cpp.read_text()
stamp_begin = file_text.find("std::string File::generateTempFileName")
stamp_end = file_text.find("\nbool File::mkdir", stamp_begin)
if stamp_begin < 0 or stamp_end < 0:
    raise SystemExit(f"utc timestamp: generateTempFileName not found in {file_cpp}")
if "gmtime_r" not in file_text[stamp_begin:stamp_end]:
    # ExaequOS ships an older libc++ that deletes the formatter of a time
    # point, and neither web libc++ has a time-zone database.
    file_text = (
        file_text[:stamp_begin]
        + """std::string File::generateTempFileName(std::string const& p_root_path,
                                       std::string const& p_extension)
{
#if defined(__EMSCRIPTEN__)
    std::time_t const now = std::time(nullptr);
    std::tm utc{};
    if (::gmtime_r(&now, &utc) == nullptr)
    {
        return p_root_path + p_extension;
    }
    char stamp[32];
    if (std::strftime(stamp, sizeof(stamp), "%Y-%m-%d/%Hh-%Mm-%Ss", &utc) == 0)
    {
        return p_root_path + p_extension;
    }
    return p_root_path + stamp + p_extension;
#else
    try
    {
        std::chrono::zoned_time const local{
            std::chrono::current_zone(),
            std::chrono::floor<std::chrono::seconds>(
                std::chrono::system_clock::now())
        };

        std::string path = p_root_path;
        path += std::format("{:%Y-%m-%d/}", local);
        path += std::format("{:%H}", local);
        path += 'h';
        path += std::format("-{:%M}", local);
        path += 'm';
        path += std::format("-{:%S}", local);
        path += 's';
        path += p_extension;
        return path;
    }
    catch (std::format_error const&)
    {
        return p_root_path + p_extension;
    }
    catch (std::runtime_error const&)
    {
        return p_root_path + p_extension;
    }
#endif
}
"""
        + file_text[stamp_end:]
    )
    if "#include <ctime>\n" not in file_text:
        file_text = file_text.replace(
            "#include <chrono>\n", "#include <chrono>\n#include <ctime>\n", 1
        )
    file_cpp.write_text(file_text)
    print(f"prepared {file_cpp.relative_to(root)} (utc timestamp)")

replace_once(
    root / "src/Renderer/Scene.cpp",
    """    m_sky["position"] = { { -1, -1, -1 }, { 1, -1, -1 }, { 1, 1, -1 },
                          { -1, 1, -1 },  { -1, -1, 1 }, { 1, -1, 1 },
                          { 1, 1, 1 },    { -1, 1, 1 } };
""",
    """    // The element type is written out: a bare braced list is ambiguous
    // between the int and the float slots under Clang.
    m_sky["position"] = std::initializer_list<std::initializer_list<float>>{
        { -1.f, -1.f, -1.f }, { 1.f, -1.f, -1.f }, { 1.f, 1.f, -1.f },
        { -1.f, 1.f, -1.f },  { -1.f, -1.f, 1.f }, { 1.f, -1.f, 1.f },
        { 1.f, 1.f, 1.f },    { -1.f, 1.f, 1.f }
    };
""",
    "skybox vertices",
)

scene = root / "src/Renderer/Scene.cpp"
scene_text = scene.read_text()
if "#include <initializer_list>" not in scene_text:
    needle = "#include <filesystem>\n"
    if needle not in scene_text:
        raise SystemExit(f"initializer_list: {scene} has no filesystem include")
    scene.write_text(
        scene_text.replace(needle, needle + "#include <initializer_list>\n", 1)
    )
    print(f"prepared {scene.relative_to(root)} (initializer_list)")

# RobotIK names units compages::si::* and calls Quat::fromRpy and yaw. Those
# live in a Compages commit that is not on the published branch yet, so a CI
# clone does not have them.
units = root / "include/Compages/Core/Units.hpp"
units_text = units.read_text()
if "namespace compages::si" not in units_text:
    units.write_text(
        units_text
        + """
namespace compages::si
{
using Seconds = ::units::time::second_t;
using Radians = ::units::angle::radian_t;
using Length = ::units::length::meter_t;
using LinearVelocity = ::units::velocity::meters_per_second_t;
using AngularVelocity = ::units::angular_velocity::radians_per_second_t;
using AngularAcceleration =
    ::units::angular_acceleration::radians_per_second_squared_t;
using Force = ::units::force::newton_t;
using Torque = ::units::torque::newton_meter_t;
} // namespace compages::si
"""
    )
    print(f"prepared {units.relative_to(root)} (si units)")

quaternion = root / "include/Compages/Core/Quaternion.hpp"
quaternion_text = quaternion.read_text()
changed = False
from_rpy = """
    static Quat<T> fromRpy(units::angle::radian_t const roll,
                           units::angle::radian_t const pitch,
                           units::angle::radian_t const yaw)
    {
        Quat<T> const rx = rotation(roll, T(1), T(0), T(0));
        Quat<T> const ry = rotation(pitch, T(0), T(1), T(0));
        Quat<T> const rz = rotation(yaw, T(0), T(0), T(1));
        return rz * ry * rx;
    }

"""
anchor = """        q.normalize();
        return q;
    }

    //--------------------------------------------------------------------------
    //! \\brief Construct a quaternion from an angle/axis
"""
if "fromRpy(" not in quaternion_text:
    if anchor not in quaternion_text:
        raise SystemExit(f"fromRpy: expected text not found in {quaternion}")
    quaternion_text = quaternion_text.replace(anchor, anchor.replace(
        "    //--------------------------------------------------------------------------\n    //! \\brief Construct a quaternion from an angle/axis\n",
        from_rpy + "    //--------------------------------------------------------------------------\n    //! \\brief Construct a quaternion from an angle/axis\n",
        1,
    ), 1)
    changed = True

if "radian_t yaw(" not in quaternion_text:
    closing = "} // namespace compages::core\n"
    if closing not in quaternion_text:
        raise SystemExit(f"yaw: expected text not found in {quaternion}")
    quaternion_text = quaternion_text.replace(
        closing,
        """template <typename T>
units::angle::radian_t yaw(Quat<T> const& q)
{
    T const siny = T(2) * (q.w * q.z + q.x * q.y);
    T const cosy = T(1) - T(2) * (q.y * q.y + q.z * q.z);
    return units::angle::radian_t(std::atan2(siny, cosy));
}

} // namespace compages::core
""",
        1,
    )
    changed = True
if changed:
    quaternion.write_text(quaternion_text)
    print(f"prepared {quaternion.relative_to(root)} (robot helpers)")

hedley = root / "external/json/include/nlohmann/thirdparty/hedley/hedley.hpp"
hedley_text = hedley.read_text()
broken = (
    "JSON_HEDLEY_NO_RETURN "
    "JSON_HEDLEY_DIAGNOSTIC_DISABLE_CPP98_COMPAT_WRAP_([[noreturn]])"
)
if "JSON_HEDLEY_NO_RETURN [[noreturn]]" in hedley_text:
    pass
elif broken in hedley_text:
    hedley.write_text(hedley_text.replace(broken, "JSON_HEDLEY_NO_RETURN [[noreturn]]", 1))
    print(f"prepared {hedley.relative_to(root)} (hedley noreturn)")
else:
    raise SystemExit(f"hedley noreturn: expected text not found in {hedley}")

# Emscripten is Clang. The shared makefile still passes GCC flags, and a
# float stored in a double unit warns on every file that includes it.
rules = root / ".makefile/rules/Makefile"
rules_text = rules.read_text()
clash = "    HARDENING_FLAGS += -fstack-clash-protection\n"
clash_guard = (
    "    ifneq ($(OS),Emscripten)\n"
    "    HARDENING_FLAGS += -fstack-clash-protection\n"
    "    endif\n"
)
if clash_guard not in rules_text:
    if clash not in rules_text:
        raise SystemExit(f"stack clash: expected text not found in {rules}")
    rules_text = rules_text.replace(clash, clash_guard, 1)
    print(f"prepared {rules.relative_to(root)} (stack clash)")

quiet = """
ifeq ($(OS),Emscripten)
    CXXFLAGS += -Wno-unknown-warning-option -Wno-unused-command-line-argument
    CXXFLAGS += -Wno-double-promotion -Wno-undefined-var-template
endif

"""
c_flags = (
    "###############################################################################\n"
    "# C specific compilation flags\n"
)
if "-Wno-undefined-var-template" not in rules_text:
    if c_flags not in rules_text:
        raise SystemExit(f"emscripten warnings: expected text not found in {rules}")
    rules_text = rules_text.replace(c_flags, quiet + c_flags, 1)
    print(f"prepared {rules.relative_to(root)} (emscripten warnings)")
rules.write_text(rules_text)

replace_once(
    root / "src/GPU/Backends/GL45/glad/GladUnit.cpp",
    '#pragma GCC diagnostic ignored "-Wuseless-cast"\n',
    '#pragma GCC diagnostic ignored "-Wold-style-cast"\n'
    '#pragma GCC diagnostic ignored "-Wuseless-cast"\n',
    "glad casts",
)

replace_once(
    root / "src/Renderer/Assets/CgltfInclude.hpp",
    '#elif defined(__clang__)\n'
    '#    pragma clang diagnostic push\n'
    '#    pragma clang diagnostic ignored "-Warith-conversion"\n',
    '#elif defined(__clang__)\n'
    '#    pragma clang diagnostic push\n'
    '#    pragma clang diagnostic ignored "-Wcast-align"\n',
    "cgltf alignment",
)
