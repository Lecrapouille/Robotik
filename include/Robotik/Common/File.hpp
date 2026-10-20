// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file File.hpp
/// Small questions about files on disk: is it a folder, is it a file, where
/// is the running program.
///
/// They never throw, unlike some @c std::filesystem calls: a missing file or a
/// denied access simply answers @c false.
///
/// @par Contract
/// - Promises: no function throws; a path that does not exist gives @c false.

#pragma once

#include <filesystem>

namespace robotik
{

// -------------------------------------------------------------------------
/// @param p_path Path to test.
/// @return True when @p_path is a directory (errors are treated as false).
// -------------------------------------------------------------------------
[[nodiscard]] bool isDirectory(std::filesystem::path const& p_path);

// -------------------------------------------------------------------------
/// @param p_path Path to test.
/// @return True when @p_path is a regular file (errors are treated as false).
// -------------------------------------------------------------------------
[[nodiscard]] bool isRegularFile(std::filesystem::path const& p_path);

// -------------------------------------------------------------------------
/// @param p_path Path to normalize.
/// @return Canonical path, or the normalized absolute path on failure.
// -------------------------------------------------------------------------
[[nodiscard]] std::filesystem::path
canonicalPath(std::filesystem::path const& p_path);

// -------------------------------------------------------------------------
/// @param p_root Folder that must contain @p_file.
/// @param p_file Path to test, usually a file under @p_root.
/// @return True when @p_file stays below @p_root once both are made canonical.
// -------------------------------------------------------------------------
[[nodiscard]] bool pathInsideRoot(std::filesystem::path const& p_root,
                                  std::filesystem::path const& p_file);

// ----------------------------------------------------------------------------
/// Folder of the running executable (Linux @c /proc/self/exe). Search roots
/// hang from it so that no lookup depends on the current directory.
/// @return The folder, or empty when it cannot be read.
// ----------------------------------------------------------------------------
[[nodiscard]] std::filesystem::path executableDirectory();

} // namespace robotik
