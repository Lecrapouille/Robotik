// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Browser.hpp
/// What the web build does with a file the desktop would leave on disk.

#pragma once

#include <string>

#if defined(__EMSCRIPTEN__) && !defined(EXAEQUOS)
#    include <emscripten.h>
#endif

namespace robotik
{

/// Web version: a file written lives in the memory of the page, so the
/// browser downloads it. Nothing to do on a desktop or inside ExaequOS.
inline void download(std::string const& p_path)
{
#if defined(__EMSCRIPTEN__) && !defined(EXAEQUOS)
    // $0 is how EM_ASM names its first argument, not a C++ identifier.
#    pragma clang diagnostic push
#    pragma clang diagnostic ignored "-Wdollar-in-identifier-extension"
    EM_ASM(
        {
            var path = UTF8ToString($0);
            var blob = new Blob([FS.readFile(path)],
                                { type: 'application/octet-stream' });
            var link = document.createElement('a');
            link.href = URL.createObjectURL(blob);
            link.download = path.split('/').pop();
            document.body.appendChild(link);
            link.click();
            link.remove();
            setTimeout(function() { URL.revokeObjectURL(link.href); }, 1000);
        },
        p_path.c_str());
#    pragma clang diagnostic pop
#else
    (void)p_path;
#endif
}

} // namespace robotik
