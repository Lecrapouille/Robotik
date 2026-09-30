// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#pragma once

//! \brief Make a derived class non copyable.
class NonCopyable
{
protected:

    constexpr NonCopyable() = default;
    ~NonCopyable() = default;

    NonCopyable(const NonCopyable&) = delete;
    const NonCopyable& operator=(const NonCopyable&) = delete;
};

//! \brief Make a derived class non creatable.
class NonCreatable
{
protected:

    constexpr NonCreatable() = default;
    ~NonCreatable() = default;

    NonCreatable(const NonCreatable&) = delete;
    const NonCreatable& operator=(const NonCreatable&) = delete;
};

//! \brief Singleton pattern implementation.
//! \tparam T Curiously recurring template pattern of the derived class.
template <class T>
class Singleton: public NonCopyable, public NonCreatable
{
public:

    static T& instance()
    {
        static T instance;
        return instance;
    }
};
