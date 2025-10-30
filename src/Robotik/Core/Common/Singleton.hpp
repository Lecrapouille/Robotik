/**
 * @file Singleton.hpp
 * @brief Singleton pattern implementation.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

//******************************************************************************
//! \brief Make a derived class non copyable.
//******************************************************************************
class NonCopyable
{
protected:

    constexpr NonCopyable() = default;
    ~NonCopyable() = default;

    NonCopyable(const NonCopyable&) = delete;
    const NonCopyable& operator=(const NonCopyable&) = delete;
};

//******************************************************************************
//! \brief Make a derived class non creatable.
//******************************************************************************
class NonCreatable
{
protected:

    constexpr NonCreatable() = default;
    ~NonCreatable() = default;

    NonCreatable(const NonCreatable&) = delete;
    const NonCreatable& operator=(const NonCreatable&) = delete;
};

//******************************************************************************
//! \brief Singleton pattern implementation.
//! \tparam T Curiously recurring template pattern of the derived class.
//******************************************************************************
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