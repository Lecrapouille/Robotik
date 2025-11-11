/**
 * @file Return.hpp
 * @brief Return type template for error handling similar to std::expected.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include <stdexcept>
#include <string>
#include <utility>

namespace robotik
{

// ****************************************************************************
//! \brief Return type template that holds either a success value or an error.
//! Inspired by Rust's Result<T, E> and C++23's std::expected<T, E>.
//!
//! This class provides type-safe error handling without exceptions or global
//! error states. It forces the caller to check for errors before accessing
//! the value.
//!
//! Example usage:
//! \code
//!   Return<int> divide(int a, int b) {
//!       if (b == 0)
//!           return Return<int>::error("Division by zero");
//!       return Return<int>::success(a / b);
//!   }
//!
//!   auto result = divide(10, 2);
//!   if (result) {
//!       std::cout << "Result: " << result.getValue() << std::endl;
//!   } else {
//!       std::cerr << "Error: " << result.getError() << std::endl;
//!   }
//! \endcode
// ****************************************************************************
template <typename T>
class Return
{
public:

    // ------------------------------------------------------------------------
    //! \brief Create a successful Return with a value.
    //! \param[in] p_value The success value.
    //! \return A Return containing the success value.
    // ------------------------------------------------------------------------
    static Return success(T&& p_value)
    {
        Return result;
        result.m_has_value = true;
        result.m_value = std::move(p_value);
        return result;
    }

    // ------------------------------------------------------------------------
    //! \brief Create a successful Return with a value (copy version).
    //! \param[in] p_value The success value.
    //! \return A Return containing the success value.
    // ------------------------------------------------------------------------
    static Return success(T const& p_value)
    {
        Return result;
        result.m_has_value = true;
        result.m_value = p_value;
        return result;
    }

    // ------------------------------------------------------------------------
    //! \brief Create a failed Return with an error message.
    //! \param[in] p_error The error message.
    //! \return A Return containing the error message.
    // ------------------------------------------------------------------------
    static Return error(std::string const& p_error)
    {
        Return result;
        result.m_has_value = false;
        result.m_error = p_error;
        return result;
    }

    // ------------------------------------------------------------------------
    //! \brief Default constructor creates a failed Return.
    // ------------------------------------------------------------------------
    Return() : m_has_value(false), m_error("Uninitialized Return") {}

    // ------------------------------------------------------------------------
    //! \brief Check if the Return contains a success value.
    //! \return True if successful, false if error.
    // ------------------------------------------------------------------------
    [[nodiscard]] bool isSuccess() const
    {
        return m_has_value;
    }

    // ------------------------------------------------------------------------
    //! \brief Check if the Return contains an error.
    //! \return True if error, false if successful.
    // ------------------------------------------------------------------------
    [[nodiscard]] bool isError() const
    {
        return !m_has_value;
    }

    // ------------------------------------------------------------------------
    //! \brief Bool conversion operator for convenient checking.
    //! \return True if successful, false if error.
    // ------------------------------------------------------------------------
    [[nodiscard]] explicit operator bool() const
    {
        return m_has_value;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the success value.
    //! \return Reference to the success value.
    //! \throw std::runtime_error if called on an error Return.
    // ------------------------------------------------------------------------
    [[nodiscard]] T& getValue()
    {
        if (!m_has_value)
        {
            throw std::runtime_error("Return::getValue() called on error: " +
                                     m_error);
        }
        return m_value;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the success value (const version).
    //! \return Const reference to the success value.
    //! \throw std::runtime_error if called on an error Return.
    // ------------------------------------------------------------------------
    [[nodiscard]] T const& getValue() const
    {
        if (!m_has_value)
        {
            throw std::runtime_error("Return::getValue() called on error: " +
                                     m_error);
        }
        return m_value;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the error message.
    //! \return The error message.
    //! \note Returns empty string if called on a success Return.
    // ------------------------------------------------------------------------
    [[nodiscard]] std::string const& getError() const
    {
        return m_error;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the value or a default if error.
    //! \param[in] p_default The default value to return on error.
    //! \return The value if success, or the default if error.
    // ------------------------------------------------------------------------
    [[nodiscard]] T valueOr(T const& p_default) const
    {
        return m_has_value ? m_value : p_default;
    }

    // ------------------------------------------------------------------------
    //! \brief Move the value out of the Return.
    //! \return The moved value.
    //! \throw std::runtime_error if called on an error Return.
    // ------------------------------------------------------------------------
    [[nodiscard]] T&& moveValue()
    {
        if (!m_has_value)
        {
            throw std::runtime_error("Return::moveValue() called on error: " +
                                     m_error);
        }
        return std::move(m_value);
    }

private:

    bool m_has_value;
    T m_value;
    std::string m_error;
};

} // namespace robotik
