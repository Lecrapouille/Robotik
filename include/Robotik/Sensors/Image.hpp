// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

//! @file Image.hpp
//! @brief CPU image: one tightly packed buffer, top-left origin.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace robotik
{

// ****************************************************************************
//! @brief Pixel layouts handled by Robotik.
// ****************************************************************************
enum class PixelFormat : std::uint8_t
{
    Gray8,    //!< One byte per pixel.
    RGB8,     //!< Three bytes per pixel, R first.
    RGBA8,    //!< Four bytes per pixel.
    Depth32F, //!< One float per pixel: distance along the optical axis (m).
};

[[nodiscard]] constexpr std::size_t bytesPerPixel(PixelFormat p_format)
{
    switch (p_format)
    {
        case PixelFormat::Gray8:
            return 1u;
        case PixelFormat::RGB8:
            return 3u;
        case PixelFormat::RGBA8:
        case PixelFormat::Depth32F:
            return 4u;
    }
    return 1u;
}

// ****************************************************************************
//! @brief Owning image with rows stored top to bottom and no padding.
//!
//! @ref resize keeps the allocation when the size does not grow, so a camera
//! reuses the same memory frame after frame. Third-party libraries can wrap
//! the buffer without a copy, e.g. in a demo:
//! @code
//! cv::Mat view(int(image.height()), int(image.width()), CV_8UC3, image.data());
//! @endcode
// ****************************************************************************
class Image
{
public:

    Image() = default;

    Image(std::uint32_t p_width, std::uint32_t p_height, PixelFormat p_format)
    {
        resize(p_width, p_height, p_format);
    }

    // -------------------------------------------------------------------------
    //! @brief Changes the geometry; contents are unspecified afterwards.
    // -------------------------------------------------------------------------
    void resize(std::uint32_t p_width,
                std::uint32_t p_height,
                PixelFormat p_format)
    {
        m_width = p_width;
        m_height = p_height;
        m_format = p_format;
        m_pixels.resize(std::size_t(p_width) * p_height *
                        bytesPerPixel(p_format));
    }

    //! @brief Releases the pixels (an empty image means "no data").
    void clear()
    {
        m_width = 0;
        m_height = 0;
        m_pixels.clear();
    }

    [[nodiscard]] bool empty() const
    {
        return m_pixels.empty();
    }

    [[nodiscard]] std::uint32_t width() const
    {
        return m_width;
    }

    [[nodiscard]] std::uint32_t height() const
    {
        return m_height;
    }

    [[nodiscard]] PixelFormat format() const
    {
        return m_format;
    }

    //! @brief Bytes per row.
    [[nodiscard]] std::size_t stride() const
    {
        return std::size_t(m_width) * bytesPerPixel(m_format);
    }

    [[nodiscard]] std::uint8_t* data()
    {
        return m_pixels.data();
    }

    [[nodiscard]] std::uint8_t const* data() const
    {
        return m_pixels.data();
    }

    [[nodiscard]] std::span<std::uint8_t> bytes()
    {
        return m_pixels;
    }

    [[nodiscard]] std::span<std::uint8_t const> bytes() const
    {
        return m_pixels;
    }

    // -------------------------------------------------------------------------
    //! @brief Typed pointer on row @p_y (e.g. @c row<float>(y) for depth).
    // -------------------------------------------------------------------------
    template <typename T = std::uint8_t>
    [[nodiscard]] T* row(std::uint32_t p_y)
    {
        return reinterpret_cast<T*>(m_pixels.data() + p_y * stride());
    }

    template <typename T = std::uint8_t>
    [[nodiscard]] T const* row(std::uint32_t p_y) const
    {
        return reinterpret_cast<T const*>(m_pixels.data() + p_y * stride());
    }

private:

    std::vector<std::uint8_t> m_pixels;
    std::uint32_t m_width = 0;
    std::uint32_t m_height = 0;
    PixelFormat m_format = PixelFormat::RGB8;
};

} // namespace robotik
