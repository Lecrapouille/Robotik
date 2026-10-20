// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Image.hpp
/// An image in memory: one buffer of pixels, the first pixel at the top left.
///
/// It is the only image type of RobotIK, so detectors do not depend on OpenGL,
/// OpenCV or a camera driver.
///
/// @par Contract
/// - Promises: one packed buffer, rows from top to bottom, no padding.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace robotik
{

// ****************************************************************************
/// Pixel layouts handled by Robotik.
// ****************************************************************************
enum class PixelFormat : std::uint8_t
{
    Gray8,    ///< One byte per pixel.
    RGB8,     ///< Three bytes per pixel, R first.
    RGBA8,    ///< Four bytes per pixel.
    Depth32F, ///< One float per pixel: distance along the optical axis (m).
};

// ----------------------------------------------------------------------------
/// Size of one pixel.
/// @param p_format Pixel layout.
/// @return The number of bytes of one pixel.
// ----------------------------------------------------------------------------
[[nodiscard]] constexpr std::size_t bytesPerPixel(PixelFormat p_format)
{
    switch (p_format)
    {
        case PixelFormat::Gray8:
            return 1U;
        case PixelFormat::RGB8:
            return 3U;
        case PixelFormat::RGBA8:
        case PixelFormat::Depth32F:
            return 4U;
    }
    return 1U;
}

// ****************************************************************************
/// Owning image with rows stored top to bottom and no padding.
///
/// @ref resize keeps the allocation when the size does not grow, so a camera
/// reuses the same memory frame after frame. Third-party libraries can wrap
/// the buffer without a copy, e.g. in a demo:
/// @code
/// cv::Mat view(int(image.height()), int(image.width()), CV_8UC3,
/// image.data());
/// @endcode
///
/// Reading a pixel:
/// @code
/// robotik::Image image(640, 480, robotik::PixelFormat::RGB8);
/// std::uint8_t const* p = image.row(y) + 3 * x;
/// std::uint8_t red = p[0], green = p[1], blue = p[2];
/// @endcode
// ****************************************************************************
class Image
{
public:

    /// Empty image ("no data").
    Image() = default;

    // -------------------------------------------------------------------------
    /// @param p_width Width in pixels.
    /// @param p_height Height in pixels.
    /// @param p_format Pixel layout.
    // -------------------------------------------------------------------------
    Image(std::uint32_t p_width, std::uint32_t p_height, PixelFormat p_format)
    {
        resize(p_width, p_height, p_format);
    }

    // -------------------------------------------------------------------------
    /// Changes the geometry; contents are unspecified afterwards.
    /// @param p_width Width in pixels.
    /// @param p_height Height in pixels.
    /// @param p_format Pixel layout.
    // -------------------------------------------------------------------------
    void
    resize(std::uint32_t p_width, std::uint32_t p_height, PixelFormat p_format)
    {
        m_width = p_width;
        m_height = p_height;
        m_format = p_format;
        m_pixels.resize(std::size_t(p_width) * p_height *
                        bytesPerPixel(p_format));
    }

    // -------------------------------------------------------------------------
    /// Releases the pixels (an empty image means "no data").
    // -------------------------------------------------------------------------
    void clear()
    {
        m_width = 0;
        m_height = 0;
        m_pixels.clear();
    }

    /// @return True when the image holds no pixel.
    [[nodiscard]] bool empty() const
    {
        return m_pixels.empty();
    }

    /// @return Width in pixels.
    [[nodiscard]] std::uint32_t width() const
    {
        return m_width;
    }

    /// @return Height in pixels.
    [[nodiscard]] std::uint32_t height() const
    {
        return m_height;
    }

    /// @return Pixel layout.
    [[nodiscard]] PixelFormat format() const
    {
        return m_format;
    }

    /// @return Bytes per row.
    [[nodiscard]] std::size_t stride() const
    {
        return std::size_t(m_width) * bytesPerPixel(m_format);
    }

    /// @return The first byte of the top-left pixel.
    [[nodiscard]] std::uint8_t* data()
    {
        return m_pixels.data();
    }

    /// @copydoc data()
    [[nodiscard]] std::uint8_t const* data() const
    {
        return m_pixels.data();
    }

    /// @return Every byte of the image, row after row.
    [[nodiscard]] std::span<std::uint8_t> bytes()
    {
        return m_pixels;
    }

    /// @copydoc bytes()
    [[nodiscard]] std::span<std::uint8_t const> bytes() const
    {
        return m_pixels;
    }

    // -------------------------------------------------------------------------
    /// Typed pointer on one row (e.g. @c row<float>(y) for depth).
    /// @tparam T Type of one channel.
    /// @param p_y Row index, 0 at the top.
    /// @return The first channel of the first pixel of the row.
    // -------------------------------------------------------------------------
    template <typename T = std::uint8_t>
    [[nodiscard]] T* row(std::uint32_t p_y)
    {
        return reinterpret_cast<T*>(m_pixels.data() + p_y * stride());
    }

    /// @copydoc row(std::uint32_t)
    template <typename T = std::uint8_t>
    [[nodiscard]] T const* row(std::uint32_t p_y) const
    {
        return reinterpret_cast<T const*>(m_pixels.data() + p_y * stride());
    }

private:

    /// Pixels, row after row.
    std::vector<std::uint8_t> m_pixels;
    /// Width in pixels.
    std::uint32_t m_width = 0;
    /// Height in pixels.
    std::uint32_t m_height = 0;
    /// Pixel layout.
    PixelFormat m_format = PixelFormat::RGB8;
};

} // namespace robotik
