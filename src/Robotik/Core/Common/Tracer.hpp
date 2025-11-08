/**
 * @file Tracer.hpp
 * @brief Tracer for the Robotik library.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#ifdef TRACER_ENABLED
#    define TRACY_ENABLE
#    define TRACY_CALLSTACK 10
#    define TRACY_CALLSTACK_DEPTH 10
#    include <tracy/Tracy.hpp>
#else
#    define ZoneScoped
#    define ZoneScopedN(name)
#    define ZoneScopedC(category)
#    define ZoneScopedCS(category, name)
#    define ZoneScopedCSN(category, name)
#    define ZoneScopedCSNC(category, name)
#    define ZoneScopedCSN(category, name)
#    define ZoneScopedCSN(category, name)
#endif