/**
 * @file Benchmark.hpp
 * @brief Simple benchmarking and profiling utilities.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

namespace robotik
{

// ****************************************************************************
//! \brief Simple timer for benchmarking.
// ****************************************************************************
class Timer
{
public:

    Timer() : m_start(std::chrono::high_resolution_clock::now()) {}

    void restart()
    {
        m_start = std::chrono::high_resolution_clock::now();
    }

    double elapsed_ms() const
    {
        auto now = std::chrono::high_resolution_clock::now();
        return std::chrono::duration<double, std::milli>(now - m_start).count();
    }

    double elapsed_us() const
    {
        auto now = std::chrono::high_resolution_clock::now();
        return std::chrono::duration<double, std::micro>(now - m_start).count();
    }

private:

    std::chrono::high_resolution_clock::time_point m_start;
};

// ****************************************************************************
//! \brief RAII profiler that automatically measures execution time.
// ****************************************************************************
class ScopedProfiler
{
public:

    ScopedProfiler(const std::string& name, double* output = nullptr)
        : m_name(name), m_output(output), m_timer()
    {
    }

    ~ScopedProfiler()
    {
        double elapsed = m_timer.elapsed_ms();
        if (m_output != nullptr)
        {
            *m_output = elapsed;
        }
        else
        {
            std::cout << "[PROFILE] " << m_name << ": " << std::fixed
                      << std::setprecision(3) << elapsed << "ms" << std::endl;
        }
    }

private:

    std::string m_name;
    double* m_output;
    Timer m_timer;
};

// ****************************************************************************
//! \brief Performance statistics collector.
// ****************************************************************************
class PerformanceStats
{
public:

    void add_sample(const std::string& name, double value_ms)
    {
        m_samples[name].push_back(value_ms);

        // Keep only last 1000 samples to avoid memory growth
        if (m_samples[name].size() > 1000)
        {
            m_samples[name].erase(m_samples[name].begin());
        }
    }

    void print_stats(const std::string& name) const
    {
        auto it = m_samples.find(name);
        if (it == m_samples.end() || it->second.empty())
        {
            std::cout << "[STATS] " << name << ": No data" << std::endl;
            return;
        }

        const auto& samples = it->second;
        double sum = 0.0;
        double min_val = samples[0];
        double max_val = samples[0];

        for (double val : samples)
        {
            sum += val;
            min_val = std::min(min_val, val);
            max_val = std::max(max_val, val);
        }

        double avg = sum / static_cast<double>(samples.size());

        std::cout << "[STATS] " << name << ": "
                  << "avg=" << std::fixed << std::setprecision(3) << avg
                  << "ms, "
                  << "min=" << min_val << "ms, "
                  << "max=" << max_val << "ms, "
                  << "samples=" << samples.size() << std::endl;
    }

    void print_all_stats() const
    {
        std::cout << "\n=== PERFORMANCE STATISTICS ===" << std::endl;
        for (const auto& pair : m_samples)
        {
            print_stats(pair.first);
        }
        std::cout << "===============================\n" << std::endl;
    }

    void clear()
    {
        m_samples.clear();
    }

private:

    std::unordered_map<std::string, std::vector<double>> m_samples;
};

// Global performance stats instance
extern PerformanceStats g_perf_stats;

// Macro for easy profiling
#define PROFILE_SCOPE(name) ScopedProfiler _prof_##__LINE__(name)
#define PROFILE_SCOPE_VAR(name, var) ScopedProfiler _prof_##__LINE__(name, &var)

} // namespace robotik
