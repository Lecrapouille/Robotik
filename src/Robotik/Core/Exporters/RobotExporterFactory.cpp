/**
 * @file RobotExporterFactory.cpp
 * @brief Implementation of robot exporter factory.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/Exporters/RobotExporterFactory.hpp"
#include "Robotik/Core/Exporters/DotExporter.hpp"

#include <algorithm>
#include <cctype>

namespace robotik
{

// ----------------------------------------------------------------------------
std::unordered_map<std::string, RobotExporterFactory::ExporterCreator>&
RobotExporterFactory::getRegistry()
{
    static std::unordered_map<std::string, ExporterCreator> registry = {
        { ".dot",
          []() -> std::unique_ptr<RobotExporter>
          { return std::make_unique<DotExporter>(); } }

        // Future exporters can be registered here:
        // {".urdf", []() -> std::unique_ptr<RobotExporter>
        // { return std::make_unique<UrdfExporter>(); }},
        // {".sdf", []() -> std::unique_ptr<RobotExporter>
        // { return std::make_unique<SdfExporter>(); }},
    };

    return registry;
}

// ----------------------------------------------------------------------------
std::string RobotExporterFactory::getExtension(std::string const& p_filename)
{
    // Find the last dot in the filename
    size_t dot_pos = p_filename.find_last_of('.');
    if (dot_pos == std::string::npos)
    {
        return "";
    }

    // Extract extension and convert to lowercase
    std::string ext = p_filename.substr(dot_pos);
    std::transform(ext.begin(),
                   ext.end(),
                   ext.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    return ext;
}

// ----------------------------------------------------------------------------
std::unique_ptr<RobotExporter>
RobotExporterFactory::create(const std::string& p_filename)
{
    std::string extension = getExtension(p_filename);

    auto& registry = getRegistry();
    if (auto it = registry.find(extension); it != registry.end())
    {
        return it->second();
    }

    // Format not supported
    return nullptr;
}

// ----------------------------------------------------------------------------
void RobotExporterFactory::registerExporter(const std::string& p_extension,
                                            ExporterCreator&& p_creator)
{
    auto& registry = getRegistry();

    // Convert extension to lowercase
    std::string ext = p_extension;
    std::transform(ext.begin(),
                   ext.end(),
                   ext.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    // Ensure extension starts with a dot
    if (!ext.empty() && ext[0] != '.')
    {
        ext = "." + ext;
    }

    registry[ext] = std::move(p_creator);
}

} // namespace robotik
