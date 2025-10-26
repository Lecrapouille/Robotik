/**
 * @file RobotLoaderFactory.cpp
 * @brief Implementation of robot loader factory.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/Core/Loaders/RobotLoaderFactory.hpp"
#include "Robotik/Core/Loaders/UrdfLoader.hpp"

#include <algorithm>
#include <cctype>

namespace robotik::loader
{

// ----------------------------------------------------------------------------
std::unordered_map<std::string, RobotLoaderFactory::LoaderCreator>&
RobotLoaderFactory::getRegistry()
{
    static std::unordered_map<std::string, LoaderCreator> registry;

    // Register default loaders on first access
    static bool initialized = false;
    if (!initialized)
    {
        // Register URDF loader
        registry[".urdf"] = []() -> std::unique_ptr<RobotLoader>
        { return std::make_unique<URDFLoader>(); };

        // Future loaders can be registered here:
        // registry[".sdf"] = []() { return std::make_unique<SDFParser>(); };
        // registry[".mjcf"] = []() { return std::make_unique<MJCFParser>(); };

        initialized = true;
    }

    return registry;
}

// ----------------------------------------------------------------------------
std::string RobotLoaderFactory::getExtension(const std::string& p_filename)
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
std::unique_ptr<RobotLoader>
RobotLoaderFactory::create(const std::string& p_filename)
{
    std::string extension = getExtension(p_filename);

    auto& registry = getRegistry();
    auto it = registry.find(extension);

    if (it != registry.end())
    {
        return it->second();
    }

    // Format not supported
    return nullptr;
}

// ----------------------------------------------------------------------------
void RobotLoaderFactory::registerLoader(const std::string& p_extension,
                                        LoaderCreator p_creator)
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

    registry[ext] = p_creator;
}

} // namespace robotik::loader
