/**
 * @file Path.cpp
 * @brief Path class for searching files in the same idea of the Unix
 * environment variable $PATH.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#include "Robotik/private/Path.hpp"
#include <algorithm>
#include <cctype>
#include <dirent.h>
#include <sstream>
#include <sys/stat.h>

namespace robotik
{

//------------------------------------------------------------------------------
Path::Path(std::string const& path, char const delimiter)
    : m_delimiter(delimiter)
{
    add(path);
}

//------------------------------------------------------------------------------
void Path::add(std::string const& path)
{
    if (!path.empty())
    {
        split(path);
    }
}

//------------------------------------------------------------------------------
void Path::reset(std::string const& path)
{
    m_search_paths.clear();
    split(path);
}

//------------------------------------------------------------------------------
void Path::clear()
{
    m_search_paths.clear();
}

//------------------------------------------------------------------------------
void Path::remove(std::string const& path)
{
    m_search_paths.remove(path);
}

//------------------------------------------------------------------------------
bool Path::exist(std::string const& path) const
{
    struct stat buffer;
    return stat(path.c_str(), &buffer) == 0;
}

//------------------------------------------------------------------------------
std::pair<std::string, bool> Path::find(std::string const& filename) const
{
    if (Path::exist(filename))
        return std::make_pair(filename, true);

    for (auto const& it : m_search_paths)
    {
        std::string file(it + filename);
        if (Path::exist(file))
            return std::make_pair(file, true);
    }

    // Not found
    return std::make_pair(std::string(), false);
}

//------------------------------------------------------------------------------
std::string Path::expand(std::string const& filename) const
{
    for (auto const& it : m_search_paths)
    {
        std::string file(it + filename);
        if (Path::exist(file))
            return file;
    }

    return filename;
}

//------------------------------------------------------------------------------
std::pair<std::string, bool>
Path::findWithExtension(std::string const& filename,
                        std::string const& extension) const
{
    std::string file_with_ext = addExtensionIfMissing(filename, extension);
    return find(file_with_ext);
}

//------------------------------------------------------------------------------
std::pair<std::string, bool>
Path::findWithExtensions(std::string const& filename,
                         std::vector<std::string> const& extensions) const
{
    // First try the filename as-is
    auto result = find(filename);
    if (result.second)
        return result;

    // Then try with each extension
    for (const auto& ext : extensions)
    {
        std::string file_with_ext = addExtensionIfMissing(filename, ext);
        result = find(file_with_ext);
        if (result.second)
            return result;
    }

    return std::make_pair(std::string(), false);
}

//------------------------------------------------------------------------------
std::string Path::expandWithExtension(std::string const& filename,
                                      std::string const& extension) const
{
    std::string file_with_ext = addExtensionIfMissing(filename, extension);
    return expand(file_with_ext);
}

//------------------------------------------------------------------------------
bool Path::exists(std::string const& filename) const
{
    return find(filename).second;
}

//------------------------------------------------------------------------------
std::vector<std::string>
Path::findAllWithExtension(std::string const& extension) const
{
    std::vector<std::string> result;

    for (const auto& search_path : m_search_paths)
    {
        DIR* dir = opendir(search_path.c_str());
        if (dir == nullptr)
            continue;

        const struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr)
        {
            std::string filename(entry->d_name);
            if (hasExtension(filename, extension))
            {
                std::string full_path = search_path + filename;
                if (exist(full_path))
                {
                    result.push_back(full_path);
                }
            }
        }
        closedir(dir);
    }

    return result;
}

//------------------------------------------------------------------------------
bool Path::open(std::string& filename,
                std::ifstream& ifs,
                std::ios_base::openmode mode) const
{
    ifs.open(filename.c_str(), mode);
    if (ifs)
        return true;

    for (auto const& it : m_search_paths)
    {
        std::string file(it + filename);
        ifs.open(file.c_str(), mode);
        if (ifs)
        {
            filename = file;
            return true;
        }
    }

    // Not found
    return false;
}

//------------------------------------------------------------------------------
bool Path::open(std::string& filename,
                std::ofstream& ofs,
                std::ios_base::openmode mode) const
{
    ofs.open(filename.c_str(), mode);
    if (ofs)
        return true;

    for (auto const& it : m_search_paths)
    {
        std::string file(it + filename);
        ofs.open(file.c_str(), mode);
        if (ofs)
        {
            filename = file;
            return true;
        }
    }

    // Not found
    return false;
}

//------------------------------------------------------------------------------
bool Path::open(std::string& filename,
                std::fstream& fs,
                std::ios_base::openmode mode) const
{
    fs.open(filename.c_str(), mode);
    if (fs)
        return true;

    for (auto const& it : m_search_paths)
    {
        std::string file(it + filename);
        fs.open(filename.c_str(), mode);
        if (fs)
        {
            filename = file;
            return true;
        }
    }

    // Not found
    return false;
}

//------------------------------------------------------------------------------
std::vector<std::string> Path::pathes() const
{
    std::vector<std::string> res;
    res.reserve(m_search_paths.size());
    for (auto const& it : m_search_paths)
    {
        res.push_back(it);
    }

    return res;
}

//------------------------------------------------------------------------------
std::string Path::toString() const
{
    std::string string_path;

    string_path += ".";
    string_path += m_delimiter;

    for (auto const& it : m_search_paths)
    {
        string_path += it;
        string_path.pop_back(); // Remove the '/' char
        string_path += m_delimiter;
    }

    return string_path;
}

//------------------------------------------------------------------------------
void Path::split(std::string const& path)
{
    std::stringstream ss(path);
    std::string directory;

    while (std::getline(ss, directory, m_delimiter))
    {
        if (directory.empty())
            continue;

        if ((*directory.rbegin() == '\\') || (*directory.rbegin() == '/'))
            m_search_paths.push_back(directory);
        else
            m_search_paths.push_back(directory + "/");
    }
}

//------------------------------------------------------------------------------
bool Path::hasExtension(std::string const& filename,
                        std::string const& extension) const
{
    if (filename.length() < extension.length())
        return false;

    return std::equal(extension.rbegin(),
                      extension.rend(),
                      filename.rbegin(),
                      [](char a, char b)
                      { return std::tolower(a) == std::tolower(b); });
}

//------------------------------------------------------------------------------
std::string Path::addExtensionIfMissing(std::string const& filename,
                                        std::string const& extension) const
{
    if (hasExtension(filename, extension))
        return filename;

    return filename + extension;
}

} // namespace robotik