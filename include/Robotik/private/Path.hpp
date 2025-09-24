/**
 * @file Path.hpp
 * @brief Path class for searching files in the same idea of the Unix
 * environment variable $PATH.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include <fstream>
#include <list>
#include <string>
#include <vector>

namespace robotik
{

//------------------------------------------------------------------------------
#if defined(__APPLE__)
std::string osx_get_resources_dir(std::string const& file);
#endif

//------------------------------------------------------------------------------
#undef GET_DATA_PATH
#if defined(__APPLE__)
#    define GET_DATA_PATH osx_get_resources_dir("")
#else
#    define GET_DATA_PATH project::info::data_path
#endif

// *****************************************************************************
//! \brief Class manipulating a set of paths for searching files in the same
//! idea of the Unix environment variable $PATH. Paths are separated by ':' and
//! search is made left to right. Ie. "/foo/bar:/usr/lib/"
// *****************************************************************************
class Path
{
public:

    //--------------------------------------------------------------------------
    //! \brief Empty constructor. No path is defined.
    //--------------------------------------------------------------------------
    Path() = default;

    //--------------------------------------------------------------------------
    //! \brief Constructor with a given path. Directories shall be separated
    //! by the character given by the param delimiter.
    //! Example: Path("/foo/bar:/usr/lib/", ':').
    //--------------------------------------------------------------------------
    explicit Path(std::string const& path, char const delimiter = ':');

    //--------------------------------------------------------------------------
    //! \brief Destructor.
    //--------------------------------------------------------------------------
    ~Path() = default;

    //--------------------------------------------------------------------------
    //! \brief Append a new path. Directories are separated by the delimiter
    //! char (by default ':'). Example: add("/foo/bar:/usr/lib/").
    //--------------------------------------------------------------------------
    void add(std::string const& path);

    //--------------------------------------------------------------------------
    //! \brief Replace the path state by a new one. Directories are separated by
    //! the delimiter char (by default ':'). Example:
    //! reset("/foo/bar:/usr/lib/").
    //--------------------------------------------------------------------------
    void reset(std::string const& path);

    //--------------------------------------------------------------------------
    //! \brief Erase the path.
    //--------------------------------------------------------------------------
    void clear();

    //--------------------------------------------------------------------------
    //! \brief Erase the given directory from the path if found.
    //--------------------------------------------------------------------------
    void remove(std::string const& path);

    //--------------------------------------------------------------------------
    //! \brief Find if a file exists in the search path. Note that you have to
    //! check again the existence of this file when opening it (with functions
    //! such as iofstream, fopen, open ...). Indeed the file may have been
    //! suppress since this method have bee called.
    //!
    //! \return the full path (if found) and the existence of this path.
    //!
    //--------------------------------------------------------------------------
    std::pair<std::string, bool> find(std::string const& filename) const;

    //--------------------------------------------------------------------------
    //! \brief Find files with specific extension in the search path.
    //! \param filename Base filename (can be with or without extension)
    //! \param extension File extension to search for (e.g., ".stl", ".urdf")
    //! \return the full path (if found) and the existence of this path.
    //--------------------------------------------------------------------------
    std::pair<std::string, bool>
    findWithExtension(std::string const& filename,
                      std::string const& extension) const;

    //--------------------------------------------------------------------------
    //! \brief Find first file matching any of the given extensions.
    //! \param filename Base filename (without extension)
    //! \param extensions List of extensions to try (e.g., {".stl", ".obj",
    //! ".ply"})
    //! \return the full path (if found) and the existence of this path.
    //--------------------------------------------------------------------------
    std::pair<std::string, bool>
    findWithExtensions(std::string const& filename,
                       std::vector<std::string> const& extensions) const;

    //--------------------------------------------------------------------------
    //! \brief Return the full path for the file (if found) else return itself.
    //! Beware of race condition: even if found the file may have suppress after
    //! this function has been called.
    //--------------------------------------------------------------------------
    std::string expand(std::string const& filename) const;

    //--------------------------------------------------------------------------
    //! \brief Expand filename with extension support.
    //! \param filename Base filename
    //! \param extension File extension to append if not present
    //! \return the full path (if found) else return original filename
    //--------------------------------------------------------------------------
    std::string expandWithExtension(std::string const& filename,
                                    std::string const& extension) const;

    //--------------------------------------------------------------------------
    //! \brief Return the container of path
    //--------------------------------------------------------------------------
    std::vector<std::string> pathes() const;

    //--------------------------------------------------------------------------
    //! \brief Return pathes as string. The first path is always ".:"
    //--------------------------------------------------------------------------
    std::string toString() const;

    //--------------------------------------------------------------------------
    //! \brief Check if a file exists (convenience method).
    //! \param filename File to check
    //! \return true if file exists
    //--------------------------------------------------------------------------
    bool exists(std::string const& filename) const;

    //--------------------------------------------------------------------------
    //! \brief Get all files with specific extension in all search paths.
    //! \param extension File extension to search for (e.g., ".stl")
    //! \return vector of full paths to matching files
    //--------------------------------------------------------------------------
    std::vector<std::string>
    findAllWithExtension(std::string const& extension) const;

    bool open(std::string& filename,
              std::ifstream& ifs,
              std::ios_base::openmode mode = std::ios_base::in) const;
    bool open(std::string& filename,
              std::ofstream& ifs,
              std::ios_base::openmode mode = std::ios_base::out) const;
    bool open(std::string& filename,
              std::fstream& ifs,
              std::ios_base::openmode mode = std::ios_base::in |
                                             std::ios_base::out) const;

protected:

    //--------------------------------------------------------------------------
    //! \brief Slipt paths separated by delimiter char into std::list
    //--------------------------------------------------------------------------
    void split(std::string const& path);

    //--------------------------------------------------------------------------
    //! \brief Return true if the path exists. be careful the file may not
    //! exist after the function ends.
    //--------------------------------------------------------------------------
    bool exist(std::string const& path) const;

    //--------------------------------------------------------------------------
    //! \brief Check if filename has the given extension.
    //! \param filename File to check
    //! \param extension Extension to check for (e.g., ".stl")
    //! \return true if filename ends with extension
    //--------------------------------------------------------------------------
    bool hasExtension(std::string const& filename,
                      std::string const& extension) const;

    //--------------------------------------------------------------------------
    //! \brief Add extension to filename if not already present.
    //! \param filename Base filename
    //! \param extension Extension to add (e.g., ".stl")
    //! \return filename with extension
    //--------------------------------------------------------------------------
    std::string addExtensionIfMissing(std::string const& filename,
                                      std::string const& extension) const;

protected:

    //! \brief Path separator when several pathes are given as a single string.
    const char m_delimiter = ':';
    //! \brief the list of pathes.
    std::list<std::string> m_search_paths;
};
} // namespace robotik