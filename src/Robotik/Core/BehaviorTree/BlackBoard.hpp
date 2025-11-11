/**
 * @file Builder.hpp
 * @brief Builder class for creating behavior trees from YAML.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

#include <any>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace bt
{

// ****************************************************************************
//! \brief Blackboard class allowing to store and share data between nodes of
//! different types using key-value pairs.
// ****************************************************************************
class Blackboard
{
public:

    template <class T>
    using Entry = std::unordered_map<std::string, T>;
    using Ptr = std::shared_ptr<Blackboard>;

    ~Blackboard() = default;

    // ------------------------------------------------------------------------
    //! \brief Set a value in the blackboard for a specific type.
    //! \param[in] key The key to set the value for.
    //! \param[in] val The value to set.
    // ------------------------------------------------------------------------
    template <typename T>
    void set(std::string const& key, T const& val)
    {
        if (key.empty())
            throw std::invalid_argument("Blackboard key cannot be empty");
        storage[key] = val;
    }

    // ------------------------------------------------------------------------
    //! \brief Get a value from the blackboard for a specific type.
    //! \param[in] key The key to get the value for.
    //! \return A pair containing the value and a boolean indicating if the
    //! value was found.
    // ------------------------------------------------------------------------
    template <typename T>
    [[nodiscard]] std::pair<T, bool> get(std::string const& key) const
    {
        if (auto it = storage.find(key); it != storage.end())
        {
            try
            {
                return { std::any_cast<T>(it->second), true };
            }
            catch (const std::bad_any_cast&)
            {
                return { T{}, false };
            }
        }
        return { T{}, false };
    }

    // ------------------------------------------------------------------------
    //! \brief Get a value from the blackboard for a specific type with a
    //! default value. \param[in] key The key to get the value for. \param[in]
    //! defaultValue The default value to return if the key is not found.
    //! \return The value from the blackboard or the default value if the key is
    //! not found.
    // ------------------------------------------------------------------------
    template <typename T>
    [[nodiscard]] T getOr(std::string const& key, const T& defaultValue) const
    {
        auto [value, found] = get<T>(key);
        return found ? value : defaultValue;
    }

    // ------------------------------------------------------------------------
    //! \brief Check if a key exists for a specific type
    //! \param[in] key The key to check
    //! \return true if the key exists, false otherwise
    // ------------------------------------------------------------------------
    template <class T>
    [[nodiscard]] bool has(std::string const& key) const
    {
        auto it = m_maps<T>.find(this);
        if (it == m_maps<T>.end())
            return false;
        return it->second.find(key) != it->second.end();
    }

    // ------------------------------------------------------------------------
    //! \brief Remove a key-value pair from the blackboard
    //! \param[in] key The key to remove
    //! \return true if the key was removed, false if it didn't exist
    // ------------------------------------------------------------------------
    template <class T>
    bool remove(std::string const& key)
    {
        auto it = m_maps<T>.find(this);
        if (it == m_maps<T>.end())
            return false;
        return it->second.erase(key) > 0;
    }

    // ------------------------------------------------------------------------
    //! \brief Get all keys for a specific type
    //! \return Vector of keys stored for type T
    // ------------------------------------------------------------------------
    template <class T>
    [[nodiscard]] std::vector<std::string> getKeys() const
    {
        std::vector<std::string> keys;
        if (auto it = m_maps<T>.find(this); it != m_maps<T>.end())
        {
            keys.reserve(it->second.size());
            for (const auto& pair : it->second)
            {
                keys.push_back(pair.first);
            }
        }
        return keys;
    }

    // ------------------------------------------------------------------------
    //! \brief Get the entries for a specific type
    //! \return The entries for the type
    // ------------------------------------------------------------------------
    template <class T>
    [[nodiscard]] inline Entry<T>& entries() const
    {
        return m_maps<T>.at(this);
    }

private:

    // ------------------------------------------------------------------------
    //! \brief The map of heterogeneous stacks
    // ------------------------------------------------------------------------
    template <class T>
    static std::unordered_map<const Blackboard*, Entry<T>> m_maps;

    // ------------------------------------------------------------------------
    //! \brief The map of heterogeneous stacks
    // ------------------------------------------------------------------------
    std::unordered_map<std::string, std::any> storage;
};
} // namespace bt