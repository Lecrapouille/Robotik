// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "Robotik/Runtime/Resources.hpp"

#include <algorithm>
#include <stdexcept>

namespace robotik
{

ResourceLease::ResourceLease(ResourceLease&& p_other) noexcept
    : m_manager(p_other.m_manager),
      m_owner(p_other.m_owner),
      m_items(p_other.m_items),
      m_count(p_other.m_count)
{
    p_other.m_manager = nullptr;
    p_other.m_count = 0;
}

ResourceLease& ResourceLease::operator=(ResourceLease&& p_other) noexcept
{
    if (this != &p_other)
    {
        release();
        m_manager = p_other.m_manager;
        m_owner = p_other.m_owner;
        m_items = p_other.m_items;
        m_count = p_other.m_count;
        p_other.m_manager = nullptr;
        p_other.m_count = 0;
    }
    return *this;
}

ResourceLease::~ResourceLease()
{
    release();
}

void ResourceLease::release()
{
    if (m_manager != nullptr)
    {
        m_manager->release(*this);
        m_manager = nullptr;
        m_count = 0;
    }
}

ResourceId ResourceManager::add(std::string p_name)
{
    if (ResourceId const existing = find(p_name); existing != NO_RESOURCE)
    {
        return existing;
    }
    if (m_names.size() >= NO_RESOURCE)
    {
        throw std::length_error("Too many resources");
    }
    m_names.push_back(std::move(p_name));
    m_failed.push_back(0u);
    m_owners.push_back(NO_OWNER);
    m_users.push_back(0u);
    return static_cast<ResourceId>(m_names.size() - 1u);
}

ResourceId ResourceManager::find(std::string_view p_name) const
{
    for (std::size_t i = 0; i < m_names.size(); ++i)
    {
        if (m_names[i] == p_name)
        {
            return static_cast<ResourceId>(i);
        }
    }
    return NO_RESOURCE;
}

ResourceRequirement ResourceManager::require(std::string_view p_name,
                                             Access p_access) const
{
    ResourceId const id = find(p_name);
    if (id == NO_RESOURCE)
    {
        throw std::invalid_argument("Unknown resource '" + std::string(p_name) +
                                    "'");
    }
    return { id, p_access };
}

void ResourceManager::fail(ResourceId p_id)
{
    if (p_id < m_failed.size())
    {
        m_failed[p_id] = 1u;
    }
}

void ResourceManager::restore(ResourceId p_id)
{
    if (p_id < m_failed.size())
    {
        m_failed[p_id] = 0u;
    }
}

void ResourceManager::fail(std::string_view p_name)
{
    fail(find(p_name));
}

void ResourceManager::restore(std::string_view p_name)
{
    restore(find(p_name));
}

void ResourceManager::restoreAll()
{
    std::fill(m_failed.begin(), m_failed.end(), std::uint8_t(0u));
}

Conflict ResourceManager::conflict(
    std::span<ResourceRequirement const> p_requirements) const
{
    for (ResourceRequirement const& requirement : p_requirements)
    {
        ResourceId const id = requirement.id;
        if (!available(id))
        {
            return { id, NO_OWNER, true };
        }
        if (m_owners[id] != NO_OWNER)
        {
            return { id, m_owners[id], false };
        }
        if (requirement.access == Access::Exclusive && m_users[id] > 0u)
        {
            return { id, NO_OWNER, false };
        }
    }
    return {};
}

ResourceLease
ResourceManager::acquire(std::span<ResourceRequirement const> p_requirements,
                         OwnerId p_owner)
{
    ResourceLease lease;
    if (p_requirements.size() > ResourceLease::CAPACITY || conflict(p_requirements))
    {
        return lease;
    }
    for (ResourceRequirement const& requirement : p_requirements)
    {
        if (requirement.access == Access::Exclusive)
        {
            m_owners[requirement.id] = p_owner;
        }
        else
        {
            ++m_users[requirement.id];
        }
        lease.m_items[lease.m_count++] = requirement;
    }
    lease.m_manager = this;
    lease.m_owner = p_owner;
    return lease;
}

void ResourceManager::release(ResourceLease& p_lease)
{
    for (ResourceRequirement const& requirement : p_lease.resources())
    {
        if (requirement.access == Access::Exclusive)
        {
            m_owners[requirement.id] = NO_OWNER;
        }
        else if (m_users[requirement.id] > 0u)
        {
            --m_users[requirement.id];
        }
    }
}

} // namespace robotik
