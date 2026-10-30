// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// The web build has no Pinocchio and no MuJoCo. The panels still open and
/// the plugins are listed; building a robot or stepping physics says why it
/// cannot.

#include "Robotik/Robot/Backends/MujocoBackend.hpp"
#include "Robotik/Robot/Backends/PinocchioBackend.hpp"

#include <stdexcept>
#include <utility>

namespace robotik
{
namespace
{

[[noreturn]] void missing(char const* p_library)
{
    throw std::runtime_error(std::string(p_library) +
                             " is not available in the web build");
}

} // namespace

struct PinocchioBackend::Impl
{
};

PinocchioBackend::PinocchioBackend(std::filesystem::path const&,
                                   std::filesystem::path const&)
    : m_impl(std::make_unique<Impl>())
{
    missing("Pinocchio");
}

PinocchioBackend::~PinocchioBackend() = default;

std::size_t PinocchioBackend::nq() const
{
    return 0U;
}

std::size_t PinocchioBackend::nv() const
{
    return 0U;
}

std::span<double> PinocchioBackend::configuration()
{
    static std::vector<double> q;
    return q;
}

std::span<double const> PinocchioBackend::configuration() const
{
    static std::vector<double> const q;
    return q;
}

std::span<double> PinocchioBackend::velocity()
{
    static std::vector<double> v;
    return v;
}

std::span<double const> PinocchioBackend::velocity() const
{
    static std::vector<double> const v;
    return v;
}

void PinocchioBackend::updateKinematics()
{
}

bool PinocchioBackend::hasJoint(std::string const&) const
{
    return false;
}

int PinocchioBackend::qIndex(std::string const&) const
{
    return -1;
}

int PinocchioBackend::vIndex(std::string const&) const
{
    return -1;
}

std::size_t PinocchioBackend::jointId(std::string const&) const
{
    missing("Pinocchio");
}

bool PinocchioBackend::hasFrame(std::string const&) const
{
    return false;
}

std::vector<std::string> PinocchioBackend::linkFrames() const
{
    return {};
}

std::size_t PinocchioBackend::frameId(std::string const&) const
{
    missing("Pinocchio");
}

Pose PinocchioBackend::framePose(std::string const&) const
{
    missing("Pinocchio");
}

Pose PinocchioBackend::framePose(std::size_t) const
{
    missing("Pinocchio");
}

std::optional<std::vector<double>>
PinocchioBackend::solveIK(std::string const&,
                          Pose const&,
                          std::span<double const>,
                          IkGoal) const
{
    return std::nullopt;
}

std::size_t PinocchioBackend::chainDofs(std::string const&) const
{
    return 0U;
}

std::array<bool, 6> PinocchioBackend::movableAxes(std::string const&,
                                                  IkGoal) const
{
    return {};
}

struct MujocoBackend::Impl
{
};

struct MujocoBackend::Member
{
};

MujocoBackend::MujocoBackend(MujocoOptions p_options)
    : m_impl(std::make_unique<Impl>()), m_options(std::move(p_options))
{
}

MujocoBackend::~MujocoBackend() = default;

std::string MujocoBackend::load(std::filesystem::path const&,
                                ChainOptions const&)
{
    missing("MuJoCo");
}

void MujocoBackend::addBox(FreeBox const&)
{
}

void MujocoBackend::addWeld(std::string, std::string, std::string)
{
}

void MujocoBackend::excludeContacts(std::string, std::string)
{
}

bool MujocoBackend::weld(std::string const&, bool)
{
    return false;
}

bool MujocoBackend::placeBody(std::string const&, Pose const&)
{
    return false;
}

int MujocoBackend::freeBody(std::string const&) const
{
    return -1;
}

void MujocoBackend::drive(int, Vector3 const&, double, double)
{
}

std::optional<Pose> MujocoBackend::bodyPose(std::string const&) const
{
    return std::nullopt;
}

std::vector<Contact> MujocoBackend::contactList() const
{
    return {};
}

void MujocoBackend::attach(std::string const&,
                           std::string const&,
                           std::string const&,
                           std::string const&)
{
    missing("MuJoCo");
}

void MujocoBackend::detach(std::string const&,
                           std::string const&,
                           std::string const&,
                           std::string const&)
{
}

void MujocoBackend::attach(Robot&)
{
}

void MujocoBackend::join(Robot&, std::string const&)
{
    missing("MuJoCo");
}

std::unique_ptr<RobotBackend>
MujocoBackend::member(std::shared_ptr<MujocoBackend>, std::string)
{
    missing("MuJoCo");
}

void MujocoBackend::reset(Robot&)
{
}

void MujocoBackend::step(Robot&, Seconds)
{
}

int MujocoBackend::contacts() const
{
    return 0;
}

std::optional<double> MujocoBackend::raycast(Vector3 const&,
                                             Vector3 const&,
                                             double) const
{
    return std::nullopt;
}

std::optional<double> MujocoBackend::raycast(Vector3 const&,
                                             Vector3 const&,
                                             double,
                                             std::string const&) const
{
    return std::nullopt;
}

std::optional<Wrench> MujocoBackend::wrench(std::string const&) const
{
    return std::nullopt;
}

void MujocoBackend::invalidate()
{
}

void MujocoBackend::compile()
{
}

void MujocoBackend::bind(Member&)
{
}

} // namespace robotik
