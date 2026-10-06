// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "PickPlaceMission.hpp"

#include "ColorDetector.hpp"

#include "Robotik/Runtime/Simulation.hpp"
#include "Robotik/Skills/PickPlaceSkills.hpp"

#define APPROACH_CLEARANCE_M 0.10
#define SKILL_PRIORITY 100

void PickPlaceMission::setup(robotik::Simulation& p_simulation,
                             robotik::SceneView* /*p_view*/)
{
    if (!m_skills)
    {
        return;
    }

    for (robotik::Scenario::Object const& object :
         p_simulation.scenario().objects)
    {
        if (m_detector == nullptr)
        {
            m_detector = &p_simulation.perception().add<ColorDetector>();
        }
        m_detector->add(object.shape.name, object.shape.color);
    }

    robotik::ResourceManager& resources = p_simulation.robot().resources();
    robotik::ActuatorSet const& actuators = p_simulation.robot().actuators();
    robotik::SkillScheduler& skills = p_simulation.skills();

    std::vector<robotik::ResourceRequirement> arm;
    if (auto const* group = actuators.first<robotik::JointGroup>())
    {
        arm.push_back(resources.require(group->name()));
    }
    std::vector<robotik::ResourceRequirement> gripper;
    if (auto const* vacuum = actuators.first<robotik::VacuumGripper>())
    {
        gripper.push_back(resources.require(vacuum->name()));
    }
    std::vector<robotik::ResourceRequirement> camera;
    if (robotik::Camera const* found = p_simulation.camera())
    {
        camera.push_back(resources.require(found->name(), robotik::Access::Shared));
    }

    auto describe = [](std::string p_name,
                       std::vector<robotik::ResourceRequirement> p_resources)
    {
        robotik::SkillDescription description;
        description.name = std::move(p_name);
        description.resources = std::move(p_resources);
        description.priority = SKILL_PRIORITY;
        return description;
    };

    skills.add<robotik::ReleaseSkill>(describe("Release", gripper));
    for (robotik::Scenario::Object const& object :
         p_simulation.scenario().objects)
    {
        std::string const& name = object.shape.name;
        skills.add<robotik::DetectSkill>(
            describe("Detect(" + name + ")", camera), name);
        skills.add<robotik::ApproachSkill>(
            describe("Approach(" + name + ")", arm),
            name,
            Length(APPROACH_CLEARANCE_M));
        skills.add<robotik::ApproachSkill>(
            describe("Reach(" + name + ")", arm), name, Length{});

        robotik::SkillDescription grasp = describe("Grasp(" + name + ")", gripper);
        grasp.wait = false;
        grasp.preconditions.push_back(
            { "gripper is empty",
              [](robotik::RobotContext const& p_context)
              {
                  robotik::VacuumGripper const* vacuum =
                      robotik::findGripper(p_context.robot, std::string{});
                  return vacuum != nullptr && !vacuum->holding();
              } });
        skills.add<robotik::GraspSkill>(std::move(grasp), name);
    }
}
