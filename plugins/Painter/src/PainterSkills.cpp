// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

#include "PainterSkills.hpp"

#include "Robotik/Perception/WorldModel.hpp"
#include "Robotik/Runtime/RobotContext.hpp"
#include "Robotik/Runtime/Scheduler.hpp"

#include <cmath>
#include <memory>
#include <numbers>

namespace painter
{

namespace
{

robotik::Status follow(std::vector<robotik::Pose> const& p_path,
                       std::size_t& p_index,
                       robotik::MoveTCPSkill& p_move,
                       robotik::RobotContext& p_context,
                       robotik::Seconds p_dt)
{
    if (p_index >= p_path.size())
    {
        return robotik::Status::SUCCESS;
    }
    robotik::Status const status = p_move.tick(p_context, p_dt);
    if (status == robotik::Status::RUNNING)
    {
        return status;
    }
    if (status == robotik::Status::FAILURE)
    {
        return status;
    }
    ++p_index;
    if (p_index >= p_path.size())
    {
        return robotik::Status::SUCCESS;
    }
    p_move.goal(p_path[p_index]);
    return robotik::Status::RUNNING;
}

enum class Gesture
{
    Dip,
    Mark,
    Rinse,
};

class GestureNode final: public bt::CallbackLeaf
{
public:

    GestureNode(robotik::SkillScheduler& p_scheduler,
                robotik::SkillId p_id,
                Gesture p_gesture)
        : CallbackLeaf([]() noexcept { return bt::Status::FAILURE; }),
          m_scheduler(p_scheduler),
          m_id(p_id),
          m_gesture(p_gesture)
    {
    }

    void fillConfig(bt::NodeConfig& p_config) const override
    {
        GestureNode* self = const_cast<GestureNode*>(this);
        p_config.callback = [self]() { return self->onTick(); };
        p_config.on_reset = [self]() {
            if (self->m_active)
            {
                self->m_scheduler.cancel(self->m_id);
                self->m_active = false;
            }
        };
    }

private:

    bt::Status onTick()
    {
        if (!m_active)
        {
            if (m_gesture != Gesture::Rinse)
            {
                auto const color = blackboard() ? blackboard()->get<std::string>("color")
                                                : std::nullopt;
                if (!color || color->empty())
                {
                    return bt::Status::FAILURE;
                }
                if (m_gesture == Gesture::Dip)
                {
                    dynamic_cast<DipSkill&>(m_scheduler.skill(m_id)).use(*color);
                }
                else
                {
                    dynamic_cast<MarkSkill&>(m_scheduler.skill(m_id)).use(*color);
                }
            }
            m_scheduler.request(m_id);
            m_active = true;
            return bt::Status::RUNNING;
        }
        switch (m_scheduler.state(m_id))
        {
            case robotik::SkillState::Idle:
            case robotik::SkillState::Waiting:
            case robotik::SkillState::Running:
                return bt::Status::RUNNING;
            case robotik::SkillState::Succeeded:
                m_active = false;
                return bt::Status::SUCCESS;
            case robotik::SkillState::Failed:
            case robotik::SkillState::Cancelled:
                m_active = false;
                return bt::Status::FAILURE;
        }
        return bt::Status::FAILURE;
    }

    robotik::SkillScheduler& m_scheduler;
    robotik::SkillId m_id = robotik::NO_SKILL;
    Gesture m_gesture = Gesture::Dip;
    bool m_active = false;
};

} // namespace

DipSkill::DipSkill(Stroke& p_stroke) : m_stroke(&p_stroke)
{
}

void DipSkill::use(std::string p_color)
{
    m_color = std::move(p_color);
}

void DipSkill::reset()
{
    m_path.clear();
    m_index = 0;
    m_ready = false;
    m_move.reset();
    m_stroke->painting = false;
}

robotik::Status DipSkill::tick(robotik::RobotContext& p_context, robotik::Seconds p_dt)
{
    if (!m_ready)
    {
        robotik::WorldObject const* pot = p_context.world_model.find(m_color);
        if (pot == nullptr)
        {
            return robotik::Status::FAILURE;
        }
        robotik::Quaternion const down = brushDown();
        double const top = pot->top();
        m_path = {
            { { pot->position.x, pot->position.y, top + 0.20 }, down },
            { { pot->position.x, pot->position.y, top - 0.01 }, down },
            { { pot->position.x, pot->position.y, top + 0.20 }, down },
        };
        m_move = robotik::MoveTCPSkill({}, m_path.front(), 2e-2);
        m_ready = true;
    }
    robotik::Status const status = follow(m_path, m_index, m_move, p_context, p_dt);
    if (status == robotik::Status::SUCCESS)
    {
        m_stroke->brush = m_color;
    }
    return status;
}

void DipSkill::cancel(robotik::RobotContext& p_context)
{
    m_stroke->painting = false;
    m_move.cancel(p_context);
}

MarkSkill::MarkSkill(Stroke& p_stroke, Strokes p_strokes, Settings p_settings)
    : m_stroke(&p_stroke), m_strokes(std::move(p_strokes)), m_settings(p_settings)
{
}

void MarkSkill::use(std::string p_color)
{
    m_color = std::move(p_color);
    m_stroke->color = m_color;
}

void MarkSkill::reset()
{
    m_path.clear();
    m_index = 0;
    m_ready = false;
    m_move.reset();
    m_stroke->painting = false;
}

robotik::Status MarkSkill::tick(robotik::RobotContext& p_context, robotik::Seconds p_dt)
{
    if (!m_ready)
    {
        auto const stroke = m_strokes.find(m_color);
        if (stroke == m_strokes.end() || stroke->second.size() < 3u)
        {
            return robotik::Status::FAILURE;
        }
        // Straight lines in Cartesian space: cut each segment into small
        // steps, each one solved by IK.
        std::vector<robotik::Pose> const& points = stroke->second;
        m_path.assign(1u, points.front());
        for (std::size_t i = 1; i < points.size(); ++i)
        {
            robotik::Vector3 const from = points[i - 1].position;
            robotik::Vector3 const to = points[i].position;
            double const length = (to - from).norm();
            std::size_t const steps =
                m_settings.step > 0.0 ? std::max<std::size_t>(1u, static_cast<std::size_t>(std::ceil(length / m_settings.step)))
                                      : 1u;
            for (std::size_t k = 1; k <= steps; ++k)
            {
                double const t = static_cast<double>(k) / static_cast<double>(steps);
                m_path.push_back(robotik::Pose{ from + (to - from) * t, points[i].rotation });
            }
            if (i == 1u)
            {
                m_paint_begin = m_path.size() - steps;
            }
            if (i + 2u == points.size())
            {
                m_paint_end = m_path.size() - 1u;
            }
        }
        m_move = robotik::MoveTCPSkill({}, m_path.front(), m_settings.tolerance);
        m_stroke->color = m_color;
        m_ready = true;
    }
    // The first point approaches, the last one leaves: the brush paints in
    // between.
    m_stroke->painting = m_index >= m_paint_begin && m_index <= m_paint_end;
    robotik::Status const status = follow(m_path, m_index, m_move, p_context, p_dt);
    if (status != robotik::Status::RUNNING)
    {
        m_stroke->painting = false;
    }
    if (status == robotik::Status::SUCCESS)
    {
        m_stroke->painted.insert(m_color);
    }
    return status;
}

void MarkSkill::cancel(robotik::RobotContext& p_context)
{
    m_stroke->painting = false;
    m_move.cancel(p_context);
}

RinseSkill::RinseSkill(Stroke& p_stroke) : m_stroke(&p_stroke)
{
}

void RinseSkill::reset()
{
    m_path.clear();
    m_index = 0;
    m_ready = false;
    m_move.reset();
    m_stroke->painting = false;
}

robotik::Status RinseSkill::tick(robotik::RobotContext& p_context, robotik::Seconds p_dt)
{
    if (!m_ready)
    {
        robotik::WorldObject const* water = p_context.world_model.find("water");
        if (water == nullptr)
        {
            return robotik::Status::FAILURE;
        }
        robotik::Quaternion const down = brushDown();
        double const x = water->position.x;
        double const y = water->position.y;
        double const wet = water->top() - 0.01;
        m_path.push_back({ { x, y, water->top() + 0.18 }, down });
        constexpr int kSteps = 8;
        for (int turn = 0; turn < 2; ++turn)
        {
            for (int i = 0; i < kSteps; ++i)
            {
                double const angle = (turn * kSteps + i) * (2.0 * std::numbers::pi / kSteps);
                m_path.push_back({ { x + 0.02 * std::cos(angle),
                                     y + 0.02 * std::sin(angle),
                                     wet },
                                   down });
            }
        }
        m_path.push_back({ { x, y, water->top() + 0.18 }, down });
        m_move = robotik::MoveTCPSkill({}, m_path.front(), 2e-2);
        m_ready = true;
    }
    robotik::Status const status = follow(m_path, m_index, m_move, p_context, p_dt);
    if (status == robotik::Status::SUCCESS)
    {
        m_stroke->brush.clear();
    }
    return status;
}

void RinseSkill::cancel(robotik::RobotContext& p_context)
{
    m_move.cancel(p_context);
}

void registerNodes(bt::NodeFactory& p_factory,
                   robotik::SkillScheduler& p_scheduler)
{
    auto bind = [&](char const* p_node, char const* p_skill, Gesture p_gesture) {
        p_factory.registerNode(
            p_node, [&p_scheduler, name = std::string(p_skill), p_gesture]() {
                return std::make_unique<GestureNode>(
                    p_scheduler, p_scheduler.find(name), p_gesture);
            });
    };
    bind("Dip", "dip", Gesture::Dip);
    bind("Mark", "mark", Gesture::Mark);
    bind("Rinse", "rinse", Gesture::Rinse);
}

} // namespace painter
