// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.
#include "Ui.hpp"

#include "Robotik/Robot/Actuators.hpp"

#include <imgui.h>
#include <implot.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <numbers>
#include <string>

namespace
{

constexpr double DEGREES = 180.0 / std::numbers::pi;

//! @brief Limits and display units of one joint.
struct JointInfo
{
    bool linear = false;
    double lower = 0.0;
    double upper = 0.0;
    double velocity = std::numeric_limits<double>::infinity();
    double effort = std::numeric_limits<double>::infinity();
    bool bounded = false;
    //!< SI value to display value: degrees for angles, metres for lengths.
    double unit = 1.0;
    char const* position_unit = "m";
    char const* velocity_unit = "m/s";
    char const* accel_unit = "m/s²";
    char const* effort_unit = "N";
};

JointInfo infoOf(robotik::JointSet const& p_joints, robotik::JointId p_id)
{
    JointInfo info;
    info.linear = p_joints.isPrismatic(p_id);
    if (info.linear)
    {
        auto const limits = p_joints.limits(p_joints.prismatic(p_id));
        info.lower = limits.lower.value();
        info.upper = limits.upper.value();
        info.velocity = limits.velocity.value();
        info.effort = limits.effort.value();
        info.bounded = limits.bounded();
        return info;
    }
    auto const limits = p_joints.limits(p_joints.revolute(p_id));
    info.lower = limits.lower.value();
    info.upper = limits.upper.value();
    info.velocity = limits.velocity.value();
    info.effort = limits.effort.value();
    info.bounded = limits.bounded();
    info.unit = DEGREES;
    info.position_unit = "°";
    info.velocity_unit = "°/s";
    info.accel_unit = "°/s²";
    info.effort_unit = "N·m";
    return info;
}

double finiteOr(double p_value, double p_fallback)
{
    return std::isfinite(p_value) && p_value > 0.0 ? p_value : p_fallback;
}

void header(App& p_app)
{
    robotik::RobotSession& robot = p_app.simulation->robot();
    robotik::Scenario const& scenario = p_app.simulation->scenario();
    ImGui::TextUnformatted(robot.name().c_str());
    ImGui::SameLine();
    ImGui::TextDisabled("%s  ·  %zu joints", scenario.world.robot().model_ref.c_str(), robot.joints().size());

    // Who moves the robot: the mission, or the operator.
    bool const rl = p_app.owns_clock;
    ImGui::BeginDisabled(rl);
    bool manual = p_app.teach.manual;
    ImGui::PushStyleColor(ImGuiCol_Button, !manual ? ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive)
                                                   : ImGui::GetStyleColorVec4(ImGuiCol_FrameBg));
    if (ImGui::Button("Mission drives") && manual)
    {
        p_app.teach.manual = false;
        p_app.simulation->suspend(false);
        p_app.teach.pendant.stop(robot);
    }
    ImGui::PopStyleColor();
    ImGui::SameLine(0.0f, 0.0f);
    ImGui::PushStyleColor(ImGuiCol_Button, manual ? ImVec4(0.55f, 0.38f, 0.10f, 1.0f)
                                                  : ImGui::GetStyleColorVec4(ImGuiCol_FrameBg));
    if (ImGui::Button("Manual") && !manual)
    {
        takeArm(p_app);
    }
    ImGui::PopStyleColor();
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (rl)
    {
        ImGui::TextDisabled("the RL plugin drives the robot");
    }
    else
    {
        ImGui::TextDisabled(manual ? "mission suspended: the robot moves even when paused"
                                   : "drag a position (Ctrl+click to type it) to take the robot");
    }

    std::string const tool = robot.tool();
    if (!tool.empty())
    {
        robotik::Pose const pose = robot.framePose(tool);
        ImGui::TextDisabled("%s  x %.3f  y %.3f  z %.3f m", tool.c_str(), pose.position.x,
                            pose.position.y, pose.position.z);
    }
    if (auto const* gripper = robot.actuators().first<robotik::VacuumGripper>())
    {
        ImGui::SameLine();
        if (gripper->holding())
        {
            ImGui::TextColored(ORANGE, "  vacuum holds %s",
                               compages::world::Entity(robot.world(), gripper->held()).name().c_str());
        }
        else
        {
            ImGui::TextDisabled(gripper->suction() ? "  vacuum on, empty" : "  vacuum off");
        }
    }
}

void positionCell(App& p_app, robotik::JointId p_id, JointInfo const& p_info)
{
    robotik::RobotSession& robot = p_app.simulation->robot();
    robotik::JointSet& joints = robot.joints();
    double const position = joints.position(p_id);
    bool const commanded = joints.mode(p_id) == robotik::JointMode::Position;
    double const target = commanded ? joints.target(p_id) : position;
    double const lower = p_info.bounded ? p_info.lower : position - std::numbers::pi;
    double const upper = p_info.bounded ? p_info.upper : position + std::numbers::pi;
    char label[48];
    std::snprintf(label, sizeof label, p_info.linear ? "%.3f %s" : "%.1f%s", position * p_info.unit,
                  p_info.position_unit);
    if (!p_app.owns_clock)
    {
        // The bar is the command: dragging it, or Ctrl+click to type a value,
        // takes the robot from the mission (Manual mode).
        double const shown = p_app.teach.manual ? target : position;
        float value = static_cast<float>(shown * p_info.unit);
        ImGui::SetNextItemWidth(-1.0f);
        if (ImGui::SliderFloat("##target", &value, static_cast<float>(lower * p_info.unit),
                               static_cast<float>(upper * p_info.unit), label,
                               ImGuiSliderFlags_AlwaysClamp))
        {
            takeArm(p_app);
            double const from = joints.mode(p_id) == robotik::JointMode::Position ? joints.target(p_id)
                                                                                   : joints.position(p_id);
            p_app.teach.pendant.jogJoint(robot, p_id, static_cast<double>(value) / p_info.unit - from);
        }
    }
    else
    {
        bool const off = joints.mode(p_id) == robotik::JointMode::Disabled;
        rangeBar(position, lower, upper, target, commanded, label, off ? RED : BLUE);
    }
    if (ImGui::BeginItemTooltip())
    {
        ImGui::Text("position %.4f %s", position * p_info.unit, p_info.position_unit);
        if (!p_app.owns_clock)
        {
            ImGui::TextDisabled("Drag, or Ctrl+click to type a value.");
        }
        if (commanded)
        {
            ImGui::Text("target   %.4f %s  (orange tick)", target * p_info.unit, p_info.position_unit);
        }
        else
        {
            ImGui::TextUnformatted(joints.mode(p_id) == robotik::JointMode::Disabled ? "not commanded"
                                                                                     : "velocity or effort mode");
        }
        if (!p_info.bounded)
        {
            ImGui::TextDisabled("no position limit");
        }
        ImGui::EndTooltip();
    }
}

void jointsTable(App& p_app)
{
    robotik::RobotSession& robot = p_app.simulation->robot();
    robotik::JointSet& joints = robot.joints();
    if (p_app.ui.plotted.size() != joints.size())
    {
        p_app.ui.plotted.assign(joints.size(), joints.size() <= 8u);
    }
    std::size_t const latest = p_app.history.latest();
    ImGuiTableFlags const flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                                  ImGuiTableFlags_Resizable | ImGuiTableFlags_ScrollY |
                                  ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_Hideable;
    // A new robot has other joint names: give the table a new id so the
    // name column is sized again.
    ImGui::PushID(robot.name().c_str());
    if (!beginTable("joints", 9, flags))
    {
        ImGui::PopID();
        return;
    }
    float name_width = ImGui::CalcTextSize("Joint").x;
    for (robotik::JointId id = 0; id < joints.size(); ++id)
    {
        name_width = std::max(name_width, ImGui::CalcTextSize(joints.name(id).c_str()).x);
    }
    ImGui::TableSetupScrollFreeze(2, 1);
    ImGui::TableSetupColumn("Plot", ImGuiTableColumnFlags_WidthFixed);
    ImGui::TableSetupColumn("Joint", ImGuiTableColumnFlags_WidthFixed, name_width);
    ImGui::TableSetupColumn("Position", ImGuiTableColumnFlags_WidthStretch, 2.2f);
    ImGui::TableSetupColumn("Min", ImGuiTableColumnFlags_WidthFixed);
    ImGui::TableSetupColumn("Max", ImGuiTableColumnFlags_WidthFixed);
    ImGui::TableSetupColumn("Velocity / limit", ImGuiTableColumnFlags_WidthStretch, 1.4f);
    ImGui::TableSetupColumn("Accel.", ImGuiTableColumnFlags_WidthFixed);
    ImGui::TableSetupColumn("Effort / limit", ImGuiTableColumnFlags_WidthStretch, 1.4f);
    ImGui::TableSetupColumn("Jog", ImGuiTableColumnFlags_WidthFixed);
    ImGui::TableHeadersRow();
    for (robotik::JointId id = 0; id < joints.size(); ++id)
    {
        JointInfo const info = infoOf(joints, id);
        ImGui::PushID(static_cast<int>(id));
        ImGui::TableNextRow();

        ImGui::TableNextColumn();
        bool plotted = p_app.ui.plotted[id];
        if (ImGui::Checkbox("##plot", &plotted))
        {
            p_app.ui.plotted[id] = plotted;
        }

        ImGui::TableNextColumn();
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted(joints.name(id).c_str());

        ImGui::TableNextColumn();
        positionCell(p_app, id, info);

        ImGui::TableNextColumn();
        ImGui::AlignTextToFramePadding();
        if (info.bounded)
        {
            ImGui::TextDisabled(info.linear ? "%.3f" : "%.0f°", info.lower * info.unit);
        }
        else
        {
            ImGui::TextDisabled("-");
        }
        ImGui::TableNextColumn();
        ImGui::AlignTextToFramePadding();
        if (info.bounded)
        {
            ImGui::TextDisabled(info.linear ? "%.3f" : "%.0f°", info.upper * info.unit);
        }
        else
        {
            ImGui::TextDisabled("-");
        }

        char label[64];
        ImGui::TableNextColumn();
        double const velocity = joints.velocity(id);
        double const vmax = finiteOr(info.velocity, std::max(std::abs(velocity), 1.0));
        if (std::isfinite(info.velocity))
        {
            std::snprintf(label, sizeof label, "%.1f / %.0f %s", velocity * info.unit, info.velocity * info.unit,
                          info.velocity_unit);
        }
        else
        {
            std::snprintf(label, sizeof label, "%.1f %s", velocity * info.unit, info.velocity_unit);
        }
        bool const fast = std::isfinite(info.velocity) && std::abs(velocity) >= 0.98 * info.velocity;
        rangeBar(velocity, -vmax, vmax, 0.0, false, label, fast ? ORANGE : GREEN);

        ImGui::TableNextColumn();
        ImGui::AlignTextToFramePadding();
        double const accel = latest < JointHistory::CAPACITY && id < p_app.history.acceleration.size()
                                 ? static_cast<double>(p_app.history.acceleration[id][latest])
                                 : 0.0;
        ImGui::Text("%.0f", accel * info.unit);
        ImGui::SetItemTooltip("Acceleration, %s, from two velocity samples.\nNo limit is declared in the URDF.",
                              info.accel_unit);

        ImGui::TableNextColumn();
        double const effort = joints.effort(id);
        double const emax = finiteOr(info.effort, std::max(std::abs(effort), 1.0));
        if (std::isfinite(info.effort))
        {
            std::snprintf(label, sizeof label, "%.1f / %.0f %s", effort, info.effort, info.effort_unit);
        }
        else
        {
            std::snprintf(label, sizeof label, "%.1f %s", effort, info.effort_unit);
        }
        bool const saturated = std::isfinite(info.effort) && std::abs(effort) >= info.effort;
        rangeBar(effort, -emax, emax, 0.0, false, label, saturated ? RED : GREEN);
        if (saturated)
        {
            ImGui::SetItemTooltip("Saturated: the motor gives all it can.");
        }

        ImGui::TableNextColumn();
        ImGui::BeginDisabled(p_app.owns_clock);
        double const step = info.linear ? static_cast<double>(p_app.teach.linear_step)
                                        : static_cast<double>(p_app.teach.joint_step);
        if (ImGui::SmallButton(" - "))
        {
            takeArm(p_app);
            p_app.teach.pendant.jogJoint(robot, id, -step);
        }
        ImGui::SameLine(0.0f, 2.0f);
        if (ImGui::SmallButton(" + "))
        {
            takeArm(p_app);
            p_app.teach.pendant.jogJoint(robot, id, step);
        }
        ImGui::EndDisabled();
        ImGui::PopID();
    }
    endTable();
    ImGui::PopID();
}

// --- Plots ------------------------------------------------------------------

struct Series
{
    JointHistory const* history;
    std::vector<float> const* values;
    double scale;
};

ImPlotPoint sample(int p_index, void* p_data)
{
    auto const* series = static_cast<Series const*>(p_data);
    std::size_t const slot =
        (series->history->head + static_cast<std::size_t>(p_index)) % JointHistory::CAPACITY;
    return ImPlotPoint(static_cast<double>(series->history->time[slot]),
                       static_cast<double>((*series->values)[slot]) * series->scale);
}

} // namespace

void robotPanel(App& p_app)
{
    if (!ImGui::Begin("Joints"))
    {
        ImGui::End();
        return;
    }
    if (!p_app.simulation)
    {
        emptyState("No scenario loaded.");
        ImGui::End();
        return;
    }
    header(p_app);
    ImGui::Spacing();
    jointsTable(p_app);
    ImGui::End();
}

void plotsPanel(App& p_app)
{
    if (!ImGui::Begin("Joint plots"))
    {
        ImGui::End();
        return;
    }
    if (!p_app.simulation || p_app.history.count == 0u)
    {
        emptyState("No data yet: press Play.");
        ImGui::End();
        return;
    }
    robotik::JointSet const& joints = p_app.simulation->robot().joints();
    JointHistory const& history = p_app.history;
    std::size_t selected = 0;
    robotik::JointId only = 0;
    for (robotik::JointId id = 0; id < joints.size() && id < p_app.ui.plotted.size(); ++id)
    {
        if (p_app.ui.plotted[id])
        {
            ++selected;
            only = id;
        }
    }
    ImGui::SetNextItemWidth(160.0f);
    ImGui::SliderFloat("##window", &p_app.ui.plot_window_s, 2.0f, 30.0f, "last %.0f s");
    ImGui::SameLine();
    ImGui::TextDisabled(selected == 1u ? "red lines: limits of %s"
                                       : "tick the Plot column of the Robot panel%s",
                        selected == 1u ? joints.name(only).c_str() : "");
    if (selected == 0u)
    {
        ImGui::End();
        return;
    }

    double const now = static_cast<double>(history.time[history.latest()]);
    double const from = now - static_cast<double>(p_app.ui.plot_window_s);
    static constexpr char const* TITLES[] = { "Position", "Velocity", "Acceleration", "Effort" };
    std::vector<std::vector<float>> const* const SERIES[] = { &history.position, &history.velocity,
                                                               &history.acceleration, &history.effort };
    ImPlotSubplotFlags const flags = ImPlotSubplotFlags_LinkAllX | ImPlotSubplotFlags_ShareItems;
    ImVec2 const area = ImGui::GetContentRegionAvail();
    // Two by two when there is room for it, one column otherwise.
    int const columns = area.x > 1.6f * area.y ? 2 : 1;
    if (ImPlot::BeginSubplots("##joints", 4 / columns, columns, area, flags))
    {
        for (int row = 0; row < 4; ++row)
        {
            if (!ImPlot::BeginPlot(TITLES[row], ImVec2(-1.0f, 0.0f), ImPlotFlags_NoMenus))
            {
                continue;
            }
            ImPlot::SetupAxes(nullptr, nullptr, 0, ImPlotAxisFlags_AutoFit);
            ImPlot::SetupAxisLimits(ImAxis_X1, from, now, p_app.playing ? ImPlotCond_Always : ImPlotCond_Once);
            for (robotik::JointId id = 0; id < joints.size() && id < p_app.ui.plotted.size(); ++id)
            {
                if (!p_app.ui.plotted[id] || id >= SERIES[row]->size())
                {
                    continue;
                }
                JointInfo const info = infoOf(joints, id);
                // Efforts stay in N or N·m; motions use degrees for angles.
                Series series{ &history, &(*SERIES[row])[id], row == 3 ? 1.0 : info.unit };
                ImPlot::PlotLineG(joints.name(id).c_str(), &sample, &series, static_cast<int>(history.count));
                if (selected == 1u)
                {
                    double limits[2] = { 0.0, 0.0 };
                    int count = 0;
                    if (row == 0 && info.bounded)
                    {
                        limits[0] = info.lower * info.unit;
                        limits[1] = info.upper * info.unit;
                        count = 2;
                    }
                    else if (row == 1 && std::isfinite(info.velocity))
                    {
                        limits[0] = -info.velocity * info.unit;
                        limits[1] = info.velocity * info.unit;
                        count = 2;
                    }
                    else if (row == 3 && std::isfinite(info.effort))
                    {
                        limits[0] = -info.effort;
                        limits[1] = info.effort;
                        count = 2;
                    }
                    if (count > 0)
                    {
                        ImPlotSpec spec;
                        spec.LineColor = ImVec4(0.95f, 0.35f, 0.35f, 0.6f);
                        spec.Flags = static_cast<int>(ImPlotInfLinesFlags_Horizontal) | static_cast<int>(ImPlotItemFlags_NoLegend);
                        ImPlot::PlotInfLines("##limits", limits, count, spec);
                    }
                }
            }
            ImPlot::EndPlot();
        }
        ImPlot::EndSubplots();
    }
    ImGui::End();
}
