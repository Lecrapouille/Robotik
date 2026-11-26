// SPDX-License-Identifier: GPL-3.0-or-later OR LicenseRef-RobotIK-Commercial
// Copyright (c) 2020-2026 Quentin Quadrat
//
// This file is part of RobotIK. It is available under the GNU GPL v3 or,
// for users who cannot use the GPL, under a commercial license.
// See LICENSING.md for details.

/// @file Story.hpp
/// Plays the story of a scenario: moves the actors of the world (people,
/// vehicles) as the events of the @c story: section say, in simulated time.
///
/// The @ref RoboticCell owns one and steps it with the world, before the
/// scenario logic: the actors are part of what the world does by itself,
/// like the @c simulate side of a scenario logic. No skill commands an
/// actor, and an actor never commands a robot. Conditions read the state
/// of the story with the queries @c (event_done e), @c (said who id),
/// @c (story_var name)... (doc/reference/story-file.md).
///
/// @par Contract
/// - Promises: the same seed gives the same story, step for step; it runs
///   headless (without a view, the actors are points that move).
/// - Expects: built after the scenario logic added its queries and metrics;
///   reset after the cell placed the actors.
/// - Errors: a condition that does not compile throws
///   @c std::invalid_argument at construction (the cell turns it into a
///   load error); a timeout fails the event, not the run.
#pragma once

#include "RobotIK/Common/Math.hpp"
#include "RobotIK/Common/Random.hpp"
#include "RobotIK/Task/Condition.hpp"
#include "RobotIK/Task/Metrics.hpp"
#include "RobotIK/Task/Story/StoryDescription.hpp"

#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace robotik
{

class RoboticCell;
class SceneView;
struct WorldDescription;

// ****************************************************************************
/// A sentence said by an actor (or, later, a robot): what a microphone would
/// hear. The story keeps them all, in order.
// ****************************************************************************
struct Utterance
{
    /// When it started (simulated s).
    Seconds time{ 0.0 };
    /// Who said it: an entity id.
    std::string speaker;
    /// Id of the sentence, for @c (said speaker id).
    std::string id;
    /// The words.
    std::string text;
    /// Whom it was said to, or empty.
    std::string to;
};

// ****************************************************************************
/// Where an event of the story is.
// ****************************************************************************
enum class EventPhase : std::uint8_t
{
    Waiting,   ///< Its start condition has not become true (or not again).
    Pending,   ///< Started, but its actor plays another event.
    Delaying,  ///< Started, waiting its @c delay.
    Running,   ///< Playing its actions.
    Finished,  ///< Done as many times as @c repeat allows.
};

// ****************************************************************************
/// Plays the events of a story on the actors of the world.
///
/// Why: a scenario of a robot among people needs the people to do the same
/// thing at each run (a test), or a drawn variant of it (an experiment
/// with seeds), without writing C++. When: the scenario has a @c story:
/// section. How: at each step, each event checks its start condition,
/// then plays its current action; a move walks on arcs at the speed of its
/// gait and tells the view which clip to play.
///
/// @code
/// // RoboticCell::step, after the physics:
/// m_story->step(*this, p_dt);
/// // a test:
/// EXPECT_EQ(cell.requireQuery("event_done", { "arrive" }), 1.0);
/// @endcode
///
/// Limits: no path planning (an actor walks straight, through walls and
/// robots); one event per actor at a time.
// ****************************************************************************
class Story
{
public:

    // ------------------------------------------------------------------------
    /// Compiles the conditions of the story.
    /// @param p_spec The story; kept by reference (the cell owns it).
    /// @param p_world The world, for the actors.
    /// @param p_compile Compiles a condition with the vocabulary of the cell.
    /// @throws std::invalid_argument naming the event of a wrong condition.
    // ------------------------------------------------------------------------
    Story(StorySpec const& p_spec,
          WorldDescription const& p_world,
          std::function<Result<Condition>(std::string_view)> const& p_compile);

    // ------------------------------------------------------------------------
    /// Back to the start: events waiting, variables at their first value,
    /// no sentence said. The actors are where the cell placed them.
    /// @param p_cell The cell, already reset.
    /// @param p_view What draws the actors, or null.
    /// @param p_seed Seed of the drawn times and branches.
    // ------------------------------------------------------------------------
    void reset(RoboticCell& p_cell, SceneView* p_view, Seed p_seed);

    // ------------------------------------------------------------------------
    /// Plays one step of every event and moves the actors.
    /// @param p_cell The cell: poses of the targets, conditions.
    /// @param p_view What draws the actors, or null (headless).
    /// @param p_dt Simulated time of the step.
    // ------------------------------------------------------------------------
    void step(RoboticCell& p_cell, SceneView* p_view, Seconds p_dt);

    /// @return Every sentence said since the reset, in order.
    [[nodiscard]] std::vector<Utterance> const& utterances() const
    {
        return m_utterances;
    }

    /// @return What happened, one line each ("2.40 s arrive: started").
    [[nodiscard]] std::vector<std::string> const& log() const
    {
        return m_log;
    }

    // --- Answers of the queries ---------------------------------------------

    /// @return Phase of an event; throws std::invalid_argument when unknown.
    [[nodiscard]] EventPhase requirePhase(std::string_view p_event) const;
    /// @return Times the event started; throws when unknown.
    [[nodiscard]] std::uint32_t requireStarts(std::string_view p_event) const;
    /// @return Times the event played all its actions; throws when unknown.
    [[nodiscard]] std::uint32_t requireCompletions(std::string_view p_event) const;
    /// @return Times the event failed (timeout) or was cancelled.
    [[nodiscard]] std::uint32_t requireFailures(std::string_view p_event) const;
    /// @return Value of a story variable; throws when unknown.
    [[nodiscard]] double requireVariable(std::string_view p_name) const;
    /// @return True when @p_speaker said the sentence @p_id.
    [[nodiscard]] bool said(std::string_view p_speaker, std::string_view p_id) const;
    /// @return Speed of an actor now (m/s); throws when it is no actor.
    [[nodiscard]] double requireSpeed(std::string_view p_actor) const;
    /// @return True while an actor is in the world (see the show action).
    [[nodiscard]] bool requireShown(std::string_view p_actor) const;
    /// @return True while an actor speaks.
    [[nodiscard]] bool requireSpeaking(std::string_view p_actor) const;
    /// @return Heading of an actor about Z (rad).
    [[nodiscard]] double requireHeading(std::string_view p_actor) const;
    /// @return True when @p_id names an actor.
    [[nodiscard]] bool isActor(std::string_view p_id) const;
    /// @return True when @p_name is a place of the story.
    [[nodiscard]] bool isPlace(std::string_view p_name) const;
    /// @return Weight of a face expression of an actor, 0 when not shown.
    [[nodiscard]] double expression(std::string_view p_actor, std::string_view p_name) const;
    /// @return Point of a place; throws when unknown.
    [[nodiscard]] Vector3 requirePlace(std::string_view p_name) const;

private:

    /// One level of actions being played (a branch of choose adds one).
    struct Frame
    {
        std::vector<StoryAction> const* actions = nullptr;
        std::size_t next = 0U;
    };

    /// What the current action of an event keeps between steps.
    struct Progress
    {
        bool begun = false;
        Seconds elapsed{ 0.0 };
        double length = 0.0;     ///< drawn duration of a wait
        std::size_t point = 0U;  ///< next point of follow_path
    };

    struct EventState
    {
        StoryEvent const* spec = nullptr;
        std::size_t actor = 0U; ///< index in m_actors, or NONE
        std::optional<Condition> start;
        std::optional<Condition> stop_when;
        EventPhase phase = EventPhase::Waiting;
        bool armed = false;        ///< start became true; waits for after/actor
        bool previous = false;     ///< start held at the last step
        bool forced = false;       ///< a start action: no wait for after
        std::uint32_t starts = 0U;
        std::uint32_t completions = 0U;
        std::uint32_t failures = 0U;
        Seconds delay_left{ 0.0 };
        std::vector<Frame> frames;
        Progress progress;
    };

    struct ActorState
    {
        std::string id;
        ActorSpec const* spec = nullptr;
        double heading = 0.0;
        double speed = 0.0;      ///< m/s this step
        bool shown = true;
        std::string role;        ///< clip role this step
        bool loop = true;
        std::size_t owner = NONE; ///< event playing it
        Seconds speaking_until{ -1.0 };
        std::string expression;
        double expression_weight = 0.0;
        Seconds expression_until{ -1.0 };
        bool face_set = false;   ///< the view was told the expression
    };

    /// Result of one step of an action.
    enum class Outcome : std::uint8_t
    {
        Busy,
        Done,
        Failed
    };

    static constexpr std::size_t NONE = static_cast<std::size_t>(-1);

    [[nodiscard]] std::size_t eventIndex(std::string_view p_event) const;
    [[nodiscard]] std::size_t actorIndex(std::string_view p_actor) const;
    [[nodiscard]] bool holds(Condition const& p_condition, RoboticCell const& p_cell) const;
    [[nodiscard]] Vector3 where(RoboticCell const& p_cell, StoryTarget const& p_target) const;
    [[nodiscard]] Vector3 positionOf(RoboticCell& p_cell, ActorState const& p_actor) const;
    void place(RoboticCell& p_cell, ActorState& p_actor, Vector3 const& p_at) const;
    void fire(RoboticCell& p_cell, EventState& p_event);
    void finish(RoboticCell& p_cell, EventState& p_event, std::string const& p_how);
    void advance(RoboticCell& p_cell, EventState& p_event, Seconds p_dt);
    [[nodiscard]] Outcome play(RoboticCell& p_cell,
                               EventState& p_event,
                               StoryAction const& p_action,
                               Seconds p_dt);
    [[nodiscard]] bool walk(RoboticCell& p_cell,
                            ActorState& p_actor,
                            Vector3 const& p_goal,
                            double p_stop,
                            double p_speed,
                            Seconds p_dt) const;
    bool turn(ActorState& p_actor, double p_heading, Seconds p_dt) const;
    void show(RoboticCell& p_cell, SceneView* p_view);
    void note(RoboticCell const& p_cell, std::string const& p_line);

    StorySpec const& m_spec;
    std::vector<EventState> m_events;
    std::vector<ActorState> m_actors;
    std::map<std::string, double, std::less<>> m_variables;
    std::vector<Utterance> m_utterances;
    std::vector<std::string> m_log;
    Random m_random;
    /// Conditions of wait_until and follow, compiled once.
    std::map<StoryAction const*, Condition> m_waits;
    /// Metrics of this step, for the conditions.
    Metrics m_metrics;
    /// Time of the cell this step.
    Seconds m_now{ 0.0 };
    /// What draws the actors this step, or null.
    SceneView* m_view = nullptr;
};

} // namespace robotik
