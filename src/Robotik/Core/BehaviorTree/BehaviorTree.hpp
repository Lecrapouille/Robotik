/**
 * @file BehaviorTree.hpp
 * @brief Main include file for the Robotik Behavior Tree library.
 *
 * Copyright (c) 2025 Quentin Quadrat <lecrapouille@gmail.com>
 * distributed under MIT License
 * @see https://github.com/Lecrapouille/Robotik
 */

#pragma once

// Core classes
#include "Robotik/Core/BehaviorTree/Core/Status.hpp"
#include "Robotik/Core/BehaviorTree/Core/Node.hpp"
#include "Robotik/Core/BehaviorTree/Core/Leaf.hpp"
#include "Robotik/Core/BehaviorTree/Core/Decorator.hpp"
#include "Robotik/Core/BehaviorTree/Core/Composite.hpp"
#include "Robotik/Core/BehaviorTree/Core/Tree.hpp"

// Blackboard
#include "Robotik/Core/BehaviorTree/Blackboard/Blackboard.hpp"
#include "Robotik/Core/BehaviorTree/Blackboard/Ports.hpp"
#include "Robotik/Core/BehaviorTree/Blackboard/Resolver.hpp"
#include "Robotik/Core/BehaviorTree/Blackboard/Serializer.hpp"

// Built-in nodes
#include "Robotik/Core/BehaviorTree/Nodes/Composites/Sequences.hpp"
#include "Robotik/Core/BehaviorTree/Nodes/Composites/Selectors.hpp"
#include "Robotik/Core/BehaviorTree/Nodes/Composites/Parallels.hpp"
#include "Robotik/Core/BehaviorTree/Nodes/Decorators/Logical.hpp"
#include "Robotik/Core/BehaviorTree/Nodes/Decorators/Repeat.hpp"
#include "Robotik/Core/BehaviorTree/Nodes/Decorators/Temporal.hpp"
#include "Robotik/Core/BehaviorTree/Nodes/Leaves/Basic.hpp"
#include "Robotik/Core/BehaviorTree/Nodes/Leaves/Action.hpp"
#include "Robotik/Core/BehaviorTree/Nodes/Leaves/Condition.hpp"
#include "Robotik/Core/BehaviorTree/Nodes/Leaves/Wait.hpp"
#include "Robotik/Core/BehaviorTree/Nodes/Leaves/SetBlackboard.hpp"

// Builder and Factory
#include "Robotik/Core/BehaviorTree/Builder/Factory.hpp"
#include "Robotik/Core/BehaviorTree/Builder/Builder.hpp"
#include "Robotik/Core/BehaviorTree/Builder/Exporter.hpp"

// Visitor
#include "Robotik/Core/BehaviorTree/Visitors/Visitor.hpp"

// Network (optional, for visualization)
#include "Robotik/Core/BehaviorTree/Network/VisualizerClient.hpp"
