/***************************************************************************
 *  asp_planer_doc_and_config.cpp - ASP-based planer plugin documentation
 *
 *  Created on Fri Dec 16 10:47:02 2016
 *  Copyright (C) 2016 by Björn Schäpers
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#include "asp_planer_thread.h"

#include <plugins/asp/aspect/clingo_access.h>

#include <algorithm>
#include <cstring>

/**
 * @typedef Clock
 * @brief The used clock for real time measurment. In contrast to simulated time.
 *
 * @typedef TimePoint
 * @brief The with the clock assoziated type to identify time points.
 */

/**
 * @struct BasicPlanElement
 * @brief The basis of an element for the plan.
 *
 * @property BasicPlanElement::Task
 * @brief The task with it's parameters.
 *
 * @property BasicPlanElement::Begin
 * @brief The game time on which the task should start.
 *
 * @property BasicPlanElement::End
 * @brief The estimated end time for the task, if available. If not it is set to zero.
 */

/**
 * @struct PlanElement
 * @brief The saved plan element.
 *
 * @property PlanElement::Begun
 * @brief Wether the task execution on the robot has started or not.
 *
 * @property PlanElement::Done
 * @brief Wether the task is done or not.
 *
 * @property PlanElement::Failed
 * @brief Wether the fask was failed or not.
 */

/**
 * @struct RobotPlan
 * @brief The plan for a robot.
 *
 * @property RobotPlan::Tasks
 * @brief The list of PlanElements forming the plan.
 *
 * @property RobotPlan::FirstNotDone
 * @brief The index in the plan for the first task which is not done.
 *
 * @property RobotPlan::FirstNotDoneOnSolveStart
 * @brief The index in the plan for the first task which was not done, when the solving started.
 *
 * @property RobotPlan::CurrentTask
 * @brief The task, the robot should be doing currently.
 */

/**
 * @struct TaskDescription
 * @brief The description of a task, a robot is executing.
 *
 * @property TaskDescription::Type
 * @brief The tasks type.
 *
 * @property TaskDescription::TaskSymbol
 * @brief The clingo symbol for the task, as cached value.
 *
 * @property TaskDescription::EstimatedEnd
 * @brief The real time seconds, we estimate the robot to be finished.
 *
 * @fn bool TaskDescription::isValid() const
 * @brief If a valid task is identified.
 *
 * @fn Clingo::Sybol TaskDescription::location() const
 * @brief Returns the location of the task.
 */

/**
 * @struct ProductIdentifier
 * @brief Type safe wrapper for product id.
 *
 * @property ProductIdentifier::ID
 * @brief The ID of the product, this is just an array index.
 *
 * @fn bool ProductIdentifier::isValid() const
 * @brief If a valid product is identified.
 */

/**
 * @struct Product
 * @brief Container to track a product.
 *
 * @property Product::Base
 * @brief The base color.
 *
 * @property Product::Rings
 * @brief The colors of the rings.
 *
 * @property Product::Cap
 * @brief The color of the cap.
 */

/**
 * @struct RobotInformation
 * @brief Stores information for a robot.
 *
 * @property RobotInformation::LastSeen
 * @brief When did we last hear of the robot.
 *
 * @property RobotInformation::Alive
 * @brief If the robot is considered alive.
 *
 * @property RobotInformation::AliveExternal
 * @brief The external if a robot is alive or not.
 *
 * @property RobotInformation::X
 * @brief The reported x coordinate of the robot.
 *
 * @property RobotInformation::Y
 * @brief The reported y coordinate of the robot.
 *
 * @property RobotInformation::Holding
 * @brief Identifies what the robot is holding.
 *
 * @property RobotInformation::Doing
 * @brief Identifies which task the robot is doing right now.
 */

/**
 * @struct MachineInformation
 * @brief Stores information about a machine.
 *
 * @property MachineInformation::BrokenUntil
 * @brief Until when do we think the machine is broken.
 *
 * @property MachineInformation::WorkingUntil
 * @brief Until when do we think the machine will work. If BrokenUntil is set we save the duration we add after the
 *        machne is not longer broken.
 *
 * @property MachineInformation::Storing
 * @brief The product which the machine processes or stores on the output side.
 *
 * @property MachineInformation::FillState
 * @brief For the ring stations, how many bases are loaded in it.
 *
 * @property MachineInformation::Prepared
 * @brief For the cap stations, if they are prepared to mount a cap.
 *
 * @property MachineInformation::State
 * @brief The last reported state of the machine.
 */

/**
 * @struct RingColorInformation
 * @brief Stores information about a ring color.
 *
 * @property RingColorInformation::Color
 * @brief The name of the color.
 *
 * @property RingColorInformation::Machine
 * @brief Which machine serves the color.
 *
 * @property RingColorInformation::Cost
 * @brief How many additional bases are needed to get the color.
 */

/**
 * @struct CapColorInformation
 * @brief The mapping of a cap color to the station.
 *
 * @property CapColorInformation::Color
 * @brief The color of the cap.
 *
 * @property CapColorInformation::Machine
 * @brief The machine.
 */

/**
 * @struct OrderInformation
 * @brief Stores information about orders.
 *
 * @property OrderInformation::Number
 * @brief The order number.
 *
 * @property OrderInformation::Quantity
 * @brief How many products for the order can be delivered.
 *
 * @property OrderInformation::Base
 * @brief The base color of the ordered product.
 *
 * @property OrderInformation::Cap
 * @brief The cap color of the ordered product.
 *
 * @property OrderInformation::Rings
 * @brief The ring colors of the ordered product, can be "none".
 *
 * @property OrderInformation::DeliveryBegin
 * @brief The begin of the delivery time window.
 *
 * @property OrderInformation::DeliveryEnd
 * @brief The end of the delivery time window.
 */

/**
 * @struct OrderTasks
 * @brief Stores the task externals assoziated with an order.
 *
 * @property OrderTasks::RingTasks
 * @brief The tasks for the rings.
 *
 * @property OrderTasks::CapTask
 * @brief The task for the cap.
 *
 * @property OrderTasks::DeliverTasks
 * @brief The task for the delivery, the first is the normal and the second is the late delivery.
 */

/**
 * @enum InterruptSolving
 * @brief States the current interrupt request.
 * @note Sort by priority. We use operator> when setting the value.
 *
 * @var InterruptSolving::Not
 * @brief Do not interrupt. But when a robot is too much behind schedule increase interrupt level and check again.
 *
 * @var InterruptSolving::JustStarted
 * @brief Only interrupt if the solving was just started.
 *
 * @var InterruptSolving::Normal
 * @brief Do interrupt, if the plan isn't to old.
 *
 * @var InterruptSolving::High
 * @brief Like normal, but with a smaller threshold.
 *
 * @var InterruptSolving::Critical
 * @brief Interrupt in any case.
 *
 * @fn const char* interruptString(const InterruptSolving)
 * @brief Converts the enum to a string.
 * @param[in] interrupt The enum value.
 * @return The value represented as string.
 */

/** @class AspPlanerThread "asp_planer_thread.h"
 * The thread to start and control the ASP planer.
 *
 * @property AspPlanerThread::LoggingComponent
 * @brief The component name for the logging facility.
 *
 * @property AspPlanerThread::ConfigPrefix
 * @brief The prefix for the config access.
 *
 * @property AspPlanerThread::RobotMemoryCallbacks
 * @brief Contains all registered callbacks in the robot memory.
 *
 * @property AspPlanerThread::TeamColor
 * @brief The team color for us as string, either "C" or "M".
 */

/**
 * @property AspPlanerThread::Unsat
 * @brief How often we were told the program was unsatisfiable in a row. If it gets to high we have to take recovering
 *        actions.
 *
 * @property AspPlanerThread::ExplorationTime
 * @brief The time for the exploration phase, in seconds.
 *
 * @property AspPlanerThread::DeliverProductTaskDuration
 * @brief How many seconds it will take a robot to put a product in a machine.
 *
 * @property AspPlanerThread::FetchProductTaskDuration
 * @brief How many seconds it will take a robot to get a product from a machine.
 *
 * @property AspPlanerThread::LookAhaed
 * @brief How many seconds the planer should look into the future.
 *
 * @property AspPlanerThread::MaxDriveDuration
 * @brief An upper bound for the time (in seconds) the robot has to drive between two locations.
 *
 * @property AspPlanerThread::MaxOrders
 * @brief The maximum amount of orders we expect.
 *
 * @property AspPlanerThread::MaxProducts
 * @brief The maximum amount of procuts which can be alive at any given point in time.
 *
 * @property AspPlanerThread::MaxQuantity
 * @brief The maximum quantity for an order we expect.
 *
 * @property AspPlanerThread::MaxTaskDuration
 * @brief An upper bound on the time (in seconds) for the execution of a task.
 *
 * @property AspPlanerThread::MaxWorkingDuration
 * @brief The longest working duration of the machines.
 *
 * @property AspPlanerThread::PossibleRobots
 * @brief The names of the robots we may have.
 *
 * @property AspPlanerThread::PrepareCSTaskDuration
 * @brief How many seconds it will take a robot to prepare a cap station.
 *
 * @property AspPlanerThread::ProductionEnd
 * @brief At which time point the production phase will end.
 *
 * @property AspPlanerThread::TimeResolution
 * @brief How many real time seconds will be one asp time step.
 *
 * @property AspPlanerThread::WorkingDuration
 * @brief The mapping from machine type to its working duration.
 */

/**
 * @property AspPlanerThread::BaseColors
 * @brief All available base colors.
 *
 * @property AspPlanerThread::SpecialBaseColor
 * @brief The base color, for the dummy products on the cap stations shelf.
 *
 * @property AspPlanerThread::CapColors
 * @brief The cap colors and their machine matching.
 *
 * @property AspPlanerThread::WorldMutex
 * @brief The mutex for the world model, including the robot informations.
 *
 * @property AspPlanerThread::GameTime
 * @brief The current game time (as reported by the refbox) in (floored) seconds.
 *
 * @property AspPlanerThread::Orders
 * @brief The information about the orders.
 *
 * @property AspPlanerThread::RingColors
 * @brief The information about the ring colors.
 *
 * @property AspPlanerThread::Robots
 * @brief The robot information in a lookup table.
 *
 * @property AspPlanerThread::Machines
 * @brief The machine information in a lookup table.
 *
 * @property AspPlanerThread::Products
 * @brief The products currently used.
 *
 * @property AspPlanerThread::ReceivedZonesToExplore
 * @brief If we have received the info which zones we have to explore.
 *
 * @property AspPlanerThread::ZonesToExplore
 * @brief Which zones we have to explore.
 *
 * @property AspPlanerThread::OrderTaskMap
 * @brief The mapping of (order #, quantitiy #) to the task external.
 */

/**
 * @property AspPlanerThread::NodePropertyASP
 * @brief The string for the asp node property. All nodes we export to ASP are marked with this.
 *
 * @property AspPlanerThread::NavgraphDistanceMutex
 * @brief The mutex for all distance stuff.
 *
 * @property AspPlanerThread::NavgraphNodesForASP
 * @brief A mapping from the navgraph node name to its asp atom.
 *
 * @property AspPlanerThread::NodesToFind
 * @brief The navgraph nodes we still have to find. If we found all we can release the driveDruation externals.
 *
 * @property AspPlanerThread::UpdateNavgraphDistances.
 * @brief If the distances have to be updated.
 *
 * @property AspPlanerThread::NavgraphDistances.
 * @brief The externals with the distance, we export to ASP.
 */

/**
 * @property AspPlanerThread::DeliveryLocation
 * @brief The location where a delivery may take place.
 *
 * @property AspPlanerThread::CapLocations
 * @brief The locations where a mount cap task may take place.
 *
 * @property AspPlanerThread::RingLocation
 * @brief The locations where a mount ring task may take place.
 */

/**
 * @typedef AspPlanerThread::GroundRequest
 * @brief Holds the information to build a Clingo::Part.
 *
 * @property AspPlanerThread::Interrupt
 * @brief The current interrupt state.
 *
 * @property AspPlanerThread::InterruptReason
 * @brief The last set reason for the interrupt.
 *
 * @property AspPlanerThread::RequestMutex
 * @brief The mutex for the requests and interupt handling.
 *
 * @property AspPlanerThread::SentCancel
 * @brief If a cancel was sent to the solver.
 *
 * @property AspPlanerThread::GroundRequests
 * @brief Requests for new groundings.
 *
 * @property AspPlanterThread::ReleaseRequests
 * @brief Requests for releasing externals.
 *
 * @property AspPlanterThread::AssignRequests
 * @brief Requests for assigning true to externals.
 */

/**
 * @property AspPlanerThread::ProgramGrounded
 * @brief If we have grounded our program.
 *
 * @property AspPlanerThread::ProductionStarted
 * @brief If the production phase has started.
 */

/**
 * @property AspPlanerThread::SolvingMutex
 * @brief Mutex for some solving related values.
 *
 * @property AspPlanerThread::LastModel
 * @brief When we received the last model.
 *
 * @property AspPlanerThread::SolvingStarted
 * @brief When we started with the solving.
 *
 * @property AspPlanerThread::NewSymbols
 * @brief Denotes if there are new (unprocessed) symbols.
 *
 * @property AspPlanerThread::Symbols
 * @brief The symbols from the last model.
 *
 * @property AspPlanerThread::StartSolvingGameTime
 * @brief The game time, when the solving was started.
 */

/**
 * @property AspPlanerThread::PlanMutex
 * @brief The mutex for the plan.
 *
 * @property AspPlanerThread::LastPlan
 * @brief When we extracted the last plan.
 *
 * @property AspPlanerThread::PlanGameTime
 * @brief The game time, when the model was read.
 *
 * @property AspPlanerThread::Plan
 * @brief The plan currently deployed.
 *
 * @property AspPlanerThread::LocationInUse
 * @brief Mapping from a location to the robot which uses it, to detect errors in the plan.
 */

/**
 * @brief Reads the config and fills the members.
 */
void
AspPlanerThread::loadConfig(void)
{
	constexpr auto infixPlaner = "planer/";
	constexpr auto infixTime = "time-estimations/";
	constexpr auto infixCapStation = "cap-station/assigned-color/";
	constexpr auto infixWorkingDuration = "working-durations/";

	constexpr auto infixPlanerLen = std::strlen(infixPlaner), infixTimeLen = std::strlen(infixTime);
	constexpr auto infixCapStationLen = std::strlen(infixCapStation);
	constexpr auto infixWorkingDurationLen = std::strlen(infixWorkingDuration);
	const auto prefixLen = std::strlen(ConfigPrefix);

	char buffer[prefixLen +
		std::max<size_t>({infixPlanerLen, infixTimeLen, infixCapStationLen, infixWorkingDurationLen}) + 20];
	std::strcpy(buffer, ConfigPrefix);

	//The plain part.
	auto suffix = buffer + prefixLen;
	std::strcpy(suffix, "exploration-time");
	ExplorationTime = config->get_int(buffer);

	std::strcpy(suffix, "production-end");
	ProductionEnd = ExplorationTime + config->get_int(buffer);

	//The planer part.
	suffix = buffer + prefixLen + infixPlanerLen;
	std::strcpy(buffer + prefixLen, infixPlaner);

	std::strcpy(suffix, "debug-level");
	ClingoAcc->DebugLevel = static_cast<fawkes::ClingoAccess::DebugLevel_t>(config->get_int(buffer));

	std::strcpy(suffix, "max-orders");
	MaxOrders = config->get_int(buffer);
	std::strcpy(suffix, "max-products");
	MaxProducts = config->get_int(buffer);
	std::strcpy(suffix, "max-quantity");
	MaxQuantity = config->get_int(buffer);
	std::strcpy(suffix, "look-ahaed");
	LookAhaed = config->get_int(buffer);
	std::strcpy(suffix, "time-resolution");
	TimeResolution = config->get_int(buffer);
	std::strcpy(suffix, "robots");
	PossibleRobots = config->get_strings(buffer);

	//The time-estimation part.
	suffix = buffer + prefixLen + infixTimeLen;
	std::strcpy(buffer + prefixLen, infixTime);

	std::strcpy(suffix, "deliver-product");
	DeliverProductTaskDuration = config->get_int(buffer);
	std::strcpy(suffix, "fetch-product");
	FetchProductTaskDuration = config->get_int(buffer);
	std::strcpy(suffix, "max-drive-duration");
	MaxDriveDuration = config->get_int(buffer);
	std::strcpy(suffix, "prepare-cs");
	PrepareCSTaskDuration = config->get_int(buffer);

	MaxTaskDuration = std::max({DeliverProductTaskDuration, FetchProductTaskDuration, PrepareCSTaskDuration});

	//The cap-station part.
	suffix = buffer + prefixLen + infixCapStationLen;
	std::strcpy(buffer + prefixLen, infixCapStation);

	CapColors.reserve(2);
	//We assume the distribution is the same, for CYAN and MAGENTA.
	std::strcpy(suffix, "C-CS1");
	CapColors.emplace_back(CapColorInformation{config->get_string(buffer), "CS1"});
	std::strcpy(suffix, "C-CS2");
	CapColors.emplace_back(CapColorInformation{config->get_string(buffer), "CS2"});

	//The working-duration part.
	suffix = buffer + prefixLen + infixWorkingDurationLen;
	std::strcpy(buffer + prefixLen, infixWorkingDuration);

	WorkingDurations.reserve(6);
	std::strcpy(suffix, "base-station");
	WorkingDurations.insert({"BS", config->get_int(buffer)});
	std::strcpy(suffix, "cap-station");
	WorkingDurations.insert({"CS1", config->get_int(buffer)});
	WorkingDurations.insert({"CS2", config->get_int(buffer)});
	std::strcpy(suffix, "delivery-station");
	WorkingDurations.insert({"DS", config->get_int(buffer)});
	std::strcpy(suffix, "ring-station");
	WorkingDurations.insert({"RS1", config->get_int(buffer)});
	WorkingDurations.insert({"RS2", config->get_int(buffer)});

	for ( const auto& pair : WorkingDurations )
	{
		MaxWorkingDuration = std::max(MaxWorkingDuration, pair.second);
	} //for ( const auto& pair : WorkingDurations )
	return;
}
