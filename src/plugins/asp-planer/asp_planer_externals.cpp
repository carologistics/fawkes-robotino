/***************************************************************************
 *  asp_planer_externals.cpp - ASP-based planer externals
 *
 *  Created on Fri Dec 16 17:00:02 2016
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

#include "asp_planer_externals.h"
#include "asp_planer_types.h"

#include <clingo.hh>

/**
 * @brief Generates the product "name" for the following product externals.
 * @param[in] product The product.
 * @return The symbol.
 */
static inline Clingo::Symbol
productName(const ProductIdentifier& product)
{
	return Clingo::Function("product", {Clingo::Number(product.ID)});
}

/**
 * @brief Generates the external, used to say a robot is alive.
 * @param[in] robotName The name of the robot.
 * @return The atom for the external.
 */
Clingo::Symbol
generateAliveExternal(const std::string& robotName)
{
	return Clingo::Function("availableRobot", {Clingo::String(robotName)});
}

/**
 * @brief Generates the external for a robots location.
 * @param[in] robotName The name of the robot.
 * @param[in] location Its location.
 * @return The external atom.
 */
Clingo::Symbol
generateRobotLocationExternal(const std::string& robotName, const Clingo::Symbol& location)
{
	return Clingo::Function("robotLocation", {Clingo::String(robotName), location, Clingo::Number(0)});
}

/**
 * @brief Generates the external for a robot holding a product.
 * @param[in] robotName The name of the robot.
 * @param[in] product The product.
 * @return The external atom.
 */
Clingo::Symbol
generateHoldingExternal(const std::string& robotName, const ProductIdentifier& product)
{
	return Clingo::Function("holding", {Clingo::String(robotName), productName(product), Clingo::Number(0)});
}

/**
 * @brief Generates the external for a robot doing a task.
 * @param[in] robotName The name of the robot.
 * @param[in] task The tasks description.
 * @param[in] duration The remaining duration of the doing.
 * @return The external atom.
 */
Clingo::Symbol
generateDoingExternal(const std::string& robotName, const TaskDescription& task, const int duration)
{
	assert(duration > 0);
	return Clingo::Function("robotDoing", {Clingo::String(robotName), task.TaskSymbol, Clingo::Number(duration)});
}

/**
 * @brief Generates the external for a location of an explore task.
 * @param[in] zone The zone number.
 * @return The external atom.
 */
Clingo::Symbol
generateExploreLocationExternal(const int zone)
{
	return Clingo::Function("location", {Clingo::Function("z", {Clingo::Number(zone)})});
}

/**
 * @brief Generates the external for a location of an explore task.
 * @param[in] zone The zone number.
 * @return The external atom.
 */
Clingo::Symbol
generateExploreTaskExternal(const int zone)
{
	return Clingo::Function("toBeDone", {Clingo::Function("explore", {Clingo::Number(zone)}), Clingo::Number(0)});
}

/**
 * @brief Generates the external for a product.
 * @param[in] product The product..
 * @return The external atom.
 */
Clingo::Symbol
generateProductExternal(const ProductIdentifier& product)
{
	return Clingo::Function("product", {productName(product)});
}

/**
 * @brief Generates the external for a products base.
 * @param[in] product The product..
 * @param[in] base The base color.
 * @return The external atom.
 */
Clingo::Symbol
generateProductBaseExternal(const ProductIdentifier& product, const std::string& base)
{
	return Clingo::Function("productBase", {productName(product), Clingo::String(base)});
}

/**
 * @brief Generates the external for a products ring.
 * @param[in] product The product..
 * @param[in] ringNumber The ring number. (Counting from 1.)
 * @param[in] color The ring color.
 * @return The external atom.
 */
Clingo::Symbol
generateProductRingExternal(const ProductIdentifier& product, const int ringNumber, const std::string& color)
{
	return Clingo::Function("productRing", {productName(product), Clingo::Number(ringNumber), Clingo::String(color)});
}

/**
 * @brief Generates the external for a products cap.
 * @param[in] product The product..
 * @param[in] cap The cap color.
 * @return The external atom.
 */
Clingo::Symbol
generateProductCapExternal(const ProductIdentifier& product, const std::string& cap)
{
	return Clingo::Function("productCap", {productName(product), Clingo::String(cap)});
}

/**
 * @brief Generates the external for a broken machine.
 * @param[in] machineName The name of the machine.
 * @param[in] duration How long the machine is broken.
 * @return The external atom.
 */
Clingo::Symbol
generateMachineBrokenExternal(const std::string& machineName, const int duration)
{
	return Clingo::Function("broken", {Clingo::String(machineName), Clingo::Number(duration), Clingo::Number(0)});
}

/**
 * @brief Generates the external for a working machine.
 * @param[in] machineName The name of the machine.
 * @param[in] duration How long the machine is working on the product.
 * @param[in] product The product, the machine is working on.
 * @return The external atom.
 */
Clingo::Symbol
generateMachineWorkingExternal(const std::string& machineName, const int duration, const ProductIdentifier& product)
{
	return Clingo::Function("processing", {Clingo::String(machineName), productName(product), Clingo::Number(duration),
		Clingo::Number(0)});
}

/**
 * @brief Generates the external for a broken machine.
 * @param[in] machineName The name of the machine.
 * @param[in] duration How long the machine is broken.
 * @return The external atom.
 */
Clingo::Symbol
generateMachineStoringExternal(const std::string& machineName, const ProductIdentifier& product)
{
	return Clingo::Function("storing", {Clingo::String(machineName), productName(product), Clingo::Number(0)});
}

