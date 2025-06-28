/*
	   __  ___   ________  _______  ______
	  / / / / | / /  _/  |/  / __ \/ ____/
	 / / / /  |/ // // /|_/ / / / / /
	/ /_/ / /|  // // /  / / /_/ / /___
	\____/_/ |_/___/_/  /_/\____/\____/

	Universal Motor Control  2025 Alexander <tecnologic86@gmail.com> Evers

	This file is part of UNIMOC.

	UNIMOC is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#ifndef UNIMOC_HARDWARE_INTERFACE_H_
#define UNIMOC_HARDWARE_INTERFACE_H_

#include <cstdint>
#include <array>
#include <functional>

#include "hardware_interface_config.hpp"
#include "three_phase_system.hpp"

/**
 * @namespace unimoc global namespace
 */
namespace unimoc
{
/**
 * @namespace hardware hardware namespace
 */
namespace hardware
{

/**
 * @brief Hardware interface class for the UNIMOC project.
 * This class provides a basic structure for hardware interfaces.
 */
struct HardwareInterface
{
	/**
	 * @brief Function to initialize the hardware interface.
	 * @return true if initialization is successful, false otherwise.
	 */
	const std::function<bool(void)>& initialize;

	/**
	 * @brief Functions to get the phase currents and voltages, and to set the phase duties.
	 */
	const std::function<system::ThreePhase(void)>& getPhaseCurrents;

	/**
	 * @brief Function to get the phase voltages.
	 */
	const std::function<system::ThreePhase(void)>& getPhaseVoltages;

	/**
	 * @brief Function to set the phase duties.
	 * @param duties An array of floats representing the phase duties.
	 */
	const std::function<void(system::ThreePhase)>& setPhaseDutys;

	/**
	 * @brief Constructor for the HardwareInterface class.
	 * @param getPhaseCurrents Function to get the phase currents.
	 * @param getPhaseVoltages Function to get the phase voltages.
	 * @param setPhaseDutys Function to set the phase duties.
	 */
	HardwareInterface(const std::function<bool(void)>& _initialize,
					  const std::function<system::ThreePhase(void)>& _getPhaseCurrents,
					  const std::function<system::ThreePhase(void)>& _getPhaseVoltages,
					  const std::function<void(system::ThreePhase)>& _setPhaseDutys)
		: initialize(_initialize),
		  getPhaseCurrents(_getPhaseCurrents),
		  getPhaseVoltages(_getPhaseVoltages),
		  setPhaseDutys(_setPhaseDutys) {};

	/**
	 * @brief Default destructor.
	 */
	~HardwareInterface() = default;
};

extern HardwareInterface motor[MOTORS];  ///< Global array of hardware interfaces for motors

}  // namespace hardware
}  // namespace unimoc

#endif /* UNIMOC_HARDWARE_INTERFACE_H_ */