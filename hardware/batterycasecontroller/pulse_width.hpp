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

#ifndef UNIMOC_HARDWARE_PULSE_WIDTH_H_
#define UNIMOC_HARDWARE_PULSE_WIDTH_H_

#include <cstdint>
#include <modm/platform.hpp>

#include "three_phase_system.hpp"
#include "base.hpp"

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
 * @namespace pulse_width pulse_width namespace
 * @brief Contains functions and definitions for handling pulse width modulation and analog inputs.
 */
namespace pulse_width
{

/**
 * @brief Initializes the pwm subsystem.
 * This function sets up the necessary configurations for the pwm outputs.
 */
void
initialize(void) noexcept;

/**
 * @brief Sets the phase duties for the motor control.
 * @param duties The phase duties to set, represented as a ThreePhase structure.
 */
void
setPhaseDuties(const system::ThreePhase& duties) noexcept;

}  // namespace pulse_width
}  // namespace hardware
}  // namespace unimoc

#endif /* UNIMOC_HARDWARE_PULSE_WIDTH_H_ */