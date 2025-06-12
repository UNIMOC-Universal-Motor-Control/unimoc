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

#include "units.h"

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
 * @namespace analog analog namespace
 */
namespace analog
{

    /**
     * @brief Initializes the ADC channels for current and voltage measurements.
     *
     * This function sets up the GPIOs for the ADC channels, initializes the ADCs,
     * and configures the injected conversion sequences for current and voltage
     * measurements.
     */
    void initialize();

}  // namespace analog
}  // namespace hardware
}  // namespace unimoc

#endif /* UNIMOC_HARDWARE_INTERFACE_H_ */