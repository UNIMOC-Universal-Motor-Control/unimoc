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

#ifndef UNIMOC_HARDWARE_ANALOG_H_
#define UNIMOC_HARDWARE_ANALOG_H_

#include <cstdint>
#include <modm/platform.hpp>

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
 * @namespace analog analog namespace
 * @brief Contains definitions and mappings for analog signals in the UNIMOC hardware.
 */
namespace analog
{
//
// ADC Channel Mapping
// Pin		Channel			Signal
// ---------------------------------------------
// PA0		ADC1_IN1		Current IA
// PA2		ADC1_IN3		Voltage VA
//
using A1_IA = modm::platform::GpioA0::In1;
using A1_VA = modm::platform::GpioA2::In3;
// ---------------------------------------------
// PA1		ADC2_IN2		Current IB
// PA6		ADC2_IN3		Voltage VB
using A2_IB = modm::platform::GpioA1::In2;
using A2_VB = modm::platform::GpioA6::In3;
// ---------------------------------------------
// PB1		ADC3_IN1		Current IC
// PB13		ADC3_IN5		Voltage VC
using A3_IC = modm::platform::GpioB1::In1;
using A3_VC = modm::platform::GpioB13::In5;
// ---------------------------------------------
// PB12		ADC4_IN3		DC Link Voltage
// PB14		ADC4_IN4		Crank Torque
using A4_VDC = modm::platform::GpioB12::In3;
using A4_CRKT = modm::platform::GpioB14::In4;
// ---------------------------------------------
// PA8		ADC5_IN1		Motor Temperature
// PA9		ADC5_IN2		Bridge Temperature
using A5_MOTT = modm::platform::GpioA8::In1;
using A5_BRDGT = modm::platform::GpioA9::In2;


/**
 * @brief Initializes the analog subsystem.
 * This function sets up the necessary configurations for the analog inputs.
 */
void
initialize(void) noexcept;

/**
 * @brief Reads the current from the specified phase.
 * @param phase The phase to read the current from (0 for A, 1 for B, 2 for C).
 * @return The current value in Amperes.
 */
system::ThreePhase getPhaseCurrents(void) noexcept;

/**
 * @brief Reads the voltage from the specified phase.
 * @param phase The phase to read the voltage from (0 for A, 1 for B, 2 for C).
 * @return The voltage value in Volts.
 */
system::ThreePhase getPhaseVoltages(void) noexcept;

/**
 * @brief Reads the DC link voltage.
 * @return The DC link voltage in Volts.
 */
float getDcLinkVoltage(void) noexcept;

/**
 * @brief Reads the crank torque.
 * @return The crank torque in Newton-meters.
 */
float getCrankTorque(void) noexcept;

/**
 * @brief Reads the motor temperature.
 * @return The motor temperature in degrees Celsius.
 */
float getMotorTemperature(void) noexcept;

/**
 * @brief Reads the bridge temperature.
 * @return The bridge temperature in degrees Celsius.
 */
float getBridgeTemperature(void) noexcept;

/**
 * @brief Sets the phase duties for the motor control.
 * @param duties The phase duties to set, represented as a ThreePhase structure.
 */
void setPhaseDuties(const system::ThreePhase& duties) noexcept;

}  // namespace analog
}  // namespace hardware
}  // namespace unimoc

#endif /* UNIMOC_HARDWARE_ANALOG_H_ */