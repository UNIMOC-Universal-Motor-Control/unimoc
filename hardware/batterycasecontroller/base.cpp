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

#include <modm/debug.hpp>
#include <modm/platform.hpp>

#include "hardware_interface.hpp"
#include "analog.hpp"
#include "pulse_width.hpp"

using namespace modm::platform;
using namespace std::chrono_literals;
using namespace modm::literals;

namespace unimoc::hardware
{

HardwareInterface motor[MOTORS] = {
	{// Initialize function
	 []() -> bool {
		 // Initialization code here
		 return true;  // Return true if initialization is successful
	 },

	 // Get phase currents function
	 []() -> system::ThreePhase {
		 return {0.0f, 0.0f, 0.0f};  // Replace with actual current readings
	 },

	 // Get phase voltages function
	 []() -> system::ThreePhase {
		 return {0.0f, 0.0f, 0.0f};  // Replace with actual voltage readings
	 },

	 // Set phase duties function
	 [](system::ThreePhase duties) {
		 // Set the phase duties here
		 for (const auto& duty : duties.to_array())
		 {
			 MODM_LOG_INFO << "Setting duty: " << duty << '\n';
		 }
	 }}};

/**
 * Initializes the hardware components of the battery case controller.
 *
 * This function sets up the system clock, initializes the ADCs, and configures
 * the pulse width modulation (PWM) for motor control.
 */
void
initialize()
{
	SystemClock::enable();
	SysTickTimer::initialize<SystemClock>();

	// Initialize the ADCs
	analog::initialize();

	// Initialize the pulse width modulation (PWM) for motor control
	pulse_width::initialize();

}

}  // namespace unimoc::hardware


// Include the necessary headers for RTT logging
modm::platform::Rtt rtt(0);
modm::IODeviceObjectWrapper<modm::platform::Rtt, modm::IOBuffer::DiscardIfFull> rtt_device(rtt);
// Set all four logger streams to use RTT
modm::log::Logger modm::log::debug(rtt_device);
modm::log::Logger modm::log::info(rtt_device);
modm::log::Logger modm::log::warning(rtt_device);
modm::log::Logger modm::log::error(rtt_device);
