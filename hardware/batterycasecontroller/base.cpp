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

#include <modm/platform.hpp>
#include <modm/debug.hpp>
#include "HardwareInterface.hpp"

using namespace modm::platform;
using namespace std::chrono_literals;

namespace unimoc::hardware
{

HardwareInterface motor[MOTORS] = {
    {
        // Initialize function
        []() -> bool {
            // Initialization code here
            return true;  // Return true if initialization is successful
        },

        // Get phase currents function
        []() -> std::array<float, PHASES> {
            return {0.0f, 0.0f, 0.0f};  // Replace with actual current readings
        },

        // Get phase voltages function
        []() -> std::array<float, PHASES> {
            return {0.0f, 0.0f, 0.0f};  // Replace with actual voltage readings
        },

        // Set phase duties function
        [](std::array<float, PHASES> duties) {
            // Set the phase duties here
            for (const auto& duty : duties) {
                MODM_LOG_INFO << "Setting duty: " << duty << '\n';
            }
        }
    }
};

}  // namespace unimoc::hardware

