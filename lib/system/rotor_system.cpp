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

#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>

#include "rotor_angle.hpp"    // Include the header file for RotorAngle
#include "rotor_system.hpp"  // Include the header file for RotorReference
#include "stator_system.hpp"  // Include the header file for StatorReference

/**
 * @namespace unimoc global namespace
 */
namespace unimoc
{
/**
 * @namespace coordinate systems.
 */
namespace system
{
// transform dq vector to alpha beta vector.
constexpr StatorReference
RotorReference::inverse_park(const RotorAngle &angle) const noexcept
{
	return StatorReference(d * angle.cos - q * angle.sin, d * angle.sin + q * angle.cos);
}
}  // namespace system
}  // namespace unimoc
