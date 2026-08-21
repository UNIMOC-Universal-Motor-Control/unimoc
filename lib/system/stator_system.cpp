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
#include "stator_system.hpp"
#include "rotor_system.hpp"
#include "rotor_angle.hpp"

/**
 * @namespace unimoc Global UNIMOC namespace.
 */
namespace unimoc
{
/**
 * @namespace unimoc::system Coordinate-system data types.
 */
namespace system
{

/** Applies the Park transform using the precomputed values in a RotorAngle. */
template <typename T>
RotorReference<T>
Stator<T>::ToRotor(const RotorAngle &angle) const noexcept
{
	return RotorReference<T>{alpha * angle.cos + beta * angle.sin,
							-alpha * angle.sin + beta * angle.cos};
}

template RotorReference<float>
Stator<float>::ToRotor(const RotorAngle &angle) const noexcept;
}  // namespace system
}  // namespace unimoc
