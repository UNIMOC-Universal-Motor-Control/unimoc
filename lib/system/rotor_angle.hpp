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

#ifndef UNIMOC_SYSTEM_ROTOR_ANGLE_H_
#define UNIMOC_SYSTEM_ROTOR_ANGLE_H_

#include <array>
#include <cmath>
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
        /// @brief RotorAngle class
        ///
        /// This class is used to represent the sine and cosine values of an angle.
        ///
        /// It provides methods to compute the sine and cosine values, normalize them, and perform arithmetic operations.   
        ///        
        class RotorAngle
        {
		private:
            // private method to compute sine and cosine from angle
			constexpr void
			update_sin_cos(const float angle) noexcept;

		public:
			float angle; // angle in radians
			float sin;   // sine of the angle
            float cos;   // cosine of the angle

            // default constructor destructor
			constexpr RotorAngle() noexcept : angle(0.0f), sin(0.0f), cos(1.0f) {}
			constexpr ~RotorAngle() = default;

			// constructor with angle
			constexpr RotorAngle(const float rotorAngle) noexcept;
			// constructor with angle and sine and cosine
			constexpr RotorAngle(const float rotorAngle, const float angleSine, const float angleCosine) noexcept
				: angle(rotorAngle), sin(angleSine), cos(angleCosine)
			{}

			// copy constructor
			constexpr RotorAngle(const RotorAngle &other) noexcept
                : angle(other.angle), sin(other.sin), cos(other.cos) {}
			// move constructor
			constexpr RotorAngle(RotorAngle &&other) noexcept
                : angle(other.angle), sin(other.sin), cos(other.cos) {}
			// copy assignment operator
			constexpr RotorAngle &
			operator=(const RotorAngle &other) noexcept
            {
                if (this != &other)
                {
                    angle = other.angle;
                    sin = other.sin;
                    cos = other.cos;
                }
                return *this;
            }

			// move assignment operator
			constexpr RotorAngle &
			operator=(RotorAngle &&other) noexcept
            {
                if (this != &other)
                {
                    angle = other.angle;
                    sin = other.sin;
                    cos = other.cos;
                }
                return *this;
            }

			// equality operator
			inline constexpr bool
			operator==(const RotorAngle &other) const noexcept
			{
                return (sin == other.sin && cos == other.cos && angle == other.angle);
            }

            // inequality operator
			inline constexpr bool
			operator!=(const RotorAngle &other) const noexcept
			{
                return !(*this == other);
            }

            // transform to array
			inline constexpr std::array<float, 2>
			to_array() const noexcept
			{
                return {sin, cos};
            }

			// sine cosine difference
			constexpr RotorAngle
			operator-(const RotorAngle &other) const noexcept;

            // addition operator
			constexpr RotorAngle
			operator+(const RotorAngle &other) const noexcept;
		};
	} // namespace system
} // namespace unimoc

#endif /* UNIMOC_SYSTEM_ROTOR_ANGLE_H_ */