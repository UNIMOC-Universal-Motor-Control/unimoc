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

#ifndef UNIMOC_SYSTEM_SIN_COS_H_
#define UNIMOC_SYSTEM_SIN_COS_H_

#include <array>
#include <cmath>
#include <concepts>	
#include "Units.hpp"
#include "arm_math.h"  // For arm_sin_cos

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
        /// @brief SinCos class
        ///
        /// This class is used to represent the sine and cosine values of an angle.
        ///
        /// It provides methods to compute the sine and cosine values, normalize them, and perform arithmetic operations.   
        ///        
        class SinCos
        {
		private:
            // private method to compute sine and cosine from angle
			constexpr void
			from_angle(const unit::Angle angle) noexcept;

            // constructor with sine and cosine values
            constexpr SinCos(float sin_value, float cos_value) noexcept
                : sin(sin_value), cos(cos_value) {}

		public:

            // sine and cosine values
            float sin;  
            float cos;

            // default constructor destructor
            constexpr SinCos() = default;
            constexpr ~SinCos() = default;

			// constructor with angle
			constexpr SinCos(const unit::Angle angle) noexcept;

			// length of the vector
			constexpr float
			length() const noexcept;
			
			// normalize the sin and cos values to one
			constexpr SinCos
			normToOne(void) noexcept;

            // copy constructor
			constexpr SinCos(const SinCos &other) noexcept;
			// move constructor
			constexpr SinCos(SinCos &&other) noexcept;
			// copy assignment operator
			constexpr SinCos &
			operator=(const SinCos &other) noexcept;

			// move assignment operator
			constexpr SinCos &
			operator=(SinCos &&other) noexcept;

			// equality operator
            inline constexpr bool operator==(const SinCos &other) const noexcept
            {
                return (sin == other.sin && cos == other.cos);
            }

            // inequality operator
            inline constexpr bool operator!=(const SinCos &other) const noexcept
            {
                return !(*this == other);
            }

            // transform to array
			inline constexpr std::array<float, 2>
			to_array() const noexcept
			{
                return {sin, cos};
            }

			inline constexpr SinCos
			operator-() const noexcept
			{
				return SinCos(-sin, -cos);
			}

			// sine cosine difference
			constexpr SinCos
			operator-(const SinCos &other) const noexcept;
		};
    } // namespace system
} // namespace unimoc

#endif /* UNIMOC_SYSTEM_SIN_COS_H_ */