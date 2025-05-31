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
#include "units.h"

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
        /// @tparam T The type of the sine and cosine values. It must be a floating point type.
        ///
        /// @note The class is designed
        /// to be used with angles in radians, and it is expected that the user will implement the `computeSinCos` method
        /// to provide the actual sine and cosine calculations based on the angle.    
        template <std::floating_point T>
        struct SinCos
        {
            T sin;
            T cos;

            // function to compute sin and cos values
            // this function should be implemented by the user
            // it should return the sin and cos values of the given angle
            virtual SinCos<T> computeSinCos(const units::unit_t<units::angle::radian, T> angle) = 0;

            // default constructor destructor
            constexpr SinCos() = default;
            virtual ~SinCos() = default;

            // constructor with angle
            constexpr SinCos(const units::unit_t<units::angle::radian, T> angle)
            {
                // compute sin and cos values
                SinCos<T> result = computeSinCos(angle);
                sin = result.sin;
                cos = result.cos;
            }

            // length of the vector
            constexpr T length() const noexcept
            {
                return std::hypot(sin, cos);
            }

            // normalize the sin and cos values to one
            constexpr auto normToOne(void) const
            {
                return SinCos<T>(sin / this.length(), cos / this.length());
            }

            // constructor with sin and cos values
            constexpr SinCos(const T sin, const T cos) : sin(sin), cos(cos) {  }

            // copy constructor
            constexpr SinCos(const SinCos &other) : sin(other.sin), cos(other.cos) { }
            // move constructor
            constexpr SinCos(SinCos &&other) noexcept : sin(other.sin), cos(other.cos) {  }
            // copy assignment operator
            constexpr auto &operator=(const SinCos &other)
            {
                if (this != &other)
                {
                    sin = other.sin;
                    cos = other.cos;
                }
                return *this;
            }

            // move assignment operator
            constexpr auto &operator=(SinCos &&other) noexcept
            {
                if (this != &other)
                {
                    sin = other.sin;
                    cos = other.cos;
                }
                return *this;
            }

            // equality operator
            constexpr bool operator==(const SinCos &other) const
            {
                return (sin == other.sin && cos == other.cos);
            }

            // inequality operator
            constexpr bool operator!=(const SinCos &other) const
            {
                return !(*this == other);
            }

            // transform to array
            constexpr auto to_array() const noexcept -> std::array<T, 2>
            {
                return {sin, cos};
            }

            constexpr auto operator-() const noexcept -> SinCos<T>
            {
                return SinCos<T>(-sin, -cos);
            }

            // sine cosine difference
            constexpr auto operator-(const SinCos<T> &other) const
            {
                // Using trigonometric identities for subtraction:
                // sin(a - b) = sin(a)cos(b) - cos(a)sin(b)
                // cos(a - b) = cos(a)cos(b) + sin(a)sin(b)
                return SinCos<T>(sin * other.cos - cos * other.sin, cos * other.cos + sin * other.sin);
            }
        };
    } // namespace system
} // namespace unimoc

#endif /* UNIMOC_SYSTEM_SIN_COS_H_ */