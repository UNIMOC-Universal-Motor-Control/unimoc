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
        ///< sinus and cosine values of an angle.
        template <typename T = float>
        struct SinCos
        {
            T sin;
            T cos;

            // function to compute sin and cos values
            // this function should be implemented by the user
            // it should return the sin and cos values of the given angle
            virtual SinCos<T> computeSinCos(T angle) = 0;

            // default constructor destructor
            constexpr SinCos() = default;
            virtual ~SinCos() = default;

            // constructor with angle
            constexpr SinCos(T angle)
            {
                // compute sin and cos values
                SinCos<T> result = computeSinCos(angle);
                sin = result.sin;
                cos = result.cos;
            }

            // length of the vector
            constexpr T length() const noexcept
            {
                return std::sqrt(sin * sin + cos * cos);
            }

            // normalize the sin and cos values to one
            constexpr NormToOne<T> normToOne() const
            {
                return SinCos<T>(sin / length(), cos / length());
            }

            // constructor with sin and cos values
            constexpr SinCos(T sin, T cos) : sin(sin), cos(cos) {  }

            // copy constructor
            constexpr SinCos(const SinCos &other) : sin(other.sin), cos(other.cos) { }
            // move constructor
            constexpr SinCos(SinCos &&other) noexcept : sin(other.sin), cos(other.cos) {  }
            // copy assignment operator
            constexpr SinCos &operator=(const SinCos &other)
            {
                if (this != &other)
                {
                    sin = other.sin;
                    cos = other.cos;
                }
                return *this;
            }

            // move assignment operator
            constexpr SinCos &operator=(SinCos &&other) noexcept
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

            constexpr auto operator-() const noexcept -> SinCos
            {
                return SinCos(-sin, -cos);
            }

            // sine cosine difference
            constexpr SinCos operator-(const SinCos &other) const
            {
                // Using trigonometric identities for subtraction:
                // sin(a - b) = sin(a)cos(b) - cos(a)sin(b)
                // cos(a - b) = cos(a)cos(b) + sin(a)sin(b)
                return SinCos(sin * other.cos - cos * other.sin, cos * other.cos + sin * other.sin);
            }

            // sin cosine difference
            constexpr T operator-(const SinCos &other) const
            {
                // Using trigonometric identities for subtraction:
                // sin(a - b) = sin(a)cos(b) - cos(a)sin(b)
                return sin * other.cos - cos * other.sin;
            }
        };
    } // namespace system
} // namespace unimoc

#endif /* UNIMOC_SYSTEM_SIN_COS_H_ */