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

#include <array>
#include <cmath>

#include "stator_system.hpp"

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

template <typename T = float>
struct ThreePhase
{
	T a;
	T b;
	T c;
	constexpr ThreePhase() = default;
	constexpr ThreePhase(T a_in, T b_in, T c_in) : a(a_in), b(b_in), c(c_in) {}

	// copy constructor
	constexpr ThreePhase(const ThreePhase &other) : a(other.a), b(other.b), c(other.c) {}
	// move constructor
	constexpr ThreePhase(ThreePhase &&other) noexcept : a(other.a), b(other.b), c(other.c) {}
	// copy assignment operator
	constexpr ThreePhase &
	operator=(const ThreePhase &other)
	{
		if (this != &other)
		{
			a = other.a;
			b = other.b;
			c = other.c;
		}
		return *this;
	}

	// move assignment operator
	constexpr ThreePhase &
	operator=(ThreePhase &&other) noexcept
	{
		if (this != &other)
		{
			a = other.a;
			b = other.b;
			c = other.c;
		}
		return *this;
	}
	// equality operator
	constexpr bool
	operator==(const ThreePhase &other) const
	{
		return (a == other.a && b == other.b && c == other.c);
	}
	// inequality operator
	constexpr bool
	operator!=(const ThreePhase &other) const
	{
		return !(*this == other);
	}
	// addition operator
	constexpr ThreePhase
	operator+(const ThreePhase &other) const
	{
		return ThreePhase(a + other.a, b + other.b, c + other.c);
	}
	// subtraction operator
	constexpr ThreePhase
	operator-(const ThreePhase &other) const
	{
		return ThreePhase(a - other.a, b - other.b, c - other.c);
	}

	// transform to array
	constexpr auto
	to_array() const noexcept -> std::array<T, 3>
	{
		return {a, b, c};
	}

	// clarke transformation
	// transform abc 3 phase vector to alpha beta vector.
	constexpr StatorReference<T>
	clark() const noexcept
	{
		constexpr T sqrt3by2 = static_cast<T>(0.86602540378443864676);
		constexpr T two_by_three = static_cast<T>(2.0f / 3.0f);

		return StatorReference<T>(
			two_by_three * (a - (static_cast<T>(0.5f) * b) - (static_cast<T>(0.5f) * c)),
			two_by_three * ((sqrt3by2 * b) - (sqrt3by2 * c)));
	}
};

}  // namespace system
}  // namespace unimoc
