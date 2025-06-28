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

#ifndef UNIMOC_SYSTEM_THREE_PHASE_H_
#define UNIMOC_SYSTEM_THREE_PHASE_H_

#include <array>

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

// Forward declaration of StatorReference to avoid circular dependency
struct StatorReference;

///< @brief ThreePhase class
/// This class represents a three-phase system with three components: a, b, and c.
/// It provides methods to perform operations such as addition, subtraction, and transformation
/// to a two-phase alpha-beta system using the Clarke transformation.
struct ThreePhase
{
	float a;
	float b;
	float c;
	constexpr ThreePhase() = default;
	constexpr ThreePhase(float _a, float _b, float _c) : a(_a), b(_b), c(_c) {}

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
	to_array() const noexcept -> std::array<float, 3>
	{
		return {a, b, c};
	}

	// clarke transformation
	// transform abc 3 phase vector to alpha beta vector.
	constexpr StatorReference
	clark() const noexcept;
};

}  // namespace system
}  // namespace unimoc

#endif /* UNIMOC_SYSTEM_THREE_PHASE_H_ */