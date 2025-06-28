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

#ifndef UNIMOC_SYSTEM_STATOR_REFERENCE_H_
#define UNIMOC_SYSTEM_STATOR_REFERENCE_H_

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

// Forward declaration of RotorReference to avoid circular dependency
struct RotorReference;
// Forward declaration of RotorAngle to avoid circular dependency
class RotorAngle;

///< @brief StatorReference class
/// This class represents a reference frame in a rotating alpha beta system.
/// It contains two components: alpha and beta, which are orthogonal components
/// of the stator voltage or current in the stationary reference frame.
/// The alpha axis is aligned with the stator winding, while the beta axis is
/// perpendicular to it.
struct StatorReference
{
	float alpha;
	float beta;

	constexpr StatorReference() = default;
	constexpr StatorReference(float _alpha, float _beta) : alpha(_alpha), beta(_beta) {}

	// copy constructor
	constexpr StatorReference(const StatorReference &other) : alpha(other.alpha), beta(other.beta)
	{}
	// move constructor
	constexpr StatorReference(StatorReference &&other) noexcept
		: alpha(other.alpha), beta(other.beta)
	{}
	// copy assignment operator
	constexpr StatorReference &
	operator=(const StatorReference &other)
	{
		if (this != &other)
		{
			alpha = other.alpha;
			beta = other.beta;
		}
		return *this;
	}

	// move assignment operator
	constexpr StatorReference &
	operator=(StatorReference &&other) noexcept
	{
		if (this != &other)
		{
			alpha = other.alpha;
			beta = other.beta;
		}
		return *this;
	}

	// equality operator
	constexpr bool
	operator==(const StatorReference &other) const
	{
		return (alpha == other.alpha && beta == other.beta);
	}

	// inequality operator
	constexpr bool
	operator!=(const StatorReference &other) const
	{
		return !(*this == other);
	}

	// addition operator
	constexpr StatorReference
	operator+(const StatorReference &other) const
	{
		return StatorReference(alpha + other.alpha, beta + other.beta);
	}

	// subtraction operator
	constexpr StatorReference
	operator-(const StatorReference &other) const
	{
		return StatorReference(alpha - other.alpha, beta - other.beta);
	}

	// transform to array
	constexpr auto
	to_array() const noexcept -> std::array<float, 2>
	{
		return {alpha, beta};
	}

	// length of the vector
	constexpr float
	length() const noexcept
	{
		return std::sqrt(alpha * alpha + beta * beta);
	}

	// transform alpha beta vector to dq vector.
	constexpr RotorReference
	park(const RotorAngle &angle) const noexcept;
};
}  // namespace system
}  // namespace unimoc

#endif /* UNIMOC_SYSTEM_STATOR_REFERENCE_H_ */