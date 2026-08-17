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
#include <concepts>

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
template <typename T>
struct RotorReference;
template <std::floating_point T>
struct SinCos;
// Forward declaration of RotorAngle to avoid circular dependency
class RotorAngle;

///< @brief StatorReference class
/// This class represents a reference frame in a rotating alpha beta system.
/// It contains two components: alpha and beta, which are orthogonal components
/// of the stator voltage or current in the stationary reference frame.
/// The alpha axis is aligned with the stator winding, while the beta axis is
/// perpendicular to it.
template <typename T = float>
struct StatorReference
{
	T alpha;
	T beta;

	constexpr StatorReference() = default;
	constexpr StatorReference(T alpha_in, T beta_in) : alpha(alpha_in), beta(beta_in) {}

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

	// multiplication operator
	constexpr RotorReference<T>
	operator*(const RotorReference<T> &other) const
	{
		return RotorReference<T>(alpha * other.d - beta * other.q,
									alpha * other.q + beta * other.d);
	}

	// division operator
	constexpr StatorReference
	operator/(const StatorReference &other) const
	{
		return StatorReference(alpha / other.alpha, beta / other.beta);
	}

	// transform to array
	constexpr auto
	ToArray() const noexcept -> std::array<T, 2>
	{
		return {alpha, beta};
	}

	// length of the vector
	constexpr T
	length() const noexcept
	{
		return std::sqrt(alpha * alpha + beta * beta);
	}

	// transform alpha beta vector to dq vector.
	constexpr RotorReference<T>
	park(const SinCos<T> &angle) const noexcept
	{
		return RotorReference<T>(
			alpha * angle.cos + beta * angle.sin,
			-alpha * angle.sin + beta * angle.cos);
	}

	// transform alpha beta vector to dq vector.
	constexpr RotorReference<T>
	park(const RotorAngle &angle) const noexcept;
};
}  // namespace system
}  // namespace unimoc
