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
// Forward declaration of Stator to avoid circular dependency.
template <typename T>
struct Stator;
template <std::floating_point T>
struct SinCos;
// Forward declaration of RotorAngle to avoid circular dependency
class RotorAngle;

///< @brief RotorReference class
/// This class represents a reference frame in a rotating dq system.
/// It contains two components: d (direct axis) and q (quadrature axis).
/// The d-axis is aligned with the rotor's magnetic field, while the q-axis is perpendicular to
/// it.
template <typename T = float>
struct RotorReference
{
	T d;
	T q;

	// default constructor
	constexpr RotorReference() = default;
	// constructor with parameters
	constexpr RotorReference(T d_in, T q_in) : d(d_in), q(q_in) {}
	// copy constructor
	constexpr RotorReference(const RotorReference &other) : d(other.d), q(other.q) {}
	// move constructor
	constexpr RotorReference(RotorReference &&other) noexcept : d(other.d), q(other.q) {}
	// copy assignment operator
	constexpr RotorReference &
	operator=(const RotorReference &other)
	{
		if (this != &other)
		{
			d = other.d;
			q = other.q;
		}
		return *this;
	}

	// move assignment operator
	constexpr RotorReference &
	operator=(RotorReference &&other) noexcept
	{
		if (this != &other)
		{
			d = other.d;
			q = other.q;
		}
		return *this;
	}

	// equality operator
	constexpr bool
	operator==(const RotorReference &other) const
	{
		return (d == other.d && q == other.q);
	}

	// inequality operator
	constexpr bool
	operator!=(const RotorReference &other) const
	{
		return !(*this == other);
	}

	// addition operator
	constexpr RotorReference
	operator+(const RotorReference &other) const
	{
		return RotorReference(d + other.d, q + other.q);
	}

	// subtraction operator
	constexpr RotorReference
	operator-(const RotorReference &other) const
	{
		return RotorReference(d - other.d, q - other.q);
	}

	// multiplication operator
	constexpr RotorReference
	operator*(const RotorReference &other) const
	{
		return RotorReference(d * other.d - q * other.q, d * other.q + q * other.d);
	}

	// division operator
	constexpr RotorReference
	operator/(const RotorReference &other) const
	{
		return RotorReference(
			(d * other.d + q * other.q) / (other.d * other.d + other.q * other.q),
			(q * other.d - d * other.q) / (other.d * other.d + other.q * other.q));
	}

	// unary minus operator
	constexpr RotorReference
	operator-() const
	{
		return RotorReference(-d, -q);
	}

	// unary plus operator
	constexpr RotorReference
	operator+() const
	{
		return *this;
	}

	// scalar multiplication operator
	constexpr RotorReference
	operator*(const T &scalar) const
	{
		return RotorReference(d * scalar, q * scalar);
	}

	// scalar division operator
	constexpr RotorReference
	operator/(const T &scalar) const
	{
		return RotorReference(d / scalar, q / scalar);
	}

	// transform to array
	constexpr std::array<T, 2>
	ToArray() const noexcept
	{
		return {d, q};
	}

	// length of the vector
	constexpr T
	length() const noexcept
	{
		return std::sqrt(d * d + q * q);
	}

	// transform dq vector to alpha beta vector.
	       constexpr Stator<T>
	inverse_park(const SinCos<T> &angle) const noexcept
	{
		       return Stator<T>(
			d * angle.cos - q * angle.sin,
			d * angle.sin + q * angle.cos);
	}

	// transform dq vector to alpha beta vector.
	Stator<T>
	inverse_park(const RotorAngle &angle) const noexcept;
};
}  // namespace system
}  // namespace unimoc
