/*
 *	   __  ___   ________  _______  ______
 *	  / / / / | / /  _/  |/  / __ \/ ____/
 *	 / / / /  |/ // // /|_/ / / / / /
 *	/ /_/ / /|  // // /  / / /_/ / /___
 *	\____/_/ |_/___/_/  /_/\____/\____/
 *
 *	@file three_phase_system.hpp
 *	@brief Three-phase system representation and transformations.
 *
 *	This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *	See the repository LICENSE file for details.
 */
#pragma once

#include <array>

#include "stator_system.hpp"
#include "units.hpp"

/**
 * @namespace unimoc Global UNIMOC namespace.
 */
/**
 * @namespace unimoc::system Coordinate-system data types.
 */
namespace unimoc::system
{

/**
 * @brief Stores one unit-typed value for each motor phase.
 *
 * All three components use the same unit type. This prevents a phase vector
 * from being instantiated with an untyped scalar or with incompatible unit
 * quantities.
 *
 * @tparam T UNIMOC unit type used for phases A, B, and C.
 */
template <unimoc::unit::UnitLike T = unimoc::unit::DimensionlessRatio>
struct ThreePhase
{
	/// Unit representation type used by each phase.
	using Representation = typename T::representation;

	/// Phase A value.
	T a;
	/// Phase B value.
	T b;
	/// Phase C value.
	T c;

	/** @brief Constructs a zero-valued three-phase vector. */
	constexpr ThreePhase() = default;

	/**
	 * @brief Constructs a three-phase vector from unit values.
	 * @param a_in Phase A value.
	 * @param b_in Phase B value.
	 * @param c_in Phase C value.
	 */
	constexpr ThreePhase(T a_in, T b_in, T c_in) : a(a_in), b(b_in), c(c_in) {}

	/**
	 * @brief Constructs a three-phase vector from base representation values.
	 * @param a_in Phase A representation value.
	 * @param b_in Phase B representation value.
	 * @param c_in Phase C representation value.
	 */
	constexpr ThreePhase(Representation a_in, Representation b_in, Representation c_in)
		: a(T{a_in}), b(T{b_in}), c(T{c_in})
	{}

	/** @brief Copies a three-phase vector. */
	constexpr ThreePhase(const ThreePhase &other) : a(other.a), b(other.b), c(other.c) {}
	/** @brief Moves a three-phase vector. */
	constexpr ThreePhase(ThreePhase &&other) noexcept : a(other.a), b(other.b), c(other.c) {}
	/**
	 * @brief Copies the phase values from another vector.
	 * @param other Vector to copy.
	 * @return This vector after assignment.
	 */
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

	/**
	 * @brief Moves the phase values from another vector.
	 * @param other Vector to move.
	 * @return This vector after assignment.
	 */
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
	/**
	 * @brief Compares two three-phase vectors for equality.
	 * @param other Vector to compare.
	 * @return `true` when all three phase values are equal.
	 */
	constexpr bool
	operator==(const ThreePhase &other) const
	{
		return (a == other.a && b == other.b && c == other.c);
	}
	/**
	 * @brief Compares two three-phase vectors for inequality.
	 * @param other Vector to compare.
	 * @return `true` when at least one phase value differs.
	 */
	constexpr bool
	operator!=(const ThreePhase &other) const
	{
		return !(*this == other);
	}
	/**
	 * @brief Adds two three-phase vectors component-wise.
	 * @param other Vector to add.
	 * @return The component-wise sum.
	 */
	constexpr ThreePhase
	operator+(const ThreePhase &other) const
	{
		return ThreePhase(a + other.a, b + other.b, c + other.c);
	}
	/**
	 * @brief Subtracts two three-phase vectors component-wise.
	 * @param other Vector to subtract.
	 * @return The component-wise difference.
	 */
	constexpr ThreePhase
	operator-(const ThreePhase &other) const
	{
		return ThreePhase(a - other.a, b - other.b, c - other.c);
	}

	/**
	 * @brief Returns the phase values in A, B, C order.
	 * @return An array containing the three unit values.
	 */
	constexpr auto
	ToArray() const noexcept -> std::array<T, 3>
	{
		return {a, b, c};
	}

	/**
	 * @brief Applies the Clarke transform to the three-phase vector.
	* @return The alpha/beta stator vector using the same unit type.
	 */
       constexpr Stator<T>
	ToStator() const noexcept
	{
		constexpr Representation sqrt3by2 = static_cast<Representation>(0.86602540378443864676);
		constexpr Representation two_by_three = static_cast<Representation>(2.0 / 3.0);

			   return Stator<T>(
			two_by_three * (a.Value() - (static_cast<Representation>(0.5) * b.Value()) -
									(static_cast<Representation>(0.5) * c.Value())),
			two_by_three * ((sqrt3by2 * b.Value()) - (sqrt3by2 * c.Value())));
	}
};

}  // namespace unimoc::system
