/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *     / /_/ / /|  // //  / / /_/ / /___
 *     \____/_/ |_/___/_/  /_/\____/\____/
 *
 *     @file stator_system.hpp
 *     @brief Stationary alpha/beta reference frame and transformations.
 *
 *     This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *     See the repository LICENSE file for details.
 */
#pragma once

#include <array>
#include <cmath>
#include <concepts>

/**
 * @namespace unimoc Global UNIMOC namespace.
 */
/**
 * @namespace unimoc::system Coordinate-system data types.
 */
namespace unimoc::system
{

/** Forward declaration of the rotating reference frame. */
template <typename T>
struct RotorReference;
/** Forward declaration of precomputed sine and cosine values. */
template <std::floating_point T>
struct SinCos;
/** Forward declaration of a rotor angle. */
class RotorAngle;

/**
 * @brief Stores a vector in the stationary alpha/beta reference frame.
 *
 * The alpha axis is aligned with the stator reference axis and the beta axis
 * is orthogonal to it. Values use the representation type supplied as `T`.
 *
 * @tparam T Representation type used for alpha and beta.
 */
template <typename T = float>
struct Stator
{
       /// Alpha-axis component.
       T alpha;
       /// Beta-axis component.
       T beta;

       /** @brief Constructs a zero-valued stator vector. */
       constexpr Stator() = default;

       /**
	* @brief Constructs a stator vector from alpha and beta values.
	* @param alpha_in Alpha-axis component.
	* @param beta_in Beta-axis component.
	*/
       constexpr Stator(T alpha_in, T beta_in) : alpha(alpha_in), beta(beta_in) {}

       /** @brief Copies a stator vector. */
       constexpr Stator(const Stator &other) : alpha(other.alpha), beta(other.beta) {}
       /** @brief Moves a stator vector. */
       constexpr Stator(Stator &&other) noexcept : alpha(other.alpha), beta(other.beta) {}

       /**
	* @brief Copies the alpha and beta values from another vector.
	* @param other Vector to copy.
	* @return This vector after assignment.
	*/
       constexpr Stator &
       operator=(const Stator &other)
       {
	       if (this != &other)
	       {
		       alpha = other.alpha;
		       beta = other.beta;
	       }
	       return *this;
       }

       /**
	* @brief Moves the alpha and beta values from another vector.
	* @param other Vector to move.
	* @return This vector after assignment.
	*/
       constexpr Stator &
       operator=(Stator &&other) noexcept
       {
	       if (this != &other)
	       {
		       alpha = other.alpha;
		       beta = other.beta;
	       }
	       return *this;
       }

       /**
	* @brief Compares two stator vectors for equality.
	* @param other Vector to compare.
	* @return `true` when both components are equal.
	*/
       constexpr bool
       operator==(const Stator &other) const
       {
	       return (alpha == other.alpha && beta == other.beta);
       }

       /**
	* @brief Compares two stator vectors for inequality.
	* @param other Vector to compare.
	* @return `true` when at least one component differs.
	*/
       constexpr bool
       operator!=(const Stator &other) const
       {
	       return !(*this == other);
       }

       /**
	* @brief Adds two stator vectors component-wise.
	* @param other Vector to add.
	* @return The component-wise sum.
	*/
       constexpr Stator
       operator+(const Stator &other) const
       {
	       return Stator(alpha + other.alpha, beta + other.beta);
       }

       /**
	* @brief Subtracts two stator vectors component-wise.
	* @param other Vector to subtract.
	* @return The component-wise difference.
	*/
       constexpr Stator
       operator-(const Stator &other) const
       {
	       return Stator(alpha - other.alpha, beta - other.beta);
       }

       /**
	* @brief Multiplies a stator vector by a rotor vector.
	* @param other Rotor vector used for the component-wise complex product.
	* @return The resulting rotor vector.
	*/
       constexpr RotorReference<T>
       operator*(const RotorReference<T> &other) const
       {
	       return RotorReference<T>(alpha * other.d - beta * other.q,
								alpha * other.q + beta * other.d);
       }

       /**
	* @brief Divides two stator vectors component-wise.
	* @param other Vector used as the divisor.
	* @return The component-wise quotient.
	*/
       constexpr Stator
       operator/(const Stator &other) const
       {
	       return Stator(alpha / other.alpha, beta / other.beta);
       }

       /**
	* @brief Returns the alpha and beta values in that order.
	* @return An array containing alpha and beta.
	*/
       constexpr auto
       ToArray() const noexcept -> std::array<T, 2>
       {
	       return {alpha, beta};
       }

       /**
	* @brief Returns the Euclidean length of the stator vector.
	* @return The vector length.
	*/
       constexpr T
       Length() const noexcept
       {
	       return std::sqrt(alpha * alpha + beta * beta);
       }

       /**
	* @brief Applies the Park transform to a stator vector.
	* @param angle Precomputed sine and cosine of the rotor angle.
	* @return The vector in the rotating d/q reference frame.
	*/
       constexpr RotorReference<T>
       ToRotor(const SinCos<T> &angle) const noexcept
       {
	       return RotorReference<T>(
		       alpha * angle.cos + beta * angle.sin,
		       -alpha * angle.sin + beta * angle.cos);
       }

       /**
	* @brief Applies the Park transform using a rotor angle.
	* @param angle Rotor angle used for the transform.
	* @return The vector in the rotating d/q reference frame.
	*/
	RotorReference<T>
       ToRotor(const RotorAngle &angle) const noexcept;
};

}  // namespace unimoc::system
