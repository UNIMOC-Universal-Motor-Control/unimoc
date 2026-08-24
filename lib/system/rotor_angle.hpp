/*
 *	   __  ___   ________  _______  ______
 *	  / / / / | / /  _/  |/  / __ \/ ____/
 *	 / / / /  |/ // // /|_/ / / / / /
 *	/ /_/ / /|  // // /  / / /_/ / /___
 *	\____/_/ |_/___/_/  /_/\____/\____/
 *
 *	@file rotor_angle.hpp
 *	@brief Fixed-point electrical rotor angle with revolution counter and sine/cosine lookup.
 *
 *	This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *	See the repository LICENSE file for details.
 */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <numbers>
#include "units.hpp"

/**
 * @namespace unimoc Global UNIMOC namespace.
 */
/**
 * @namespace unimoc::system Coordinate-system data types.
 */
namespace unimoc::system {

/**
 * @namespace unimoc::system::detail Implementation helpers for the rotor angle.
 */
namespace detail {

/// Number of entries in the sine lookup table, excluding the wrap-around entry.
inline constexpr std::size_t kSinTableSize = 512U;
/// Number of entries in a quarter revolution.
inline constexpr std::size_t kSinTableQuarterSize = kSinTableSize / 4U;
/// Number of low bits of the Q1.31 phase used as the interpolation fraction.
inline constexpr std::uint32_t kSinTableFractionBits = 23U;
/// Mask selecting the interpolation fraction bits of the Q1.31 phase.
inline constexpr std::uint32_t kSinTableFractionMask = (1U << kSinTableFractionBits) - 1U;
/// Scale converting the fraction bits into a normalised fraction in [0, 1).
inline constexpr float kSinTableFractionScale = 1.0F / static_cast<float>(1U << kSinTableFractionBits);

/// Number of Q1.31 counts per full revolution.
inline constexpr std::int64_t kCountsPerRevolution = 0x1'0000'0000LL;
/// Number of Q1.31 counts per half revolution, the positive wrap limit.
inline constexpr std::int64_t kCountsPerHalfRevolution = 0x8000'0000LL;
/// Number of Q1.31 counts per full revolution as a float.
inline constexpr float kCountsPerRevolutionF = static_cast<float>(kCountsPerRevolution);
/// Radians represented by a single Q1.31 count.
inline constexpr float kRadiansPerCount = static_cast<float>(2.0 * std::numbers::pi_v<double> / static_cast<double>(kCountsPerRevolution));
/// Revolutions represented by one radian.
inline constexpr float kRevolutionsPerRadian = static_cast<float>(1.0 / (2.0 * std::numbers::pi_v<double>));
/// Radians per full revolution.
inline constexpr float kRadiansPerRevolution = 2.0F * std::numbers::pi_v<float>;

/**
 * @brief Evaluates the sine Taylor series for compile-time table generation.
 * @param angleInRadians Argument in radians, accurate for |angleInRadians| <= pi/4.
 * @return The sine of @p angleInRadians.
 */
constexpr double TaylorSin(double angleInRadians) noexcept {
  const double x2 = angleInRadians * angleInRadians;
  double term = angleInRadians;
  double sum = angleInRadians;
  for (int order = 1; order <= 12; ++order) {
    term *= -x2 / static_cast<double>((2 * order) * ((2 * order) + 1));
    sum += term;
  }
  return sum;
}

/**
 * @brief Evaluates the cosine Taylor series for compile-time table generation.
 * @param angleInRadians Argument in radians, accurate for |angleInRadians| <= pi/4.
 * @return The cosine of @p angleInRadians.
 */
constexpr double TaylorCos(double angleInRadians) noexcept {
  const double x2 = angleInRadians * angleInRadians;
  double term = 1.0;
  double sum = 1.0;
  for (int order = 1; order <= 12; ++order) {
    term *= -x2 / static_cast<double>(((2 * order) - 1) * (2 * order));
    sum += term;
  }
  return sum;
}

/**
 * @brief Computes a sine table entry using octant reduction.
 * @param index Table index; entry @p index represents sin(2*pi*index/kSinTableSize).
 * @return The sine value of the table entry.
 */
constexpr double SinTableEntry(std::size_t index) noexcept {
  constexpr double kPi = std::numbers::pi_v<double>;

  std::size_t sinTableIndex = index % kSinTableSize;
  bool negate = false;

  if (sinTableIndex >= kSinTableSize / 2U) {
    sinTableIndex -= kSinTableSize / 2U;
    negate = true;
  }
  if (sinTableIndex > kSinTableSize / 4U) {
    sinTableIndex = (kSinTableSize / 2U) - sinTableIndex;
  }

  const double angleRadians = (2.0 * kPi * static_cast<double>(sinTableIndex)) / static_cast<double>(kSinTableSize);
  const double computedAngleValue = (sinTableIndex <= kSinTableSize / 8U) ? TaylorSin(angleRadians) : TaylorCos((kPi / 2.0) - angleRadians);
  return negate ? -computedAngleValue : computedAngleValue;
}

/// Sine over one full revolution plus a quarter-revolution tail for cosine lookup.
inline constexpr std::array<float, kSinTableSize + kSinTableQuarterSize + 1U> kSinTable = [] {
  std::array<float, kSinTableSize + kSinTableQuarterSize + 1U> table{};
  for (std::size_t i = 0U; i <= kSinTableSize + kSinTableQuarterSize; ++i) {
    table.at(i) = static_cast<float>(SinTableEntry(i));
  }
  return table;
}();

/**
 * @brief Interpolates a table segment linearly.
 * @param f1 Function value at the start of the segment.
 * @param f2 Function value at the end of the segment.
 * @param h Normalised position inside the segment, in [0, 1).
 * @return The interpolated function value.
 */
constexpr float LinearInterpolate(float f1, float f2, float h) noexcept { return f1 + (h * (f2 - f1)); }

/**
 * @brief Rounds a float to the nearest integer, away from zero on ties.
 * @param value Value to round.
 * @return The rounded value.
 */
constexpr std::int64_t RoundToInt64(float value) noexcept { return static_cast<std::int64_t>(value >= 0.0F ? value + 0.5F : value - 0.5F); }

}  // namespace detail

/**
 * @brief Electrical rotor angle stored as a Q1.31 phase plus a revolution counter.
 *
 * The Q1.31 phase covers exactly one revolution, so the full 32-bit range maps to
 * [-pi, pi): `INT32_MIN` is -pi and the value wraps back to -pi after +pi. Wrapping is
 * therefore free and the accumulated position never drifts, because every advance is
 * applied in integer arithmetic. The revolution counter records how often the +pi/-pi
 * seam has been crossed.
 *
 * This type represents the electrical angle only. Converting to or from a mechanical
 * angle, including any pole-pair scaling, is the responsibility of the caller.
 */
class RotorAngle {
 public:
  /** @brief Constructs a zero angle with zero revolutions. */
  constexpr RotorAngle() noexcept = default;

  /**
   * @brief Constructs a rotor angle from an angle relative to zero.
   * @param angle Electrical angle; magnitudes beyond one revolution set the counter.
   * @return The corresponding rotor angle.
   */
  static constexpr RotorAngle FromAngle(unit::Angle angle) noexcept {
    RotorAngle result;
    result.Advance(angle);
    return result;
  }

  /**
   * @brief Constructs a rotor angle from its raw fixed-point state.
   * @param raw Q1.31 phase inside the current revolution.
   * @param revolutions Number of completed revolutions.
   * @return The corresponding rotor angle.
   */
  static constexpr RotorAngle FromRaw(std::int32_t raw, std::int32_t revolutions = 0) noexcept {
    RotorAngle result;
    result.raw_ = raw;
    result.revolutions_ = revolutions;
    result.UpdateSinCos();
    return result;
  }

  /**
   * @brief Advances the angle by an offset and updates the revolution counter.
   * @param offset Angle to add; may span any number of revolutions.
   *
   * @note The offset is converted from a float and rounded to the nearest Q1.31 count, so a
   * single call carries at most half a count of quantisation error. That error does not
   * compound, because the phase itself is accumulated in integer counts.
   */
  constexpr void Advance(unit::Angle offset) noexcept {
    const float revolutions = offset.Value() * detail::kRevolutionsPerRadian;
    std::int64_t carry = static_cast<std::int64_t>(revolutions);
    const float fraction = revolutions - static_cast<float>(carry);
    std::int64_t counts = detail::RoundToInt64(fraction * detail::kCountsPerRevolutionF);

    // Keep the residual within half a revolution so it fits a single wrapping add.
    if (counts >= detail::kCountsPerHalfRevolution) {
      counts -= detail::kCountsPerRevolution;
      ++carry;
    } else if (counts < -detail::kCountsPerHalfRevolution) {
      counts += detail::kCountsPerRevolution;
      --carry;
    }

    revolutions_ += static_cast<std::int32_t>(carry);
    AddRaw(static_cast<std::int32_t>(counts));
    UpdateSinCos();
  }

  /**
   * @brief Returns the angle inside the current revolution.
   * @return The angle in [-pi, pi).
   */
  constexpr unit::Angle AngleInRevolution() const noexcept { return unit::Angle(static_cast<float>(raw_) * detail::kRadiansPerCount); }

  /**
   * @brief Returns the number of completed revolutions.
   * @return The signed revolution count.
   */
  constexpr std::int32_t Revolutions() const noexcept { return revolutions_; }

  /**
   * @brief Returns the absolute angle including all completed revolutions.
   * @return The absolute angle in radians.
   *
   * @note This getter is a float convenience. It loses resolution once the revolution
   * count grows large; use AbsoluteRaw() when exactness is required.
   */
  constexpr unit::Angle AbsoluteAngle() const noexcept {
    return unit::Angle((static_cast<float>(revolutions_) * detail::kRadiansPerRevolution) + AngleInRevolution().Value());
  }

  /**
   * @brief Returns the Q1.31 phase inside the current revolution.
   * @return The raw fixed-point phase.
   */
  constexpr std::int32_t Raw() const noexcept { return raw_; }

  /**
   * @brief Returns the exact absolute position in Q1.31 counts.
   * @return The absolute position, counting 2^32 counts per revolution.
   */
  constexpr std::int64_t AbsoluteRaw() const noexcept {
    return (static_cast<std::int64_t>(revolutions_) * detail::kCountsPerRevolution) + static_cast<std::int64_t>(raw_);
  }

  /**
   * @brief Returns the sine of the angle inside the current revolution.
   * @return The interpolated sine value.
   */
  constexpr unit::DimensionlessRatio Sin() const noexcept { return sin_; }

  /**
   * @brief Returns the cosine of the angle inside the current revolution.
   * @return The interpolated cosine value.
   */
  constexpr unit::DimensionlessRatio Cos() const noexcept { return cos_; }

  /**
   * @brief Advances this angle by an offset.
   * @param offset Angle to add.
   * @return This angle after the advance.
   */
  constexpr RotorAngle& operator+=(unit::Angle offset) noexcept {
    Advance(offset);
    return *this;
  }

  /**
   * @brief Retards this angle by an offset.
   * @param offset Angle to subtract.
   * @return This angle after the advance.
   */
  constexpr RotorAngle& operator-=(unit::Angle offset) noexcept {
    Advance(-offset);
    return *this;
  }

  /**
   * @brief Returns this angle advanced by an offset.
   * @param offset Angle to add.
   * @return The advanced angle.
   */
  constexpr RotorAngle operator+(unit::Angle offset) const noexcept {
    RotorAngle result = *this;
    result.Advance(offset);
    return result;
  }

  /**
   * @brief Returns this angle retarded by an offset.
   * @param offset Angle to subtract.
   * @return The retarded angle.
   */
  constexpr RotorAngle operator-(unit::Angle offset) const noexcept {
    RotorAngle result = *this;
    result.Advance(-offset);
    return result;
  }

  /**
   * @brief Returns the shortest-path difference between two angles.
   * @param other Angle to subtract.
   * @return The difference in (-pi, pi], ignoring the revolution counters.
   */
  constexpr unit::Angle operator-(const RotorAngle& other) const noexcept {
    const std::int32_t delta = static_cast<std::int32_t>(static_cast<std::uint32_t>(raw_) - static_cast<std::uint32_t>(other.raw_));
    return unit::Angle(static_cast<float>(delta) * detail::kRadiansPerCount);
  }

  /**
   * @brief Compares two rotor angles for equality.
   * @param other Angle to compare.
   * @return `true` when phase and revolution count match.
   */
  constexpr bool operator==(const RotorAngle& other) const noexcept { return (raw_ == other.raw_) && (revolutions_ == other.revolutions_); }

  /**
   * @brief Compares two rotor angles for inequality.
   * @param other Angle to compare.
   * @return `true` when phase or revolution count differ.
   */
  constexpr bool operator!=(const RotorAngle& other) const noexcept { return !(*this == other); }

 private:
  /**
   * @brief Adds a raw phase delta and counts a seam crossing.
   * @param delta Q1.31 phase delta with a magnitude of at most half a revolution.
   */
  constexpr void AddRaw(std::int32_t delta) noexcept {
    // Wrapping is done unsigned because signed overflow is undefined behaviour.
    const std::int32_t next = static_cast<std::int32_t>(static_cast<std::uint32_t>(raw_) + static_cast<std::uint32_t>(delta));
    if (((raw_ ^ next) & (delta ^ next)) < 0) {
      revolutions_ += (delta >= 0) ? 1 : -1;
    }
    raw_ = next;
  }

  /** @brief Recomputes sine and cosine from the lookup table. */
  constexpr void UpdateSinCos() noexcept {
    const std::uint32_t phase = static_cast<std::uint32_t>(raw_);
    const std::size_t table_index = phase >> detail::kSinTableFractionBits;
    const float h = static_cast<float>(phase & detail::kSinTableFractionMask) * detail::kSinTableFractionScale;
    const float* const table = detail::kSinTable.data() + table_index;

    sin_ = unit::DimensionlessRatio(detail::LinearInterpolate(table[0U], table[1U], h));
    cos_ = unit::DimensionlessRatio(detail::LinearInterpolate(table[detail::kSinTableQuarterSize], table[detail::kSinTableQuarterSize + 1U], h));
  }

  /// Q1.31 phase inside the current revolution.
  std::int32_t raw_{0};
  /// Number of completed revolutions.
  std::int32_t revolutions_{0};
  /// Sine of the phase inside the current revolution.
  unit::DimensionlessRatio sin_{0.0F};
  /// Cosine of the phase inside the current revolution.
  unit::DimensionlessRatio cos_{1.0F};
};

}  // namespace unimoc::system
