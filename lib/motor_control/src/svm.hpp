/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / /_/ / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file svm.hpp
 *    @brief Unit-typed space-vector PWM modulator.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <algorithm>
#include <array>
#include <concepts>
#include "nvm_settings.hpp"
#include "stator_system.hpp"
#include "three_phase_system.hpp"

/**
 * @namespace unimoc global namespace
 */
namespace unimoc {
/**
 * @namespace control control algorithms namespace
 */
namespace control {

using namespace unit;

/**
 * @brief Space Vector Modulation (SVPWM) with centered PWM.
 *
 * Converts a stationary-frame voltage reference (α/β, normalised to the DC-link
 * voltage) into three PWM duty cycles.
 *
 * Conventions
 * -----------
 * - v_alpha and v_beta are normalised by the DC bus voltage V_dc so that
 *   a value of 1.0 represents V_dc.  The full linear SVPWM range corresponds to
 *   a vector magnitude of 1/√3 ≈ 0.577.
 * - The zero-sequence offset is chosen so that the sum of the three duty cycles
 *   is always 1.5 (each centred around 0.5), giving centred / symmetric PWM.
 * - After centring the output duty cycles are clamped to [duty_min, duty_max]
 *   (default 5 % … 95 %) to leave headroom for current measurement and dead-time
 *   compensation without preloading the timer counter.
 */
struct Svm {
  /// Minimum duty cycle (keeps time for ADC sampling and dead-time headroom).
  unit::DimensionlessRatio duty_min{0.05_ratio};
  /// Maximum duty cycle (symmetric headroom on the upper side).
  unit::DimensionlessRatio duty_max{0.95_ratio};

  /**
   * @brief Load SVM duty limits from NVM settings.
   *
   * @param settings Validated NVM settings.
   */
  constexpr void init(const settings::NvmSettings& settings) noexcept {
    duty_min = settings.svm_duty_min;
    duty_max = settings.svm_duty_max;
  }

  /**
   * @brief Compute three-phase duty cycles from a stationary-frame voltage vector.
   *
   * @param v  Voltage vector with alpha/beta components normalised by V_dc.
   * @return   Three-phase duty cycles [0, 1] clamped to [duty_min, duty_max].
   */
  [[nodiscard]] constexpr system::ThreePhase<unit::DimensionlessRatio> calculate(const system::Stator<unit::DimensionlessRatio>& v) const noexcept {
    // --- Inverse Clarke (amplitude-invariant) ---
    // Transforms the α/β reference into three phase-voltage references.
    const auto phase_voltages = v.ToThreePhase();
    unit::DimensionlessRatio va = phase_voltages.a;
    unit::DimensionlessRatio vb = phase_voltages.b;
    unit::DimensionlessRatio vc = phase_voltages.c;

    // --- Zero-sequence injection for centred SVM ---
    // The zero-sequence component centres the modulated waveforms so that the
    // mid-point of (max + min) is always at 0.  Adding it to each phase shifts
    // all duties to be symmetric around 0.5.
    unit::DimensionlessRatio vmax = std::max({va, vb, vc});
    unit::DimensionlessRatio vmin = std::min({va, vb, vc});
    unit::DimensionlessRatio v0 = (vmax + vmin) * -0.5F;

    // Convert phase voltages [-0.5, 0.5] → duty cycles [0, 1]
    unit::DimensionlessRatio da = 0.5_ratio + va + v0;
    unit::DimensionlessRatio db = 0.5_ratio + vb + v0;
    unit::DimensionlessRatio dc = 0.5_ratio + vc + v0;

    // --- Clamp to [duty_min, duty_max] ---
    da = da.Clamp(duty_min.Value(), duty_max.Value());
    db = db.Clamp(duty_min.Value(), duty_max.Value());
    dc = dc.Clamp(duty_min.Value(), duty_max.Value());

    return system::ThreePhase<unit::DimensionlessRatio>{da, db, dc};
  }
};

}  // namespace control
}  // namespace unimoc
