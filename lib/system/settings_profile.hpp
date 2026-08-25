/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file settings_profile.hpp
 *    @brief Immutable target capabilities and factory settings.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include "nvm_settings.hpp"

namespace unimoc::system {

/**
 * @brief Immutable electrical capabilities of a hardware target.
 *
 * A target provides these values in a linked, read-only profile. They are
 * safety limits, not runtime settings, and therefore are never writable by a
 * settings operation or a Cyphal register.
 */
struct HardwareCapabilities {
  /// Absolute phase-current limit of the inverter [A].
  unit::Current max_phase_current{unit::Current{100.0F}};

  /// Absolute resultant motor-current limit [A].
  unit::Current max_motor_current{unit::Current{40.0F}};

  /// Absolute battery-discharge current limit [A].
  unit::Current max_battery_drive_current{unit::Current{15.0F}};

  /// Absolute regenerative battery-charge current limit [A].
  unit::Current max_battery_charge_current{unit::Current{5.0F}};
};

/**
 * @brief Read-only target profile used to initialize a settings store.
 *
 * The target-specific instance should be defined by the hardware adapter. Its
 * factory settings are copied into writable settings storage only when no
 * valid user settings image exists.
 */
struct SettingsProfile {
  /// Stock settings compiled into the target image.
  NvmSettings factory_settings{};

  /// Hardware safety capabilities for validating runtime settings.
  HardwareCapabilities capabilities{};
};

}  // namespace unimoc::system
