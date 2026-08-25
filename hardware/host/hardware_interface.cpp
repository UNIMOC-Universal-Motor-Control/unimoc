/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file hardware_interface.cpp
 *    @brief Hosted hardware interface implementation.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "hardware_interface.hpp"
#include <cstdlib>
#include <fstream>

namespace unimoc {
namespace hardware {

namespace {

const char* GetSettingsPath() noexcept {
  const char* const path = std::getenv("UNIMOC_SETTINGS_FILE");
  return path == nullptr ? "unimoc_settings.bin" : path;
}

system::SettingsStorageStatus LoadSettings(void* const, const std::span<std::byte> image) noexcept {
  std::ifstream input{GetSettingsPath(), std::ios::binary};
  if (!input.is_open()) return system::SettingsStorageStatus::kNotFound;

  input.read(reinterpret_cast<char*>(image.data()), static_cast<std::streamsize>(image.size()));
  return input.gcount() == static_cast<std::streamsize>(image.size()) ? system::SettingsStorageStatus::kSuccess
                                                                      : system::SettingsStorageStatus::kIoError;
}

system::SettingsStorageStatus SaveSettings(void* const, const std::span<const std::byte> image) noexcept {
  std::ofstream output{GetSettingsPath(), std::ios::binary | std::ios::trunc};
  if (!output.is_open()) return system::SettingsStorageStatus::kIoError;

  output.write(reinterpret_cast<const char*>(image.data()), static_cast<std::streamsize>(image.size()));
  return output.good() ? system::SettingsStorageStatus::kSuccess : system::SettingsStorageStatus::kIoError;
}

const system::SettingsProfile kSettingsProfile = [] {
  system::SettingsProfile profile{};
  profile.factory_settings.motor_i_max = unit::Current{80.0F};
  profile.factory_settings.battery_drive_current_max = unit::Current{50.0F};
  profile.factory_settings.battery_charge_current_max = unit::Current{20.0F};
  profile.capabilities.max_phase_current = unit::Current{100.0F};
  profile.capabilities.max_motor_current = unit::Current{80.0F};
  profile.capabilities.max_battery_drive_current = unit::Current{50.0F};
  profile.capabilities.max_battery_charge_current = unit::Current{20.0F};
  return profile;
}();

const system::SettingsStorage kSettingsStorage{nullptr, LoadSettings, SaveSettings};

}  // namespace

HardwareSettingsInterface settings{kSettingsProfile, kSettingsStorage};

// Stub function implementations
static bool InitializeStub() {
  // Stub: Always return successful initialization
  return true;
}

static system::ThreePhase<unit::Current> GetPhaseCurrentsStub() {
  // Stub: Return zero currents
  return system::ThreePhase<unit::Current>{unit::Current{0.0f}, unit::Current{0.0f}, unit::Current{0.0f}};
}

static system::ThreePhase<unit::Voltage> GetPhaseVoltagesStub() {
  // Stub: Return zero voltages
  return system::ThreePhase<unit::Voltage>{unit::Voltage{0.0f}, unit::Voltage{0.0f}, unit::Voltage{0.0f}};
}

static void SetPhaseDutiesStub(const system::ThreePhase<unit::DimensionlessRatio>& duties) {
  // Stub: Accept duties but do nothing
  (void)duties;
}

// Initialize the motor array with stub implementations
HardwareInterface motor[MOTORS] = {HardwareInterface(InitializeStub, GetPhaseCurrentsStub, GetPhaseVoltagesStub, SetPhaseDutiesStub)};

}  // namespace hardware
}  // namespace unimoc