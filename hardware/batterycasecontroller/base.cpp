/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file base.cpp
 *    @brief BatteryCaseController hardware interface implementation.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include <modm/debug.hpp>
#include <modm/platform.hpp>
#include "analog.hpp"
#include "hardware_interface.hpp"
#include "pulse_width.hpp"

using namespace modm::platform;
using namespace std::chrono_literals;
using namespace modm::literals;

namespace unimoc::hardware {

namespace {

system::SettingsStorageStatus LoadSettings(void* const, const std::span<std::byte>) noexcept { return system::SettingsStorageStatus::kUnavailable; }

system::SettingsStorageStatus SaveSettings(void* const, const std::span<const std::byte>) noexcept {
  return system::SettingsStorageStatus::kUnavailable;
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

bool Initialize(void);

HardwareInterface motor[MOTORS] = {{// Initialize function
                                    initialize,

                                    // Get phase currents function
                                    analog::GetPhaseCurrents,

                                    // Get phase voltages function
                                    analog::GetPhaseVoltages,

                                    // Set phase duties function
                                    pulse_width::SetPhaseDuties}};

/**
 * Initializes the hardware components of the battery case controller.
 *
 * This function sets up the system clock, initializes the ADCs, and configures
 * the pulse width modulation (PWM) for motor control.
 */
bool Initialize(void) {
  if (!SystemClock::enable()) {
    MODM_LOG_ERROR << "Failed to enable system clock." << modm::endl;
    return false;  // Return false if system clock initialization fails
  }

  // Initialize the ADCs
  if (!analog::Initialize()) {
    MODM_LOG_ERROR << "Failed to initialize ADCs." << modm::endl;
    return false;  // Return false if ADC initialization fails
  }

  // Initialize the pulse width modulation (PWM) for motor control
  if (!pulse_width::Initialize()) {
    MODM_LOG_ERROR << "Failed to initialize PWM." << modm::endl;
    return false;  // Return false if PWM initialization fails
  }

  MODM_LOG_INFO << "Hardware initialization successful." << modm::endl;
  return true;  // Return true if all initializations are successful
}

}  // namespace unimoc::hardware

// Include the necessary headers for RTT logging
using LoggerDevice = modm::IODeviceWrapper<Rtt<0>, modm::IOBuffer::DiscardIfFull>;
static LoggerDevice rtt_device;
// Set all four logger streams to use RTT
modm::log::Logger modm::log::debug(rtt_device);
modm::log::Logger modm::log::info(rtt_device);
modm::log::Logger modm::log::warning(rtt_device);
modm::log::Logger modm::log::error(rtt_device);
