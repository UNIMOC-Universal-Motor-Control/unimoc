/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file hardware_interface.hpp
 *    @brief Hardware callbacks for motor input and output.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <functional>
#include <utility>
#include "hardware_interface_config.hpp"
#include "settings_profile.hpp"
#include "settings_storage.hpp"
#include "three_phase_system.hpp"

namespace unimoc::hardware {

/**
 * @brief Callback-based hardware interface for one motor.
 *
 * The hardware target supplies the callbacks. The interface owns the callback
 * objects so temporary `std::function` instances cannot leave dangling
 * references in the motor table.
 */
class HardwareInterface {
 public:
  using InitializeCallback = std::function<bool()>;
  using GetPhaseCurrentsCallback = std::function<system::ThreePhase<unit::Current>()>;
  using GetPhaseVoltagesCallback = std::function<system::ThreePhase<unit::Voltage>()>;
  using SetPhaseDutiesCallback = std::function<void(const system::ThreePhase<unit::DimensionlessRatio>&)>;

  /**
   * @brief Constructs a motor hardware interface.
   * @param initialize Initializes the hardware and returns success.
   * @param get_phase_currents Reads the three phase currents.
   * @param get_phase_voltages Reads the three phase voltages.
   * @param set_phase_duties Writes the three phase PWM duties.
   */
  HardwareInterface(InitializeCallback initialize,
                    GetPhaseCurrentsCallback get_phase_currents,
                    GetPhaseVoltagesCallback get_phase_voltages,
                    SetPhaseDutiesCallback set_phase_duties)
      : initialize_(std::move(initialize)),
        get_phase_currents_(std::move(get_phase_currents)),
        get_phase_voltages_(std::move(get_phase_voltages)),
        set_phase_duties_(std::move(set_phase_duties)) {}

  /**
   * @brief Initializes the hardware interface.
   * @return True when initialization succeeds.
   */
  [[nodiscard]] bool Initialize() const { return initialize_(); }

  /**
   * @brief Reads the three phase currents.
   * @return The measured phase currents.
   */
  [[nodiscard]] system::ThreePhase<unit::Current> GetPhaseCurrents() const { return get_phase_currents_(); }

  /**
   * @brief Reads the three phase voltages.
   * @return The measured phase voltages.
   */
  [[nodiscard]] system::ThreePhase<unit::Voltage> GetPhaseVoltages() const { return get_phase_voltages_(); }

  /**
   * @brief Writes the three phase PWM duties.
   * @param duties Normalized duties for phases A, B, and C.
   */
  void SetPhaseDuties(const system::ThreePhase<unit::DimensionlessRatio>& duties) const { set_phase_duties_(duties); }

 private:
  InitializeCallback initialize_;
  GetPhaseCurrentsCallback get_phase_currents_;
  GetPhaseVoltagesCallback get_phase_voltages_;
  SetPhaseDutiesCallback set_phase_duties_;
};

/**
 * @brief Hardware-owned settings profile and persistence boundary.
 *
 * The profile is immutable and normally resides in target read-only memory.
 * The storage callbacks address the separate writable settings image.
 */
class HardwareSettingsInterface {
 public:
  /**
   * @brief Constructs the settings hardware boundary.
   * @param profile Immutable factory settings and hardware capabilities.
   * @param storage Platform callbacks for the writable settings image.
   */
  HardwareSettingsInterface(const system::SettingsProfile& profile, system::SettingsStorage storage) noexcept
      : profile_{profile}, storage_{storage} {}

  /**
   * @brief Returns the immutable target profile.
   */
  [[nodiscard]] const system::SettingsProfile& GetSettingsProfile() const noexcept { return profile_; }

  /**
   * @brief Returns the platform settings storage callbacks.
   */
  [[nodiscard]] const system::SettingsStorage& GetSettingsStorage() const noexcept { return storage_; }

 private:
  const system::SettingsProfile& profile_;
  system::SettingsStorage storage_;
};

extern HardwareSettingsInterface settings;  ///< Node settings hardware boundary.
extern HardwareInterface motor[MOTORS];     ///< Global array of hardware interfaces for motors

}  // namespace unimoc::hardware
