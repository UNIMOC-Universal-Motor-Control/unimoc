/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file main.cpp
 *    @brief Firmware application entry point.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#include <modm/debug.hpp>
#include <modm/platform.hpp>
#include "hardware_interface.hpp"
#include "settings_store.hpp"

using namespace modm::platform;
using namespace std::chrono_literals;

int main() {
  unimoc::system::SettingsStore settings_store{unimoc::hardware::settings.GetSettingsProfile(), unimoc::hardware::settings.GetSettingsStorage()};
  const auto settings_status = settings_store.Load();
  if (settings_status != unimoc::system::SettingsStatus::kSuccess && settings_status != unimoc::system::SettingsStatus::kFactoryDefaults) {
    MODM_LOG_ERROR << "Failed to load motor settings\n";
    return 1;
  }

  for (auto& motor : unimoc::hardware::motor) {
    if (!motor.Initialize()) {
      MODM_LOG_ERROR << "Failed to initialize motor interface\n";
      return 1;  // Exit if initialization fails
    }
  }

  while (true) {
    modm::delay(0.5s);
  }
}