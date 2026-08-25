/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file settings_store_test.cpp
 *    @brief Tests for settings storage, validation, and mutation boundaries.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include "settings_store.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstring>
#include <ranges>
#include <type_traits>
#include <utility>
#include "settings_codec.hpp"

namespace unimoc::system::test {
namespace {

struct MemoryBackend {
  std::array<std::byte, kSettingsImageSize> image{};
  bool has_image{false};
  bool fail_save{false};
  uint32_t save_count{0U};
};

SettingsStorageStatus LoadImage(void* context, std::span<std::byte> image) noexcept {
  auto& backend = *static_cast<MemoryBackend*>(context);
  if (!backend.has_image) {
    return SettingsStorageStatus::kNotFound;
  }
  std::ranges::copy(backend.image, image.begin());
  return SettingsStorageStatus::kSuccess;
}

SettingsStorageStatus SaveImage(void* context, std::span<const std::byte> image) noexcept {
  auto& backend = *static_cast<MemoryBackend*>(context);
  ++backend.save_count;
  if (backend.fail_save) {
    return SettingsStorageStatus::kIoError;
  }
  std::ranges::copy(image, backend.image.begin());
  backend.has_image = true;
  return SettingsStorageStatus::kSuccess;
}

SettingsProfile MakeProfile() {
  SettingsProfile profile{};
  profile.factory_settings.motor_i_max = unit::Current{30.0F};
  profile.factory_settings.battery_drive_current_max = unit::Current{12.0F};
  profile.factory_settings.battery_charge_current_max = unit::Current{4.0F};
  profile.capabilities.max_phase_current = unit::Current{50.0F};
  profile.capabilities.max_motor_current = unit::Current{30.0F};
  profile.capabilities.max_battery_drive_current = unit::Current{12.0F};
  profile.capabilities.max_battery_charge_current = unit::Current{4.0F};
  return profile;
}

SettingsStorage MakeStorage(MemoryBackend& backend) { return SettingsStorage{&backend, LoadImage, SaveImage}; }

}  // namespace

TEST(SettingsStoreTest, SnapshotExposesOnlyConstSettings) {
  static_assert(std::is_same_v<decltype(std::declval<const SettingsSnapshot&>().Get()), const NvmSettings&>);
}

TEST(SettingsStoreTest, MissingImageUsesAndPersistsFactoryDefaults) {
  MemoryBackend backend{};
  SettingsStore store{MakeProfile(), MakeStorage(backend)};

  EXPECT_EQ(store.Load(), SettingsStatus::kFactoryDefaults);
  EXPECT_TRUE(backend.has_image);
  EXPECT_EQ(store.GetSnapshot().Get().motor_i_max.Value(), 30.0F);
}

TEST(SettingsStoreTest, CodecRoundTripPreservesTypedValues) {
  NvmSettings original{};
  original.stator_r = unit::Resistance{0.23F};
  original.l_d = unit::Inductance{0.0012F};
  original.motor_j = unit::Inertia{0.00042F};
  original.pwm_frequency = unit::Frequency{32000.0F};

  std::array<std::byte, kSettingsImageSize> image{};
  NvmSettings decoded{};

  ASSERT_TRUE(SettingsCodec::Encode(original, image));
  ASSERT_TRUE(SettingsCodec::Decode(image, decoded));
  EXPECT_EQ(decoded.stator_r.Value(), original.stator_r.Value());
  EXPECT_EQ(decoded.l_d.Value(), original.l_d.Value());
  EXPECT_EQ(decoded.motor_j.Value(), original.motor_j.Value());
  EXPECT_EQ(decoded.pwm_frequency, original.pwm_frequency);
}

TEST(SettingsStoreTest, AuthorizedOperationUpdatesAndPersists) {
  MemoryBackend backend{};
  SettingsStore store{MakeProfile(), MakeStorage(backend)};
  ASSERT_EQ(store.Load(), SettingsStatus::kFactoryDefaults);

  auto operations = store.GetOperations();
  EXPECT_EQ(operations.SetMotorCurrentLimit(unit::Current{20.0F}), SettingsStatus::kSuccess);
  EXPECT_EQ(store.GetSnapshot().Get().motor_i_max.Value(), 20.0F);
  EXPECT_EQ(backend.save_count, 2U);
}

TEST(SettingsStoreTest, CapabilityViolationLeavesSnapshotUnchanged) {
  MemoryBackend backend{};
  SettingsStore store{MakeProfile(), MakeStorage(backend)};
  ASSERT_EQ(store.Load(), SettingsStatus::kFactoryDefaults);
  auto operations = store.GetOperations();

  EXPECT_EQ(operations.SetMotorCurrentLimit(unit::Current{31.0F}), SettingsStatus::kOutOfRange);
  EXPECT_EQ(store.GetSnapshot().Get().motor_i_max.Value(), 30.0F);
  EXPECT_EQ(backend.save_count, 1U);
}

TEST(SettingsStoreTest, SaveFailureLeavesSnapshotUnchanged) {
  MemoryBackend backend{};
  SettingsStore store{MakeProfile(), MakeStorage(backend)};
  ASSERT_EQ(store.Load(), SettingsStatus::kFactoryDefaults);
  backend.fail_save = true;

  auto operations = store.GetOperations();
  EXPECT_EQ(operations.SetMotorCurrentLimit(unit::Current{20.0F}), SettingsStatus::kStorageError);
  EXPECT_EQ(store.GetSnapshot().Get().motor_i_max.Value(), 30.0F);
}

TEST(SettingsStoreTest, ResetRestoresFactoryProfile) {
  MemoryBackend backend{};
  SettingsStore store{MakeProfile(), MakeStorage(backend)};
  ASSERT_EQ(store.Load(), SettingsStatus::kFactoryDefaults);
  auto operations = store.GetOperations();
  ASSERT_EQ(operations.SetMotorCurrentLimit(unit::Current{20.0F}), SettingsStatus::kSuccess);

  EXPECT_EQ(operations.ResetToFactoryDefaults(), SettingsStatus::kSuccess);
  EXPECT_EQ(store.GetSnapshot().Get().motor_i_max.Value(), 30.0F);
}

}  // namespace unimoc::system::test
