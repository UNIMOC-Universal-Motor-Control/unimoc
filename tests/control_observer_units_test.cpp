/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // //  /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file control_observer_units_test.cpp
 *    @brief Tests for unit-typed control and observer configuration.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */

#include <gtest/gtest.h>
#include <type_traits>
#include "dead_time_compensation.hpp"
#include "hfi.hpp"
#include "mechanical_observer.hpp"
#include "nvm_settings.hpp"
#include "svm.hpp"

namespace unimoc::test {

TEST(ControlObserverUnitsTest, MechanicalObserverUsesUnitTypes) {
  static_assert(std::is_same_v<decltype(observer::MechanicalObserver<float>::psi), unit::MagneticFlux>);
  static_assert(std::is_same_v<decltype(observer::MechanicalObserver<float>::J), unit::Inertia>);
  static_assert(std::is_same_v<decltype(observer::MechanicalObserver<float>::omega), unit::AngularVelocity>);
  static_assert(std::is_same_v<decltype(observer::MechanicalObserver<float>::theta), unit::Angle>);
  static_assert(std::is_same_v<decltype(observer::MechanicalObserver<float>::m_l), unit::Torque>);
}

TEST(ControlObserverUnitsTest, AlgorithmsLoadInitialSettingsFromNvm) {
  settings::NvmSettings settings{};
  settings.flux_pm = unit::MagneticFlux{0.012F};
  settings.l_d = unit::Inductance{0.002F};
  settings.l_q = unit::Inductance{0.003F};
  settings.motor_j = unit::Inertia{0.004F};
  settings.motor_omega_max = unit::AngularVelocity{123.0F};
  settings.motor_omega_min = unit::AngularVelocity{-45.0F};
  settings.mech_obs_q = 0.005F;
  settings.mech_obs_r = 0.006F;
  settings.dtc_dead_time = unit::Time{0.000002F};
  settings.dtc_f_pwm = unit::Frequency{25000.0F};
  settings.dtc_i_threshold = unit::Current{0.7F};
  settings.hfi_v_inject = unit::Voltage{2.5F};
  settings.hfi_error_gain = 3.0F;
  settings.svm_duty_min = unit::DimensionlessRatio{0.1F};
  settings.svm_duty_max = unit::DimensionlessRatio{0.9F};

  observer::MechanicalObserver<float> mechanical_observer;
  mechanical_observer.init(settings);
  EXPECT_FLOAT_EQ(mechanical_observer.psi.Value(), 0.012F);
  EXPECT_FLOAT_EQ(mechanical_observer.L_d.Value(), 0.002F);
  EXPECT_FLOAT_EQ(mechanical_observer.L_q.Value(), 0.003F);
  EXPECT_FLOAT_EQ(mechanical_observer.J.Value(), 0.004F);
  EXPECT_FLOAT_EQ(mechanical_observer.omega_max.Value(), 123.0F);
  EXPECT_FLOAT_EQ(mechanical_observer.omega_min.Value(), -45.0F);
  EXPECT_FLOAT_EQ(mechanical_observer.Q, 0.005F);
  EXPECT_FLOAT_EQ(mechanical_observer.R, 0.006F);

  control::DeadTimeCompensation<float> dead_time_compensation;
  dead_time_compensation.init(settings);
  EXPECT_FLOAT_EQ(dead_time_compensation.dead_time.Value(), 0.000002F);
  EXPECT_FLOAT_EQ(dead_time_compensation.f_pwm.Value(), 25000.0F);
  EXPECT_FLOAT_EQ(dead_time_compensation.i_threshold.Value(), 0.7F);

  observer::Hfi<float> hfi;
  hfi.init(settings);
  EXPECT_FLOAT_EQ(hfi.v_inject.Value(), 2.5F);
  EXPECT_FLOAT_EQ(hfi.error_gain, 3.0F);

  control::Svm<float> svm;
  svm.init(settings);
  EXPECT_FLOAT_EQ(svm.duty_min.Value(), 0.1F);
  EXPECT_FLOAT_EQ(svm.duty_max.Value(), 0.9F);
}

}  // namespace unimoc::test