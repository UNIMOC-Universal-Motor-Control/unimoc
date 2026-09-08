#pragma once

#ifndef UNIMOC_TEST_EXCITATION_CONTROLLER_H_
#define UNIMOC_TEST_EXCITATION_CONTROLLER_H_

#include <gtest/gtest.h>
#include <cmath>
#include "excitation_controller.hpp"

namespace unimoc {
namespace control {
namespace test {

class ExcitationControllerTest : public ::testing::Test {
 protected:
  using Ctrl = ExcitationController<float>;

  Ctrl make_current_mode() {
    Ctrl c;
    c.mode = ExcitationMode::CurrentMode;
    c.kp = unimoc::unit::DimensionlessRatio{5.0F};
    c.ki = unimoc::unit::InverseTime{50.0F};
    c.i_f_min = unimoc::unit::Current{0.0F};
    c.i_f_max = unimoc::unit::Current{10.0F};
    c.L_m = unimoc::unit::Inductance{47.0e-3F};
    c.current_setpoint = unimoc::unit::Current{};
    return c;
  }

  Ctrl make_flux_mode() {
    Ctrl c = make_current_mode();
    c.mode = ExcitationMode::FluxMode;
    // ψ_f* = 2.35 Wb  →  I_f* = 2.35 / 0.047 = 50 A (clamped to 10 A)
    c.flux_setpoint = unimoc::unit::MagneticFlux{0.47F};  // ψ_f* [Wb] → I_f* = 0.47/0.047 = 10 A
    return c;
  }
};

// --- CurrentMode: zero setpoint, zero measurement → zero output
TEST_F(ExcitationControllerTest, CurrentModeZeroError) {
  auto c = make_current_mode();
  const float out = c.update(unimoc::unit::Current{}, unimoc::unit::Time{1.0e-4F}).Value();
  EXPECT_FLOAT_EQ(out, 0.0f);
}

// --- CurrentMode: positive setpoint drives positive output
TEST_F(ExcitationControllerTest, CurrentModePositiveSetpoint) {
  auto c = make_current_mode();
  c.current_setpoint = unimoc::unit::Current{5.0F};
  const float out = c.update(unimoc::unit::Current{}, unimoc::unit::Time{1.0e-4F}).Value();
  EXPECT_GT(out, 0.0f);
  EXPECT_LE(out, c.i_f_max.Value());
}

// --- CurrentMode: integrator saturates at i_f_max
TEST_F(ExcitationControllerTest, CurrentModeIntegratorClamp) {
  auto c = make_current_mode();
  c.current_setpoint = unimoc::unit::Current{100.0F};  // Large error
  for (int i = 0; i < 1000; ++i) c.update(unimoc::unit::Current{}, unimoc::unit::Time{1.0e-3F});
  EXPECT_FLOAT_EQ(c.i_f_ref.Value(), c.i_f_max.Value());
  EXPECT_LE(c.integrator.Value(), c.i_f_max.Value());
}

// --- CurrentMode: output never goes below i_f_min
TEST_F(ExcitationControllerTest, CurrentModeMinClamp) {
  auto c = make_current_mode();
  c.current_setpoint = unimoc::unit::Current{-100.0F};  // drive negative
  for (int i = 0; i < 1000; ++i) c.update(unimoc::unit::Current{}, unimoc::unit::Time{1.0e-3F});
  EXPECT_FLOAT_EQ(c.i_f_ref.Value(), c.i_f_min.Value());
}

// --- FluxMode: ψ_f* / L_m = correct current reference
TEST_F(ExcitationControllerTest, FluxModeConversion) {
  auto c = make_flux_mode();
  // setpoint = 0.47 Wb, L_m = 0.047 H → I_f* = 10 A (exactly at max)
  for (int i = 0; i < 5000; ++i) c.update(unimoc::unit::Current{}, unimoc::unit::Time{1.0e-4F});
  EXPECT_FLOAT_EQ(c.i_f_ref.Value(), c.i_f_max.Value());
}

// --- reset() clears integrator and output
TEST_F(ExcitationControllerTest, ResetClearsState) {
  auto c = make_current_mode();
  c.current_setpoint = unimoc::unit::Current{5.0F};
  for (int i = 0; i < 100; ++i) c.update(unimoc::unit::Current{}, unimoc::unit::Time{1.0e-3F});
  c.reset();
  EXPECT_FLOAT_EQ(c.integrator.Value(), 0.0F);
  EXPECT_FLOAT_EQ(c.i_f_ref.Value(), 0.0F);
}

// --- Output converges to setpoint when feedback equals reference
TEST_F(ExcitationControllerTest, CurrentModeConverges) {
  auto c = make_current_mode();
  c.current_setpoint = unimoc::unit::Current{3.0F};
  float meas = 0.0f;

  for (int i = 0; i < 5000; ++i) {
    const float ref = c.update(unimoc::unit::Current{meas}, unimoc::unit::Time{1.0e-4F}).Value();
    // Simulate first-order plant: meas tracks ref slowly
    meas += 0.01f * (ref - meas);
  }
  EXPECT_NEAR(meas, 3.0f, 0.1f);
}

}  // namespace test
}  // namespace control
}  // namespace unimoc

#endif /* UNIMOC_TEST_EXCITATION_CONTROLLER_H_ */
