#pragma once

#ifndef UNIMOC_TEST_POSITION_CONTROLLER_H_
#define UNIMOC_TEST_POSITION_CONTROLLER_H_

#include <gtest/gtest.h>
#include <cmath>
#include "position_controller.hpp"
#include "position_tracker.hpp"

namespace unimoc {
namespace control {
namespace test {

class PositionControllerTest : public ::testing::Test {
 protected:
  using Ctrl = PositionController<float>;
  using Tracker = unimoc::observer::PositionTracker<float>;
  using Angle = unimoc::unit::Angle;
  using AngularVelocity = unimoc::unit::AngularVelocity;
  using AngularAcceleration = unimoc::unit::AngularAcceleration;
  using Current = unimoc::unit::Current;
  using Time = unimoc::unit::Time;

  Ctrl make_default() {
    Ctrl c;
    c.kp_pos = unimoc::unit::AngularVelocityPerAngle{10.0F};
    c.kp_speed = unimoc::unit::DimensionlessRatio{5.0F};
    c.ki_speed = unimoc::unit::InverseTime{20.0F};
    c.speed_limit = AngularVelocity{50.0F};
    c.accel_limit = AngularAcceleration{500.0F};
    c.position_tolerance = Angle{0.01F};
    c.speed_tolerance = AngularVelocity{0.5F};
    c.homing_speed = AngularVelocity{3.0F};
    c.pos_ref_rad = Angle{};
    return c;
  }
};

constexpr void set_home_adapter(void* ctx, int pole_pairs) { static_cast<unimoc::observer::PositionTracker<float>*>(ctx)->set_home(pole_pairs); }

// --- At rest, zero setpoint → zero output
TEST_F(PositionControllerTest, ZeroSetpointZeroOutput) {
  auto c = make_default();
  const float out = c.update(Angle{}, AngularVelocity{}, Time{1.0e-4F}).Value();
  EXPECT_FLOAT_EQ(out, 0.0f);
}

// --- Positive position error drives positive omega_ref
TEST_F(PositionControllerTest, PositiveErrorDrivesPositiveOutput) {
  auto c = make_default();
  c.pos_ref_rad = Angle{1.0F};
  const float out = c.update(Angle{}, AngularVelocity{}, Time{1.0e-4F}).Value();
  EXPECT_GT(out, 0.0f);
}

TEST_F(PositionControllerTest, LargeJumpUsesTrapezoidPlanning) {
  auto c = make_default();
  c.trapezoid_jump_threshold = Angle{0.2F};
  c.pos_ref_rad = Angle{10.0F};
  c.update(Angle{}, AngularVelocity{}, Time{1.0e-3F});
  EXPECT_LT(c.pos_ref_limited, c.pos_ref_rad);
}

// --- Output never exceeds speed_limit
TEST_F(PositionControllerTest, OutputClampedToSpeedLimit) {
  auto c = make_default();
  c.pos_ref_rad = Angle{1000.0F};  // huge error
  for (int i = 0; i < 1000; ++i) c.update(Angle{}, AngularVelocity{}, Time{1.0e-3F});
  EXPECT_LE(std::abs(c.omega_ref.Value()), c.speed_limit.Value());
}

// --- in_position flag set when close to target
TEST_F(PositionControllerTest, InPositionFlagSet) {
  auto c = make_default();
  c.pos_ref_rad = Angle{};
  c.update(Angle{}, AngularVelocity{}, Time{1.0e-4F});
  EXPECT_TRUE(c.in_position);
}

// --- in_position not set when position error is large
TEST_F(PositionControllerTest, InPositionFlagClearWhenFar) {
  auto c = make_default();
  c.pos_ref_rad = Angle{10.0F};
  c.update(Angle{}, AngularVelocity{}, Time{1.0e-4F});
  EXPECT_FALSE(c.in_position);
}

// --- Homing: SEARCHING state outputs homing_speed
TEST_F(PositionControllerTest, HomingSearchingOutputsHomingSpeed) {
  auto c = make_default();
  c.start_homing();
  EXPECT_EQ(c.homing_state, HomingState::SEARCHING);
  const float out = c.update(Angle{}, AngularVelocity{}, Time{1.0e-4F}).Value();
  EXPECT_FLOAT_EQ(out, c.homing_speed.Value());
}

// --- Homing: trigger_zeroing advances to ZEROING then DONE
TEST_F(PositionControllerTest, HomingTriggerZeroingAdvancesToDone) {
  auto c = make_default();
  Tracker tracker;
  tracker.update(Angle{0.3F}, 2);
  c.set_home_callback(&set_home_adapter, &tracker, 2);
  c.start_homing();
  c.update(Angle{}, AngularVelocity{}, Time{1.0e-4F});  // SEARCHING step
  c.trigger_zeroing();
  EXPECT_EQ(c.homing_state, HomingState::ZEROING);
  c.update(Angle{}, AngularVelocity{}, Time{1.0e-4F});  // ZEROING step → transitions to DONE
  EXPECT_EQ(c.homing_state, HomingState::DONE);
  EXPECT_TRUE(tracker.is_homed);
}

TEST_F(PositionControllerTest, HomingAutoZeroingOnCurrentThreshold) {
  auto c = make_default();
  c.homing_block_current_threshold = Current{5.0F};
  c.start_homing();
  EXPECT_EQ(c.homing_state, HomingState::SEARCHING);
  c.update(Angle{}, AngularVelocity{}, Time{1.0e-4F}, Current{5.1F});
  EXPECT_EQ(c.homing_state, HomingState::DONE);
}

// --- trigger_zeroing ignored when not SEARCHING
TEST_F(PositionControllerTest, TriggerZeroingIgnoredWhenNotSearching) {
  auto c = make_default();
  c.trigger_zeroing();  // in IDLE
  EXPECT_EQ(c.homing_state, HomingState::IDLE);
}

// --- fault() transitions to FAULT and zeroes output
TEST_F(PositionControllerTest, FaultTransitionAndZeroOutput) {
  auto c = make_default();
  c.start_homing();
  c.fault();
  EXPECT_EQ(c.homing_state, HomingState::FAULT);
  EXPECT_FLOAT_EQ(c.omega_ref.Value(), 0.0F);
}

// --- reset() returns to IDLE
TEST_F(PositionControllerTest, ResetReturnsToIdle) {
  auto c = make_default();
  c.start_homing();
  c.fault();
  c.reset();
  EXPECT_EQ(c.homing_state, HomingState::IDLE);
  EXPECT_FLOAT_EQ(c.speed_integrator.Value(), 0.0F);
  EXPECT_FLOAT_EQ(c.omega_ref.Value(), 0.0F);
}

// --- Closed-loop convergence: shaft reaches target
TEST_F(PositionControllerTest, ConvergesToSetpoint) {
  auto c = make_default();
  float pos = 0.0f;
  float vel = 0.0f;

  c.pos_ref_rad = Angle{2.0F};  // 2 rad target

  for (int i = 0; i < 50000; ++i) {
    const float omega_cmd = c.update(Angle{pos}, AngularVelocity{vel}, Time{1.0e-4F}).Value();
    // Simple first-order motor model
    vel += 0.01f * (omega_cmd - vel);
    pos += vel * 1e-4f;
  }
  EXPECT_NEAR(pos, 2.0f, 0.1f);
}

}  // namespace test
}  // namespace control
}  // namespace unimoc

#endif /* UNIMOC_TEST_POSITION_CONTROLLER_H_ */
