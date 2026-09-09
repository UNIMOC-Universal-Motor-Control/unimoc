#pragma once

#ifndef UNIMOC_TEST_EXCITATION_OBSERVER_H_
#define UNIMOC_TEST_EXCITATION_OBSERVER_H_

#include <gtest/gtest.h>
#include <cmath>
#include "excitation_observer.hpp"

namespace unimoc {
namespace observer {
namespace test {

using namespace unit;

class ExcitationObserverTest : public ::testing::Test {
 protected:
  using Obs = ExcitationObserver<float>;
};

// --- Default-constructed observer has zero outputs
TEST_F(ExcitationObserverTest, DefaultStateIsZero) {
  Obs o;
  EXPECT_FLOAT_EQ(o.i_f_hat.Value(), 0.0F);
  EXPECT_FLOAT_EQ(o.psi_f_hat.Value(), 0.0F);
}

// --- Single step moves toward measurement
TEST_F(ExcitationObserverTest, SingleStepConvergesDirection) {
  Obs o;
  o.tau = 5.0_ms;
  o.L_m = 47.0_mH;
  o.update(10.0_A, 100_us);  // dt = 0.1 ms
  EXPECT_GT(o.i_f_hat.Value(), 0.0F);
  EXPECT_LT(o.i_f_hat.Value(), 10.0F);
}

// --- After many steps the output converges to the measurement
TEST_F(ExcitationObserverTest, ConvergesAfterManySteps) {
  Obs o;
  o.tau = 1.0_ms;
  o.L_m = 47.0_mH;
  for (int i = 0; i < 10000; ++i) o.update(8.0_A, 10_us);  // total time >> tau
  EXPECT_NEAR(o.i_f_hat.Value(), 8.0F, 1.0e-3F);
  EXPECT_NEAR(o.psi_f_hat.Value(), 8.0F * 47e-3F, 1.0e-4F);
}

// --- psi_f_hat = L_m * i_f_hat at every step
TEST_F(ExcitationObserverTest, PsiEqualsLmTimesI) {
  Obs o;
  o.tau = 2.0_ms;
  o.L_m = 50.0_mH;
  for (int i = 0; i < 20; ++i) {
    o.update(5.0_A, 100_us);
    EXPECT_NEAR(o.psi_f_hat.Value(), o.L_m.Value() * o.i_f_hat.Value(), 1.0e-7F);
  }
}

// --- Zero time constant passes measurement through immediately
TEST_F(ExcitationObserverTest, ZeroTauPassThrough) {
  Obs o;
  o.tau = unimoc::unit::Time{};
  o.L_m = 47.0_mH;
  o.update(7.5_A, 100_us);
  EXPECT_FLOAT_EQ(o.i_f_hat.Value(), 7.5F);
}

// --- reset() with init value
TEST_F(ExcitationObserverTest, ResetWithInitValue) {
  Obs o;
  o.tau = 2.0_ms;
  o.L_m = 47.0_mH;
  for (int i = 0; i < 100; ++i) o.update(5.0_A, 100_us);
  o.reset(3.0_A);
  EXPECT_FLOAT_EQ(o.i_f_hat.Value(), 3.0F);
  EXPECT_NEAR(o.psi_f_hat.Value(), 3.0F * 47.0e-3F, 1.0e-7F);
}

}  // namespace test
}  // namespace observer
}  // namespace unimoc

#endif /* UNIMOC_TEST_EXCITATION_OBSERVER_H_ */
