#pragma once

#ifndef UNIMOC_TEST_EXCITATION_OBSERVER_H_
#define UNIMOC_TEST_EXCITATION_OBSERVER_H_

#include <gtest/gtest.h>
#include <cmath>
#include "excitation_observer.hpp"

namespace unimoc {
namespace observer {
namespace test {

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
  o.tau = unimoc::unit::Time{5.0e-3F};  // 5 ms
  o.L_m = unimoc::unit::Inductance{47.0e-3F};
  o.update(unimoc::unit::Current{10.0F}, unimoc::unit::Time{1.0e-4F});  // dt = 0.1 ms
  EXPECT_GT(o.i_f_hat.Value(), 0.0F);
  EXPECT_LT(o.i_f_hat.Value(), 10.0F);
}

// --- After many steps the output converges to the measurement
TEST_F(ExcitationObserverTest, ConvergesAfterManySteps) {
  Obs o;
  o.tau = unimoc::unit::Time{1.0e-3F};
  o.L_m = unimoc::unit::Inductance{47.0e-3F};
  for (int i = 0; i < 10000; ++i) o.update(unimoc::unit::Current{8.0F}, unimoc::unit::Time{1.0e-5F});  // total time >> tau
  EXPECT_NEAR(o.i_f_hat.Value(), 8.0F, 1.0e-3F);
  EXPECT_NEAR(o.psi_f_hat.Value(), 8.0F * 47e-3F, 1.0e-4F);
}

// --- psi_f_hat = L_m * i_f_hat at every step
TEST_F(ExcitationObserverTest, PsiEqualsLmTimesI) {
  Obs o;
  o.tau = unimoc::unit::Time{2.0e-3F};
  o.L_m = unimoc::unit::Inductance{50.0e-3F};
  for (int i = 0; i < 20; ++i) {
    o.update(unimoc::unit::Current{5.0F}, unimoc::unit::Time{1.0e-4F});
    EXPECT_NEAR(o.psi_f_hat.Value(), o.L_m.Value() * o.i_f_hat.Value(), 1.0e-7F);
  }
}

// --- Zero time constant passes measurement through immediately
TEST_F(ExcitationObserverTest, ZeroTauPassThrough) {
  Obs o;
  o.tau = unimoc::unit::Time{};
  o.L_m = unimoc::unit::Inductance{47.0e-3F};
  o.update(unimoc::unit::Current{7.5F}, unimoc::unit::Time{1.0e-4F});
  EXPECT_FLOAT_EQ(o.i_f_hat.Value(), 7.5F);
}

// --- reset() with init value
TEST_F(ExcitationObserverTest, ResetWithInitValue) {
  Obs o;
  o.tau = unimoc::unit::Time{2.0e-3F};
  o.L_m = unimoc::unit::Inductance{47.0e-3F};
  for (int i = 0; i < 100; ++i) o.update(unimoc::unit::Current{5.0F}, unimoc::unit::Time{1.0e-4F});
  o.reset(unimoc::unit::Current{3.0F});
  EXPECT_FLOAT_EQ(o.i_f_hat.Value(), 3.0F);
  EXPECT_NEAR(o.psi_f_hat.Value(), 3.0F * 47.0e-3F, 1.0e-7F);
}

}  // namespace test
}  // namespace observer
}  // namespace unimoc

#endif /* UNIMOC_TEST_EXCITATION_OBSERVER_H_ */
