#include "svm.hpp"
#include <gtest/gtest.h>

using Stator = unimoc::system::Stator<unimoc::unit::DimensionlessRatio>;
using Svm = unimoc::control::Svm<float>;

class SvmTest : public ::testing::Test {};

TEST_F(SvmTest, ZeroVoltageProducesCenteredDuties) {
  const Svm svm;

  const auto duties = svm.calculate(Stator{0.0F, 0.0F});

  EXPECT_FLOAT_EQ(duties.a.Value(), 0.5F);
  EXPECT_FLOAT_EQ(duties.b.Value(), 0.5F);
  EXPECT_FLOAT_EQ(duties.c.Value(), 0.5F);
}

TEST_F(SvmTest, AlphaVoltageUsesCenteredZeroSequence) {
  const Svm svm;

  const auto duties = svm.calculate(Stator{0.2F, 0.0F});

  EXPECT_NEAR(duties.a.Value(), 0.65F, 1.0e-6F);
  EXPECT_NEAR(duties.b.Value(), 0.35F, 1.0e-6F);
  EXPECT_NEAR(duties.c.Value(), 0.35F, 1.0e-6F);
  EXPECT_NEAR(duties.a.Value() + duties.c.Value(), 1.0F, 1.0e-6F);
}

TEST_F(SvmTest, DutiesAreClampedToConfiguredRange) {
  unimoc::settings::NvmSettings settings;
  settings.svm_duty_min = unimoc::unit::DimensionlessRatio{0.2F};
  settings.svm_duty_max = unimoc::unit::DimensionlessRatio{0.8F};

  Svm svm;
  svm.init(settings);

  const auto duties = svm.calculate(Stator{1.0F, 0.0F});

  EXPECT_FLOAT_EQ(duties.a.Value(), 0.8F);
  EXPECT_FLOAT_EQ(duties.b.Value(), 0.2F);
  EXPECT_FLOAT_EQ(duties.c.Value(), 0.2F);
}