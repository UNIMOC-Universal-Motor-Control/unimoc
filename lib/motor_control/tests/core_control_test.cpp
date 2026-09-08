#include <gtest/gtest.h>
#include <cmath>
#include "current_controller.hpp"
#include "dead_time_compensation.hpp"
#include "field_weakening.hpp"
#include "mtpa.hpp"

using DimensionlessStator = unimoc::system::Stator<unimoc::unit::DimensionlessRatio>;
using CurrentStator = unimoc::system::Stator<unimoc::unit::Current>;
using CurrentRotor = unimoc::system::Rotor<unimoc::unit::Current>;
using VoltageRotor = unimoc::system::Rotor<unimoc::unit::Voltage>;

TEST(DeadTimeCompensationTest, ReconstructsAndTransformsPhaseSigns) {
  unimoc::settings::NvmSettings settings;
  settings.dtc_dead_time = unimoc::unit::Time{2.0e-6F};
  settings.dtc_f_pwm = unimoc::unit::Frequency{10000.0F};
  settings.dtc_i_threshold = unimoc::unit::Current{1.0F};

  unimoc::control::DeadTimeCompensation<float> compensation;
  compensation.init(settings);

  const auto result = compensation.calculate(CurrentStator{1.0F, 0.0F});

  EXPECT_NEAR(result.alpha.Value(), 0.02F, 1.0e-6F);
  EXPECT_NEAR(result.beta.Value(), 0.0F, 1.0e-6F);
}

TEST(DeadTimeCompensationTest, SoftSignInterpolatesAndSaturates) {
  unimoc::control::DeadTimeCompensation<float> compensation;
  compensation.dead_time = unimoc::unit::Time{2.0e-6F};
  compensation.f_pwm = unimoc::unit::Frequency{10000.0F};
  compensation.i_threshold = unimoc::unit::Current{1.0F};

  const auto interpolated = compensation.calculate(CurrentStator{0.5F, 0.0F});
  const auto saturated = compensation.calculate(CurrentStator{2.0F, 0.0F});

  EXPECT_NEAR(interpolated.alpha.Value(), 0.01F, 1.0e-6F);
  EXPECT_NEAR(saturated.alpha.Value(), 0.0266667F, 1.0e-6F);
}

TEST(MtpaTest, ZeroSaliencyReturnsZero) {
  unimoc::control::Mtpa<float> mtpa;
  mtpa.flux_pm = unimoc::unit::MagneticFlux{0.1F};
  mtpa.L_d = unimoc::unit::Inductance{1.0e-3F};
  mtpa.L_q = unimoc::unit::Inductance{1.0e-3F};

  EXPECT_FLOAT_EQ(mtpa.calculate(unimoc::unit::Current{2.0F}).Value(), 0.0F);
}

TEST(MtpaTest, CalculatesAndLimitsDaxisCurrent) {
  unimoc::control::Mtpa<float> mtpa;
  mtpa.flux_pm = unimoc::unit::MagneticFlux{0.0F};
  mtpa.L_d = unimoc::unit::Inductance{2.0e-3F};
  mtpa.L_q = unimoc::unit::Inductance{1.0e-3F};
  EXPECT_FLOAT_EQ(mtpa.calculate(unimoc::unit::Current{2.0F}).Value(), -1.0F);

  mtpa.flux_pm = unimoc::unit::MagneticFlux{0.01F};
  mtpa.L_d = unimoc::unit::Inductance{1.0e-3F};
  mtpa.L_q = unimoc::unit::Inductance{2.0e-3F};
  EXPECT_FLOAT_EQ(mtpa.calculate(unimoc::unit::Current{1.0F}).Value(), -1.0F);
}

TEST(FieldWeakeningTest, IntegratesOnlyNegativeVoltageHeadroom) {
  unimoc::control::FieldWeakening<float> field_weakening;
  field_weakening.v_max = unimoc::unit::Voltage{0.9F};
  field_weakening.ki = unimoc::unit::CurrentPerVoltageTime{10.0F};
  field_weakening.i_d_min = unimoc::unit::Current{-0.5F};

  EXPECT_FLOAT_EQ(field_weakening.update(unimoc::system::Stator<unimoc::unit::Voltage>{0.5F, 0.0F}, unimoc::unit::Time{0.1F}).Value(), 0.0F);
  EXPECT_NEAR(field_weakening.update(unimoc::system::Stator<unimoc::unit::Voltage>{1.0F, 0.0F}, unimoc::unit::Time{0.1F}).Value(), -0.1F, 1.0e-6F);
  EXPECT_FLOAT_EQ(field_weakening.update(unimoc::system::Stator<unimoc::unit::Voltage>{10.0F, 0.0F}, unimoc::unit::Time{1.0F}).Value(), -0.5F);

  field_weakening.reset();
  EXPECT_FLOAT_EQ(field_weakening.i_d_fw.Value(), 0.0F);
}

TEST(CurrentControllerTest, UnsaturatedPiStepUpdatesIntegrator) {
  unimoc::control::CurrentController<float> controller;
  controller.kp_d = unimoc::unit::VoltagePerCurrent{2.0F};
  controller.ki_d = unimoc::unit::VoltagePerCurrentTime{10.0F};
  controller.v_max = unimoc::unit::DimensionlessRatio{1.0F};

  const VoltageRotor output = controller.update(CurrentRotor{1.0F, 0.0F},
                                                CurrentRotor{0.0F, 0.0F},
                                                unimoc::unit::AngularVelocity{0.0F},
                                                unimoc::unit::Time{0.1F},
                                                unimoc::unit::Voltage{10.0F});

  EXPECT_FLOAT_EQ(output.d.Value(), 2.0F);
  EXPECT_FLOAT_EQ(output.q.Value(), 0.0F);
  EXPECT_FLOAT_EQ(controller.integrator_d.Value(), 1.0F);
}

TEST(CurrentControllerTest, CircularLimitScalesBothAxes) {
  unimoc::control::CurrentController<float> controller;
  controller.kp_d = unimoc::unit::VoltagePerCurrent{2.0F};
  controller.kp_q = unimoc::unit::VoltagePerCurrent{2.0F};
  controller.ki_d = unimoc::unit::VoltagePerCurrentTime{0.0F};
  controller.ki_q = unimoc::unit::VoltagePerCurrentTime{0.0F};
  controller.kb_d = unimoc::unit::InverseTime{0.0F};
  controller.kb_q = unimoc::unit::InverseTime{0.0F};
  controller.v_max = unimoc::unit::DimensionlessRatio{0.1F};

  const VoltageRotor output = controller.update(CurrentRotor{1.0F, 1.0F},
                                                CurrentRotor{0.0F, 0.0F},
                                                unimoc::unit::AngularVelocity{0.0F},
                                                unimoc::unit::Time{0.1F},
                                                unimoc::unit::Voltage{10.0F});
  const float magnitude = std::sqrt(output.d.Value() * output.d.Value() + output.q.Value() * output.q.Value());

  EXPECT_NEAR(magnitude, 1.0F, 1.0e-6F);
  EXPECT_NEAR(output.d.Value(), output.q.Value(), 1.0e-6F);
}