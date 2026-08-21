/*
 *	   __  ___   ________  _______  ______
 *	  / / / / | / /  _/  |/  / __ \/ ____/
 *	 / / / /  |/ // // /|_/ / / / / /
 *	/ /_/ / /|  // // /  / / /_/ / /___
 *	\____/_/ |_/___/_/  /_/\____/\____/
 *
 *	@file rotor_angle_test.cpp
 *	@brief Unit tests for the fixed-point electrical rotor angle.
 *
 *	This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *	See the repository LICENSE file for details.
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <numbers>

#include "rotor_angle.hpp"
#include "units.hpp"

using unimoc::system::RotorAngle;
using unimoc::unit::Angle;

namespace
{
constexpr float kPi = std::numbers::pi_v<float>;
constexpr float kTwoPi = 2.0F * kPi;
constexpr std::int64_t kCountsPerRevolution = 0x1'0000'0000LL;

/// Sine and cosine tolerance of the 512 entry cubic Hermite table.
constexpr float kTrigTolerance = 1.0e-5F;
}  // namespace

// Compile-time usability of the whole interface.
static_assert(RotorAngle{}.Raw() == 0);
static_assert(RotorAngle{}.Revolutions() == 0);
static_assert(RotorAngle{}.Cos().Value() == 1.0F);
static_assert(RotorAngle::FromRaw(0x4000'0000).Raw() == 0x4000'0000);
static_assert(RotorAngle::FromAngle(Angle{kTwoPi + 0.1F}).Revolutions() == 1);
static_assert(RotorAngle{} == RotorAngle::FromRaw(0));

class RotorAngleTest : public ::testing::Test
{
protected:
	void
	SetUp() override
	{
	}

	void
	TearDown() override
	{
	}
};

TEST_F(RotorAngleTest, DefaultConstructorIsZero)
{
	const RotorAngle angle;
	EXPECT_EQ(angle.Raw(), 0);
	EXPECT_EQ(angle.Revolutions(), 0);
	EXPECT_EQ(angle.AbsoluteRaw(), 0);
	EXPECT_FLOAT_EQ(angle.AngleInRevolution().Value(), 0.0F);
	EXPECT_NEAR(angle.Sin().Value(), 0.0F, kTrigTolerance);
	EXPECT_NEAR(angle.Cos().Value(), 1.0F, kTrigTolerance);
}

TEST_F(RotorAngleTest, RawScaleMapsFullRangeToOneRevolution)
{
	EXPECT_NEAR(RotorAngle::FromRaw(INT32_MIN).AngleInRevolution().Value(), -kPi, 1.0e-6F);
	EXPECT_NEAR(RotorAngle::FromRaw(-0x4000'0000).AngleInRevolution().Value(), -kPi / 2.0F,
				1.0e-6F);
	EXPECT_NEAR(RotorAngle::FromRaw(0x4000'0000).AngleInRevolution().Value(), kPi / 2.0F,
				1.0e-6F);
	EXPECT_NEAR(RotorAngle::FromRaw(INT32_MAX).AngleInRevolution().Value(), kPi, 1.0e-6F);
}

TEST_F(RotorAngleTest, FromAngleRoundTripsWithinResolution)
{
	for (int i = -180; i < 180; ++i)
	{
		const float expected = (static_cast<float>(i) * kPi) / 180.0F;
		const RotorAngle angle = RotorAngle::FromAngle(Angle{expected});
		EXPECT_EQ(angle.Revolutions(), 0);
		EXPECT_NEAR(angle.AngleInRevolution().Value(), expected, 1.0e-6F);
	}
}

TEST_F(RotorAngleTest, SineAndCosineMatchStandardLibrary)
{
	constexpr int kSamples = 4096;
	for (int i = 0; i < kSamples; ++i)
	{
		const float expected =
			-kPi + ((kTwoPi * static_cast<float>(i)) / static_cast<float>(kSamples));
		const RotorAngle angle = RotorAngle::FromAngle(Angle{expected});
		EXPECT_NEAR(angle.Sin().Value(), std::sin(expected), kTrigTolerance) << "i=" << i;
		EXPECT_NEAR(angle.Cos().Value(), std::cos(expected), kTrigTolerance) << "i=" << i;
	}
}

TEST_F(RotorAngleTest, SineAndCosineFollowRawWrap)
{
	// The seam at +-pi must not introduce a sign flip.
	const RotorAngle below = RotorAngle::FromRaw(INT32_MAX);
	const RotorAngle above = RotorAngle::FromRaw(INT32_MIN);
	EXPECT_NEAR(below.Sin().Value(), above.Sin().Value(), 1.0e-5F);
	EXPECT_NEAR(below.Cos().Value(), above.Cos().Value(), 1.0e-5F);
	EXPECT_NEAR(above.Cos().Value(), -1.0F, kTrigTolerance);
}

TEST_F(RotorAngleTest, AdvanceCountsPositiveWrap)
{
	RotorAngle angle = RotorAngle::FromAngle(Angle{kPi - 0.1F});
	ASSERT_EQ(angle.Revolutions(), 0);

	angle.Advance(Angle{0.2F});
	EXPECT_EQ(angle.Revolutions(), 1);
	EXPECT_NEAR(angle.AngleInRevolution().Value(), -kPi + 0.1F, 1.0e-5F);
	EXPECT_NEAR(angle.AbsoluteAngle().Value(), kPi + 0.1F, 1.0e-5F);
}

TEST_F(RotorAngleTest, AdvanceCountsNegativeWrap)
{
	RotorAngle angle = RotorAngle::FromAngle(Angle{-kPi + 0.1F});
	ASSERT_EQ(angle.Revolutions(), 0);

	angle.Advance(Angle{-0.2F});
	EXPECT_EQ(angle.Revolutions(), -1);
	EXPECT_NEAR(angle.AngleInRevolution().Value(), kPi - 0.1F, 1.0e-5F);
	EXPECT_NEAR(angle.AbsoluteAngle().Value(), -kPi - 0.1F, 1.0e-5F);
}

TEST_F(RotorAngleTest, AdvanceAcceptsManyRevolutionsAtOnce)
{
	RotorAngle angle;
	angle.Advance(Angle{100.5F * kTwoPi});
	EXPECT_EQ(angle.Revolutions(), 101);
	EXPECT_NEAR(angle.AngleInRevolution().Value(), -kPi, 1.0e-3F);

	angle.Advance(Angle{-100.5F * kTwoPi});
	EXPECT_EQ(angle.Revolutions(), 0);
}

TEST_F(RotorAngleTest, RepeatedAdvanceDoesNotCompoundError)
{
	constexpr int kStepsPerRevolution = 1000;
	constexpr int kRevolutions = 25;
	constexpr int kSteps = kStepsPerRevolution * kRevolutions;
	const Angle step{kTwoPi / static_cast<float>(kStepsPerRevolution)};

	RotorAngle angle;
	for (int i = 0; i < kSteps; ++i)
	{
		angle.Advance(step);
	}

	// Accumulation is integer, so the error is bounded by half a count per call and
	// never compounds; a float phase accumulator would drift far beyond that.
	EXPECT_EQ(angle.Revolutions(), kRevolutions);
	EXPECT_LE(std::abs(static_cast<double>(angle.Raw())), 0.5 * kSteps);
}

TEST_F(RotorAngleTest, AbsoluteRawIsMonotonicAcrossWrap)
{
	RotorAngle angle = RotorAngle::FromAngle(Angle{kPi - 0.05F});
	std::int64_t previous = angle.AbsoluteRaw();
	for (int i = 0; i < 100; ++i)
	{
		angle.Advance(Angle{0.01F});
		EXPECT_GT(angle.AbsoluteRaw(), previous);
		previous = angle.AbsoluteRaw();
	}
	EXPECT_EQ(angle.Revolutions(), 1);
}

TEST_F(RotorAngleTest, AbsoluteRawMatchesAbsoluteAngle)
{
	const RotorAngle angle = RotorAngle::FromAngle(Angle{7.0F});
	const double expected = static_cast<double>(angle.AbsoluteRaw()) *
							(2.0 * std::numbers::pi_v<double> /
							 static_cast<double>(kCountsPerRevolution));
	EXPECT_NEAR(angle.AbsoluteAngle().Value(), static_cast<float>(expected), 1.0e-5F);
}

TEST_F(RotorAngleTest, DifferenceTakesShortestPath)
{
	const RotorAngle ahead = RotorAngle::FromAngle(Angle{kPi - 0.1F});
	const RotorAngle behind = RotorAngle::FromAngle(Angle{-kPi + 0.1F});

	EXPECT_NEAR((behind - ahead).Value(), 0.2F, 1.0e-5F);
	EXPECT_NEAR((ahead - behind).Value(), -0.2F, 1.0e-5F);
}

TEST_F(RotorAngleTest, DifferenceIgnoresRevolutionCounter)
{
	const RotorAngle first = RotorAngle::FromRaw(0x2000'0000, 3);
	const RotorAngle second = RotorAngle::FromRaw(0x2000'0000, -7);
	EXPECT_FLOAT_EQ((first - second).Value(), 0.0F);
}

TEST_F(RotorAngleTest, OffsetOperators)
{
	const RotorAngle base = RotorAngle::FromAngle(Angle{1.0F});

	RotorAngle mutated = base;
	mutated += Angle{0.5F};
	EXPECT_NEAR(mutated.AngleInRevolution().Value(), 1.5F, 1.0e-5F);
	mutated -= Angle{0.5F};
	EXPECT_EQ(mutated, base);

	EXPECT_NEAR((base + Angle{0.25F}).AngleInRevolution().Value(), 1.25F, 1.0e-5F);
	EXPECT_NEAR((base - Angle{0.25F}).AngleInRevolution().Value(), 0.75F, 1.0e-5F);
	EXPECT_EQ(base, RotorAngle::FromAngle(Angle{1.0F}));
}

TEST_F(RotorAngleTest, EqualityComparesPhaseAndRevolutions)
{
	EXPECT_EQ(RotorAngle::FromRaw(1234, 5), RotorAngle::FromRaw(1234, 5));
	EXPECT_NE(RotorAngle::FromRaw(1234, 5), RotorAngle::FromRaw(1234, 6));
	EXPECT_NE(RotorAngle::FromRaw(1234, 5), RotorAngle::FromRaw(1235, 5));
}
