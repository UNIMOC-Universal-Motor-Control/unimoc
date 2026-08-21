/*
	   __  ___   ________  _______  ______
	  / / / / | / /  _/  |/  / __ \/ ____/
	 / / / /  |/ // // /|_/ / / / / /
	/ /_/ / /|  // // /  / / /_/ / /___
	\____/_/ |_/___/_/  /_/\____/\____/

	Universal Motor Control  2025 Alexander <tecnologic86@gmail.com> Evers

	This file is part of UNIMOC.

	UNIMOC is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <gtest/gtest.h>
#include <cmath>
#include <concepts>

#include "SinCos.hpp"
#include "rotor_angle.hpp"
#include "rotor_system.hpp"
#include "three_phase_system.hpp"

using ThreePhase = unimoc::system::ThreePhase<unimoc::unit::DimensionlessRatio>;
using Stator = unimoc::system::Stator<float>;
using Rotor = unimoc::system::RotorReference<float>;
using SinCos = unimoc::system::SinCos<float>;

static_assert(unimoc::unit::UnitLike<unimoc::unit::Current>);
static_assert(!unimoc::unit::UnitLike<float>);

template <typename T>
concept CanInstantiateThreePhase = requires { typename unimoc::system::ThreePhase<T>; };

static_assert(!CanInstantiateThreePhase<float>);
static_assert(std::same_as<decltype(unimoc::system::ThreePhase<unimoc::unit::Current>{}.ToStator()),
						  Stator>);

// Test fixture for ThreePhase tests
class ThreePhaseTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		// Setup code if needed
	}

	void TearDown() override
	{
		// Cleanup code if needed
	}
};

// Test default constructor
TEST_F(ThreePhaseTest, DefaultConstructor)
{
	ThreePhase phase;
	// Default constructor should initialize values (implementation dependent)
	// Just verify object is created without throwing
	EXPECT_NO_THROW(ThreePhase());

	(void)phase; // Suppress unused variable warning
}

// Test parameterized constructor
TEST_F(ThreePhaseTest, ParameterizedConstructor)
{
	ThreePhase phase(1.5f, 2.5f, 3.5f);

	EXPECT_FLOAT_EQ(phase.a.Value(), 1.5f);
	EXPECT_FLOAT_EQ(phase.b.Value(), 2.5f);
	EXPECT_FLOAT_EQ(phase.c.Value(), 3.5f);
}

// Test parameterized constructor with zero values
TEST_F(ThreePhaseTest, ConstructorWithZeros)
{
	ThreePhase phase(0.0f, 0.0f, 0.0f);

	EXPECT_FLOAT_EQ(phase.a.Value(), 0.0f);
	EXPECT_FLOAT_EQ(phase.b.Value(), 0.0f);
	EXPECT_FLOAT_EQ(phase.c.Value(), 0.0f);
}

// Test parameterized constructor with negative values
TEST_F(ThreePhaseTest, ConstructorWithNegativeValues)
{
	ThreePhase phase(-1.0f, -2.0f, -3.0f);

	EXPECT_FLOAT_EQ(phase.a.Value(), -1.0f);
	EXPECT_FLOAT_EQ(phase.b.Value(), -2.0f);
	EXPECT_FLOAT_EQ(phase.c.Value(), -3.0f);
}

// Test copy constructor
TEST_F(ThreePhaseTest, CopyConstructor)
{
	ThreePhase original(1.0f, 2.0f, 3.0f);
	ThreePhase copy(original);

	EXPECT_FLOAT_EQ(copy.a.Value(), 1.0f);
	EXPECT_FLOAT_EQ(copy.b.Value(), 2.0f);
	EXPECT_FLOAT_EQ(copy.c.Value(), 3.0f);

	// Verify independence
	copy.a = unimoc::unit::DimensionlessRatio{10.0f};
	EXPECT_FLOAT_EQ(original.a.Value(), 1.0f);
}

// Test move constructor
TEST_F(ThreePhaseTest, MoveConstructor)
{
	ThreePhase original(1.0f, 2.0f, 3.0f);
	ThreePhase moved(std::move(original));

	EXPECT_FLOAT_EQ(moved.a.Value(), 1.0f);
	EXPECT_FLOAT_EQ(moved.b.Value(), 2.0f);
	EXPECT_FLOAT_EQ(moved.c.Value(), 3.0f);
}

// Test copy assignment operator
TEST_F(ThreePhaseTest, CopyAssignmentOperator)
{
	ThreePhase original(1.0f, 2.0f, 3.0f);
	ThreePhase copy(4.0f, 5.0f, 6.0f);

	copy = original;

	EXPECT_FLOAT_EQ(copy.a.Value(), 1.0f);
	EXPECT_FLOAT_EQ(copy.b.Value(), 2.0f);
	EXPECT_FLOAT_EQ(copy.c.Value(), 3.0f);
}

// Test move assignment operator
TEST_F(ThreePhaseTest, MoveAssignmentOperator)
{
	ThreePhase original(1.0f, 2.0f, 3.0f);
	ThreePhase moved(4.0f, 5.0f, 6.0f);

	moved = std::move(original);

	EXPECT_FLOAT_EQ(moved.a.Value(), 1.0f);
	EXPECT_FLOAT_EQ(moved.b.Value(), 2.0f);
	EXPECT_FLOAT_EQ(moved.c.Value(), 3.0f);
}

// Test equality operator with equal values
TEST_F(ThreePhaseTest, EqualityOperatorTrue)
{
	ThreePhase phase1(1.0f, 2.0f, 3.0f);
	ThreePhase phase2(1.0f, 2.0f, 3.0f);

	EXPECT_TRUE(phase1 == phase2);
	EXPECT_TRUE(phase2 == phase1);
}

// Test equality operator with different values
TEST_F(ThreePhaseTest, EqualityOperatorFalse)
{
	ThreePhase phase1(1.0f, 2.0f, 3.0f);
	ThreePhase phase2(1.0f, 2.0f, 4.0f);

	EXPECT_FALSE(phase1 == phase2);
}

// Test equality operator with self
TEST_F(ThreePhaseTest, EqualityOperatorSelf)
{
	ThreePhase phase(1.0f, 2.0f, 3.0f);

	EXPECT_TRUE(phase == phase);
}

// Test inequality operator
TEST_F(ThreePhaseTest, InequalityOperatorTrue)
{
	ThreePhase phase1(1.0f, 2.0f, 3.0f);
	ThreePhase phase2(4.0f, 5.0f, 6.0f);

	EXPECT_TRUE(phase1 != phase2);
	EXPECT_TRUE(phase2 != phase1);
}

// Test inequality operator with equal values
TEST_F(ThreePhaseTest, InequalityOperatorFalse)
{
	ThreePhase phase1(1.0f, 2.0f, 3.0f);
	ThreePhase phase2(1.0f, 2.0f, 3.0f);

	EXPECT_FALSE(phase1 != phase2);
}

// Test addition operator
TEST_F(ThreePhaseTest, AdditionOperator)
{
	ThreePhase phase1(1.0f, 2.0f, 3.0f);
	ThreePhase phase2(4.0f, 5.0f, 6.0f);

	ThreePhase result = phase1 + phase2;

	EXPECT_FLOAT_EQ(result.a.Value(), 5.0f);
	EXPECT_FLOAT_EQ(result.b.Value(), 7.0f);
	EXPECT_FLOAT_EQ(result.c.Value(), 9.0f);
}

// Test addition with zero
TEST_F(ThreePhaseTest, AdditionWithZero)
{
	ThreePhase phase1(1.0f, 2.0f, 3.0f);
	ThreePhase zero(0.0f, 0.0f, 0.0f);

	ThreePhase result = phase1 + zero;

	EXPECT_FLOAT_EQ(result.a.Value(), 1.0f);
	EXPECT_FLOAT_EQ(result.b.Value(), 2.0f);
	EXPECT_FLOAT_EQ(result.c.Value(), 3.0f);
}

// Test addition with negative values
TEST_F(ThreePhaseTest, AdditionWithNegativeValues)
{
	ThreePhase phase1(1.0f, 2.0f, 3.0f);
	ThreePhase phase2(-1.0f, -2.0f, -3.0f);

	ThreePhase result = phase1 + phase2;

	EXPECT_FLOAT_EQ(result.a.Value(), 0.0f);
	EXPECT_FLOAT_EQ(result.b.Value(), 0.0f);
	EXPECT_FLOAT_EQ(result.c.Value(), 0.0f);
}

// Test addition commutativity
TEST_F(ThreePhaseTest, AdditionCommutativity)
{
	ThreePhase phase1(1.0f, 2.0f, 3.0f);
	ThreePhase phase2(4.0f, 5.0f, 6.0f);

	ThreePhase result1 = phase1 + phase2;
	ThreePhase result2 = phase2 + phase1;

	EXPECT_TRUE(result1 == result2);
}

// Test subtraction operator
TEST_F(ThreePhaseTest, SubtractionOperator)
{
	ThreePhase phase1(5.0f, 7.0f, 9.0f);
	ThreePhase phase2(1.0f, 2.0f, 3.0f);

	ThreePhase result = phase1 - phase2;

	EXPECT_FLOAT_EQ(result.a.Value(), 4.0f);
	EXPECT_FLOAT_EQ(result.b.Value(), 5.0f);
	EXPECT_FLOAT_EQ(result.c.Value(), 6.0f);
}

// Test subtraction with zero
TEST_F(ThreePhaseTest, SubtractionWithZero)
{
	ThreePhase phase1(1.0f, 2.0f, 3.0f);
	ThreePhase zero(0.0f, 0.0f, 0.0f);

	ThreePhase result = phase1 - zero;

	EXPECT_FLOAT_EQ(result.a.Value(), 1.0f);
	EXPECT_FLOAT_EQ(result.b.Value(), 2.0f);
	EXPECT_FLOAT_EQ(result.c.Value(), 3.0f);
}

// Test subtraction with itself
TEST_F(ThreePhaseTest, SubtractionWithSelf)
{
	ThreePhase phase(1.0f, 2.0f, 3.0f);

	ThreePhase result = phase - phase;

	EXPECT_FLOAT_EQ(result.a.Value(), 0.0f);
	EXPECT_FLOAT_EQ(result.b.Value(), 0.0f);
	EXPECT_FLOAT_EQ(result.c.Value(), 0.0f);
}

// Test subtraction with negative values
TEST_F(ThreePhaseTest, SubtractionWithNegativeValues)
{
	ThreePhase phase1(1.0f, 2.0f, 3.0f);
	ThreePhase phase2(-1.0f, -2.0f, -3.0f);

	ThreePhase result = phase1 - phase2;

	EXPECT_FLOAT_EQ(result.a.Value(), 2.0f);
	EXPECT_FLOAT_EQ(result.b.Value(), 4.0f);
	EXPECT_FLOAT_EQ(result.c.Value(), 6.0f);
}

// Test ToArray conversion
TEST_F(ThreePhaseTest, ToArrayConversion)
{
	ThreePhase phase(1.5f, 2.5f, 3.5f);

	auto arr = phase.ToArray();

	EXPECT_EQ(arr.size(), 3);
	EXPECT_FLOAT_EQ(arr[0].Value(), 1.5f);
	EXPECT_FLOAT_EQ(arr[1].Value(), 2.5f);
	EXPECT_FLOAT_EQ(arr[2].Value(), 3.5f);
}

// Test ToArray with zero values
TEST_F(ThreePhaseTest, ToArrayWithZeros)
{
	ThreePhase phase(0.0f, 0.0f, 0.0f);

	auto arr = phase.ToArray();

	EXPECT_FLOAT_EQ(arr[0].Value(), 0.0f);
	EXPECT_FLOAT_EQ(arr[1].Value(), 0.0f);
	EXPECT_FLOAT_EQ(arr[2].Value(), 0.0f);
}

// Test ToArray with negative values
TEST_F(ThreePhaseTest, ToArrayWithNegativeValues)
{
	ThreePhase phase(-1.0f, -2.0f, -3.0f);

	auto arr = phase.ToArray();

	EXPECT_FLOAT_EQ(arr[0].Value(), -1.0f);
	EXPECT_FLOAT_EQ(arr[1].Value(), -2.0f);
	EXPECT_FLOAT_EQ(arr[2].Value(), -3.0f);
}

TEST_F(ThreePhaseTest, ToStatorUsesUnitRepresentation)
{
	const unimoc::system::ThreePhase<unimoc::unit::Current> phase{1.0f, 2.0f, 3.0f};

	const auto stator = phase.ToStator();

	EXPECT_FLOAT_EQ(stator.alpha, -1.0f);
	EXPECT_NEAR(stator.beta, -0.577350269f, 1.0e-6f);
}

TEST_F(ThreePhaseTest, ToStatorRejectsZeroSequence)
{
	const unimoc::system::ThreePhase<unimoc::unit::Current> phase{2.0f, 2.0f, 2.0f};

	const auto stator = phase.ToStator();

	EXPECT_FLOAT_EQ(stator.alpha, 0.0f);
	EXPECT_FLOAT_EQ(stator.beta, 0.0f);
}

TEST_F(ThreePhaseTest, ToStatorTransformsBalancedPhaseVector)
{
	const unimoc::system::ThreePhase<unimoc::unit::Current> phase{1.0f, -0.5f, -0.5f};

	const auto stator = phase.ToStator();

	EXPECT_FLOAT_EQ(stator.alpha, 1.0f);
	EXPECT_NEAR(stator.beta, 0.0f, 1.0e-6f);
}

TEST_F(ThreePhaseTest, StatorToRotorAtZeroAngle)
{
	const Stator stator{1.0f, 2.0f};
	const SinCos angle{0.0f, 1.0f};

	const auto rotor = stator.ToRotor(angle);

	EXPECT_FLOAT_EQ(rotor.d, 1.0f);
	EXPECT_FLOAT_EQ(rotor.q, 2.0f);
}

TEST_F(ThreePhaseTest, StatorToRotorAtQuarterTurn)
{
	const Stator stator{1.0f, 2.0f};
	const SinCos angle{1.0f, 0.0f};

	const auto rotor = stator.ToRotor(angle);

	EXPECT_FLOAT_EQ(rotor.d, 2.0f);
	EXPECT_FLOAT_EQ(rotor.q, -1.0f);
}

TEST_F(ThreePhaseTest, RotorToStatorAtZeroAngle)
{
	const Rotor rotor{1.0f, 2.0f};
	const SinCos angle{0.0f, 1.0f};

	const auto stator = rotor.inverse_park(angle);

	EXPECT_FLOAT_EQ(stator.alpha, 1.0f);
	EXPECT_FLOAT_EQ(stator.beta, 2.0f);
}

TEST_F(ThreePhaseTest, RotorToStatorAtQuarterTurn)
{
	const Rotor rotor{1.0f, 2.0f};
	const SinCos angle{1.0f, 0.0f};

	const auto stator = rotor.inverse_park(angle);

	EXPECT_FLOAT_EQ(stator.alpha, -2.0f);
	EXPECT_FLOAT_EQ(stator.beta, 1.0f);
}

TEST_F(ThreePhaseTest, ParkAndInverseParkRoundTrip)
{
	const Stator original{1.25f, -0.75f};
	const SinCos angle{0.6f, 0.8f};

	const Rotor rotor = original.ToRotor(angle);
	const Stator restored = rotor.inverse_park(angle);

	EXPECT_NEAR(restored.alpha, original.alpha, 1.0e-6f);
	EXPECT_NEAR(restored.beta, original.beta, 1.0e-6f);
}

TEST_F(ThreePhaseTest, ParkTransformAcceptsRotorAngle)
{
	const Stator stator{1.0f, 2.0f};
	const unimoc::system::RotorAngle angle;

	const auto rotor = stator.ToRotor(angle);

	EXPECT_FLOAT_EQ(rotor.d, 1.0f);
	EXPECT_FLOAT_EQ(rotor.q, 2.0f);
}

TEST_F(ThreePhaseTest, InverseParkTransformAcceptsRotorAngle)
{
	const Rotor rotor{1.0f, 2.0f};
	const unimoc::system::RotorAngle angle;

	const auto stator = rotor.inverse_park(angle);

	EXPECT_FLOAT_EQ(stator.alpha, 1.0f);
	EXPECT_FLOAT_EQ(stator.beta, 2.0f);
}

// Test constexpr functionality (compile-time evaluation)
TEST_F(ThreePhaseTest, ConstexprConstructor)
{
	constexpr ThreePhase phase(1.0f, 2.0f, 3.0f);

	EXPECT_FLOAT_EQ(phase.a.Value(), 1.0f);
	EXPECT_FLOAT_EQ(phase.b.Value(), 2.0f);
	EXPECT_FLOAT_EQ(phase.c.Value(), 3.0f);
}

// Test constexpr addition
TEST_F(ThreePhaseTest, ConstexprAddition)
{
	constexpr ThreePhase phase1(1.0f, 2.0f, 3.0f);
	constexpr ThreePhase phase2(4.0f, 5.0f, 6.0f);
	constexpr ThreePhase result = phase1 + phase2;

	EXPECT_FLOAT_EQ(result.a.Value(), 5.0f);
	EXPECT_FLOAT_EQ(result.b.Value(), 7.0f);
	EXPECT_FLOAT_EQ(result.c.Value(), 9.0f);
}

// Test constexpr subtraction
TEST_F(ThreePhaseTest, ConstexprSubtraction)
{
	constexpr ThreePhase phase1(5.0f, 7.0f, 9.0f);
	constexpr ThreePhase phase2(1.0f, 2.0f, 3.0f);
	constexpr ThreePhase result = phase1 - phase2;

	EXPECT_FLOAT_EQ(result.a.Value(), 4.0f);
	EXPECT_FLOAT_EQ(result.b.Value(), 5.0f);
	EXPECT_FLOAT_EQ(result.c.Value(), 6.0f);
}

// Test constexpr equality
TEST_F(ThreePhaseTest, ConstexprEquality)
{
	constexpr ThreePhase phase1(1.0f, 2.0f, 3.0f);
	constexpr ThreePhase phase2(1.0f, 2.0f, 3.0f);
	constexpr bool equal = phase1 == phase2;

	EXPECT_TRUE(equal);
}

// Test with large values
TEST_F(ThreePhaseTest, LargeValues)
{
	ThreePhase phase(1000.0f, 2000.0f, 3000.0f);

	EXPECT_FLOAT_EQ(phase.a.Value(), 1000.0f);
	EXPECT_FLOAT_EQ(phase.b.Value(), 2000.0f);
	EXPECT_FLOAT_EQ(phase.c.Value(), 3000.0f);
}

// Test with very small values
TEST_F(ThreePhaseTest, SmallValues)
{
	ThreePhase phase(0.001f, 0.002f, 0.003f);

	EXPECT_FLOAT_EQ(phase.a.Value(), 0.001f);
	EXPECT_FLOAT_EQ(phase.b.Value(), 0.002f);
	EXPECT_FLOAT_EQ(phase.c.Value(), 0.003f);
}

// Test addition and subtraction chaining
TEST_F(ThreePhaseTest, ArithmeticChaining)
{
	ThreePhase phase1(1.0f, 2.0f, 3.0f);
	ThreePhase phase2(4.0f, 5.0f, 6.0f);
	ThreePhase phase3(7.0f, 8.0f, 9.0f);

	ThreePhase result = phase1 + phase2 - phase3;

	EXPECT_FLOAT_EQ(result.a.Value(), -2.0f);
	EXPECT_FLOAT_EQ(result.b.Value(), -1.0f);
	EXPECT_FLOAT_EQ(result.c.Value(), 0.0f);
}

// Test multiple operations
TEST_F(ThreePhaseTest, MultipleOperations)
{
	ThreePhase phase1(10.0f, 20.0f, 30.0f);
	ThreePhase phase2(5.0f, 10.0f, 15.0f);

	// Add
	ThreePhase sum = phase1 + phase2;
	EXPECT_FLOAT_EQ(sum.a.Value(), 15.0f);
	EXPECT_FLOAT_EQ(sum.b.Value(), 30.0f);
	EXPECT_FLOAT_EQ(sum.c.Value(), 45.0f);

	// Subtract
	ThreePhase diff = phase1 - phase2;
	EXPECT_FLOAT_EQ(diff.a.Value(), 5.0f);
	EXPECT_FLOAT_EQ(diff.b.Value(), 10.0f);
	EXPECT_FLOAT_EQ(diff.c.Value(), 15.0f);

	// Convert to array
	auto arr = sum.ToArray();
	EXPECT_FLOAT_EQ(arr[0].Value(), 15.0f);
	EXPECT_FLOAT_EQ(arr[1].Value(), 30.0f);
	EXPECT_FLOAT_EQ(arr[2].Value(), 45.0f);
}

// Test floating point precision edge cases
TEST_F(ThreePhaseTest, FloatingPointPrecision)
{
	ThreePhase phase1(0.1f + 0.2f, 0.3f, 0.0f);
	ThreePhase phase2(0.3f, 0.3f, 0.0f);

	// Due to floating point precision, these might not be exactly equal
	// but should be very close
	EXPECT_NEAR(phase1.a.Value(), phase2.a.Value(), 1e-6f);
	EXPECT_FLOAT_EQ(phase1.b.Value(), phase2.b.Value());
}

// Test that operations don't modify original objects
TEST_F(ThreePhaseTest, OperationsImmutability)
{
	ThreePhase phase1(1.0f, 2.0f, 3.0f);
	ThreePhase phase2(4.0f, 5.0f, 6.0f);

	ThreePhase sum = phase1 + phase2;
	ThreePhase diff = phase1 - phase2;

	// Original objects should remain unchanged
	EXPECT_FLOAT_EQ(phase1.a.Value(), 1.0f);
	EXPECT_FLOAT_EQ(phase1.b.Value(), 2.0f);
	EXPECT_FLOAT_EQ(phase1.c.Value(), 3.0f);

	EXPECT_FLOAT_EQ(phase2.a.Value(), 4.0f);
	EXPECT_FLOAT_EQ(phase2.b.Value(), 5.0f);
	EXPECT_FLOAT_EQ(phase2.c.Value(), 6.0f);

	EXPECT_FLOAT_EQ(sum.a.Value(), 5.0f);
	EXPECT_FLOAT_EQ(sum.b.Value(), 7.0f);
	EXPECT_FLOAT_EQ(sum.c.Value(), 9.0f);

	EXPECT_FLOAT_EQ(diff.a.Value(), -3.0f);
	EXPECT_FLOAT_EQ(diff.b.Value(), -3.0f);
	EXPECT_FLOAT_EQ(diff.c.Value(), -3.0f);
}
