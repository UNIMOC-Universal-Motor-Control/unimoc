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


#include "SinCos.hpp"

/**
 * @namespace unimoc global namespace
 */
namespace unimoc
{
/**
 * @namespace coordinate systems.
 */
namespace system
{
// Size of the sine table
static constexpr size_t TABLE_SIZE = 512;

// Precomputed sine values for 512 points in the range [0, 2*pi]
// +1 one to include the value for 2*pi
static constexpr std::array<float, TABLE_SIZE + 1> sin_table = [] {
	std::array<float, TABLE_SIZE + 1> table = {};
	for (size_t i = 0; i <= TABLE_SIZE; ++i)
	{
		table[i] = std::sin((2 * std::numbers::pi_v<float> * i) / TABLE_SIZE);
	}
	return table;
}();

// constructor with angle
constexpr SinCos::SinCos(const unit::Angle angle) noexcept
    : sin(sin_table[static_cast<size_t>(angle.value() * TABLE_SIZE / (2 * std::numbers::pi_v<float>)) % TABLE_SIZE]),
      cos(sin_table[(static_cast<size_t>(angle.value() * TABLE_SIZE / (2 * std::numbers::pi_v<float>)) + TABLE_SIZE / 4) % TABLE_SIZE])
{
	from_angle(angle);
}

// length of the vector
constexpr float
SinCos::length() const noexcept
{
	return std::hypot(sin, cos);
}

// normalize the sin and cos values to one
constexpr SinCos
SinCos::normToOne(void) noexcept
{
	sin /= length();
	cos /= length();
}

// default constructor
constexpr SinCos::SinCos() noexcept : sin(0.0f), cos(1.0f) {}

// copy constructor
constexpr SinCos::SinCos(const SinCos &other) noexcept : sin(other.sin), cos(other.cos) {}
// move constructor
constexpr SinCos::SinCos(SinCos &&other) noexcept : sin(other.sin), cos(other.cos) {}

// copy assignment operator
constexpr SinCos &
SinCos::operator=(const SinCos &other)
{
	if (this != &other)
	{
		sin = other.sin;
		cos = other.cos;
	}
	return *this;
}

// move assignment operator
constexpr SinCos &
SinCos::operator=(SinCos &&other) noexcept
{
	if (this != &other)
	{
		sin = other.sin;
		cos = other.cos;
	}
	return *this;
}

// sine cosine difference
constexpr SinCos SinCos::operator - (const SinCos &other) const noexcept
{
	// Using trigonometric identities for subtraction:
	// sin(a - b) = sin(a)cos(b) - cos(a)sin(b)
	// cos(a - b) = cos(a)cos(b) + sin(a)sin(b)
	return SinCos(sin * other.cos - cos * other.sin, cos * other.cos + sin * other.sin);
}

/**
 * @brief Computes the sine and cosine values for the given angle in radians.
 * @param angle The angle in radians.
 */
constexpr void
SinCos::from_angle(const unit::Angle angle) noexcept
{
    // Precomputed constants
    static constexpr float invTwoPi = 1.0f / (2.0f * std::numbers::pi_v<float>);
    static constexpr float delta    = (2.0f * std::numbers::pi_v<float>) / TABLE_SIZE;

    float in      = std::fabsf(angle.value() * invTwoPi);
    in           -= static_cast<int32_t>(in);
    const float fIndex = static_cast<float>(TABLE_SIZE) * in;
    const uint16_t idxS = static_cast<uint16_t>(static_cast<int>(fIndex) & 0x1FF);
    const uint16_t idxC = static_cast<uint16_t>((idxS + (TABLE_SIZE / 4)) & 0x1FF);

    const float fract = fIndex - static_cast<float>(idxS);

    // Cosine
    {
        const float f1 = sin_table[idxC];
        const float f2 = sin_table[idxC + 1];
        const float d1 = -sin_table[idxS];
        const float d2 = -sin_table[idxS + 1];
        const float Df = f2 - f1;
        float tmp = delta * (d1 + d2) - 2.0f * Df;
        tmp = fract * tmp + (3.0f * Df - (d2 + 2.0f * d1) * delta);
        tmp = fract * tmp + d1 * delta;
        this->cos = fract * tmp + f1;
    }

    // Sine
    {
        const float f1 = sin_table[idxS];
        const float f2 = sin_table[idxS + 1];
        const float d1 = sin_table[idxC];
        const float d2 = sin_table[idxC + 1];
        const float Df = f2 - f1;
        float tmp = delta * (d1 + d2) - 2.0f * Df;
        tmp = fract * tmp + (3.0f * Df - (d2 + 2.0f * d1) * delta);
        tmp = fract * tmp + d1 * delta;
        this->sin = fract * tmp + f1;
    }

    if (angle.value() < 0.0f) [[likely]]
    {
        this->sin = -this->sin;
    }
}
}  // namespace system
}  // namespace unimoc
