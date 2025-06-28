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

#include "pulse_width.hpp"
#include <modm/platform.hpp>

using namespace modm::platform;
using namespace std::chrono_literals;


namespace unimoc::hardware::pulse_width
{

void initialize()
{
    // Initialize GPIO pins for PWM output
    // Phase A: GPIO B6, A7
    GpioB6::setAlternateFunction(5); // Set GPIO B6 to alternate function mode (AF5 for TIM8)
    GpioA7::setAlternateFunction(4); // Set GPIO A7 to alternate function mode (AF4 for TIM8)
    GpioB6::setOutput(Gpio::OutputType::PushPull); // Set GPIO B6 to push-pull output type
	GpioA7::setOutput(Gpio::OutputType::PushPull);  // Set GPIO A7 to push-pull output type
    // Set GPIO speed to high for better PWM performance
    GpioB6::reset(); // Reset GPIO B6 to low state
    GpioA7::reset(); // Reset GPIO A7 to low state

	// Phase B: GPIO B8, B0
    GpioB8::setAlternateFunction(10); // Set GPIO B8 to alternate function mode (AF10 for TIM8)
	GpioB0::setAlternateFunction(4);  // Set GPIO B0 to alternate function mode (AF4 for TIM8)
	GpioB8::setOutput(Gpio::OutputType::PushPull);     // Set GPIO B8 to push-pull output type
	GpioB0::setOutput(Gpio::OutputType::PushPull);     // Set GPIO B0 to push-pull output type
    GpioB8::reset(); // Reset GPIO B8 to low state
    GpioB0::reset(); // Reset GPIO B0 to low state

	// Phase C: GPIO B9, B5
	GpioB9::setAlternateFunction(10); // Set GPIO B9 to alternate function mode (AF10 for TIM8)
    GpioB5::setAlternateFunction(3); // Set GPIO B5 to alternate function mode (AF3 for TIM8)
	GpioB9::setOutput(Gpio::OutputType::PushPull);     // Set GPIO B9 to push-pull output type
	GpioB5::setOutput(Gpio::OutputType::PushPull);     // Set GPIO B5 to push-pull output type
    GpioB9::reset(); // Reset GPIO B9 to low state
    GpioB5::reset(); // Reset GPIO B5 to low state

	Timer8::connect<GpioOutputB6::Ch1, GpioOutputA7::Ch1n, GpioOutputB8::Ch2, GpioOutputB0::Ch2n, GpioOutputB9::Ch3,
					GpioOutputB5::Ch3n>();
    Timer8::enable(); // Enable Timer 8 for PWM operation

	Timer8::setMode(Timer8::Mode::CenterAligned3, Timer8::SlaveMode::Disabled,
					Timer8::SlaveModeTrigger::Internal0, Timer8::MasterMode::CompareOc4Ref, false,
					Timer8::MasterMode2::Update);

    Timer8::setPrescaler(1); // Set prescaler to 1 for maximum frequency
    Timer8::setPeriod<SystemClock>(62.5us, true); // Set period to 62.5 microseconds (16 kHz PWM frequency)

	Timer8::configureOutputChannel<GpioOutputB6::Ch1>(
		Timer8::OutputCompareMode::Pwm, Timer8::PinState::Enable,
		Timer8::OutputComparePolarity::ActiveHigh, Timer8::PinState::Enable,
		Timer8::OutputComparePolarity::ActiveHigh, Timer8::OutputComparePreload::Enable);
    
    Timer8::configureOutputChannel<GpioOutputB8::Ch2>(
        Timer8::OutputCompareMode::Pwm, Timer8::PinState::Enable,
        Timer8::OutputComparePolarity::ActiveHigh, Timer8::PinState::Enable,
        Timer8::OutputComparePolarity::ActiveHigh, Timer8::OutputComparePreload::Enable);
    
    Timer8::configureOutputChannel<GpioOutputB9::Ch3>(
        Timer8::OutputCompareMode::Pwm, Timer8::PinState::Enable,
        Timer8::OutputComparePolarity::ActiveHigh, Timer8::PinState::Enable,
        Timer8::OutputComparePolarity::ActiveHigh, Timer8::OutputComparePreload::Enable);
}


//! \brief Converts a duty cycle (0.0 to 1.0) to a compare value based on the period.
constexpr uint16_t dutyCycleToCompareValue(float dutyCycle, uint16_t period) noexcept
{
    // Convert duty cycle (0.0 to 1.0) to compare value based on the period
    return static_cast<uint16_t>(dutyCycle * period);
}

//! \brief Sets the PWM duty cycles for the three phases (A, B, C).
void setPhaseDuties(const system::ThreePhase& duties) noexcept
{
	Timer8::setCompareValue<GpioOutputB6::Ch1>(dutyCycleToCompareValue(duties.a, Timer8::getOverflow()));
    Timer8::setCompareValue<GpioOutputB8::Ch2>(dutyCycleToCompareValue(duties.b, Timer8::getOverflow()));
    Timer8::setCompareValue<GpioOutputB9::Ch3>(dutyCycleToCompareValue(duties.c, Timer8::getOverflow()));
}
} // namespace unimoc::hardware::pulse_width


