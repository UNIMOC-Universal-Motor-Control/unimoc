/*
 * Copyright (c) 2009-2012, Fabian Greif
 * Copyright (c) 2009, 2011-2012, Georgi Grinshpun
 * Copyright (c) 2010, Martin Rosekeit
 * Copyright (c) 2011, 2013-2017, Niklas Hauser
 * Copyright (c) 2013, 2015, 2022, Sascha Schade
 * Copyright (c) 2013, 2016, Kevin Läufer
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_TIMER_8_HPP
#define MODM_STM32_TIMER_8_HPP

#include <chrono>
#include <limits>
#include "advanced_base.hpp"
#include <modm/platform/gpio/connector.hpp>

namespace modm::platform
{

/**
 * @brief		Advanced Control Timer 8
 *
 * 
 * Interrupt handler for High Density:
 * - TIM8_BRK
 * - TIM8_UP
 * - TIM8_TRG_COM
 * - TIM8_CC
 *
 * Interrupt handler for XL Density:
 * - TIM8_BRK_TIM12
 * - TIM8_UP_TIM13
 * - TIM8_TRG_COM_TIM14
 * - TIM8_CC
 * 
 *
 * Example:
 * @code
 * MODM_ISR(TIM8_UP)
 * {
 *     Timer8::resetInterruptFlags(Timer8::...);
 *
 *     ...
 * }
 * @endcode
 *
 * @warning	The Timer has much more possibilities than presented by this
 * 			interface (e.g. Input Capture, Trigger for other Timers, DMA).
 * 			It might be expanded in the future.
 *
 * @author		Fabian Greif
 * @author		Kevin Laeufer
 * @ingroup		modm_platform_timer
 */
class Timer8 : public AdvancedControlTimer
{
public:
	// This type is the internal size of the counter.
	using Value = uint16_t;

	template< class... Signals >
	static void
	connect()
	{
		using Connector = GpioConnector<Peripheral::Tim8, Signals...>;
		Connector::connect();
	}

	static void
	enable();

	static void
	disable();

	static bool
	isEnabled();

	static inline void
	pause()
	{
		TIM8->CR1 &= ~TIM_CR1_CEN;
	}

	static inline void
	start()
	{
		TIM8->CR1 |= TIM_CR1_CEN;
	}

	static void
	setMode(Mode mode,
			SlaveMode slaveMode = SlaveMode::Disabled,
			SlaveModeTrigger slaveModeTrigger = SlaveModeTrigger::Internal0,
			MasterMode masterMode = MasterMode::Reset,
			bool enableOnePulseMode = false
			, MasterMode2 masterMode2 = MasterMode2::Reset
			);

	static inline void
	setCaptureCompareControlUpdate(CaptureCompareControlUpdate update)
	{
		TIM8->CR2 =  (TIM8->CR2 & ~TIM_CR2_CCUS)
							| static_cast<uint32_t>(update);
	};


	/*
	 * Enables Capture/Compare preloaded control
	 *
	 * If enabled CCxE, CCxNE and OCxM bits are preloaded and only
	 * updated when the COMG bit is set or on a rising edge on TRGI
	 * This is determined by the CaptureCompareControlUpdate setting.
	 */
	static inline void
	enableCaptureComparePreloadedControl
	(CaptureCompareControlUpdate update = CaptureCompareControlUpdate::SetComg)
	{
		TIM8->CR2 = (TIM8->CR2 & ~TIM_CR2_CCUS)
							| static_cast<uint32_t>(update) | TIM_CR2_CCPC;
	}

	static inline void
	disableCaptureComparePreloadedControl()
	{
		TIM8->CR2 &= ~TIM_CR2_CCPC;
	}

	static inline void
	setPrescaler(uint16_t prescaler)
	{
		// Because a prescaler of zero is not possible the actual
		// prescaler value is \p prescaler - 1 (see Datasheet)
		TIM8->PSC = prescaler - 1;
	}

	static uint16_t
	getPrescaler()
	{
		return TIM8->PSC + 1;
	}

	static inline void
	setOverflow(Value overflow)
	{
		TIM8->ARR = overflow;
	}

	static inline Value
	getOverflow()
	{
		return TIM8->ARR;
	}

	template<class SystemClock>
	static constexpr uint32_t
	getClockFrequency()
	{
		return SystemClock::Timer8;
	}

	template<class SystemClock, class Rep, class Period>
	static Value
	setPeriod(std::chrono::duration<Rep, Period> duration, bool autoApply = true)
	{
		// This will be inaccurate for non-smooth frequencies (last six digits unequal to zero)
		const uint32_t cycles = duration.count() * SystemClock::Timer8 * Period::num / Period::den;
		const uint16_t prescaler = (cycles + std::numeric_limits<Value>::max() - 1) / std::numeric_limits<Value>::max();	// always round up
		const Value overflow = cycles / prescaler - 1;

		setPrescaler(prescaler);
		setOverflow(overflow);

		// Generate Update Event to apply the new settings for ARR
		if (autoApply) {
			applyAndReset();
		}

		return overflow;
	}

	static inline void
	generateEvent(Event ev)
	{
		TIM8->EGR = static_cast<uint32_t>(ev);
	}

	static inline void
	applyAndReset()
	{
		// Generate Update Event to apply the new settings for ARR
		generateEvent(Event::Update);
	}

	static inline Value
	getValue()
	{
		return TIM8->CNT;
	}

	static inline void
	setValue(Value value)
	{
		TIM8->CNT = value;
	}

	static constexpr bool
	hasAdvancedPwmControl()
	{
		return true;
	}

	static inline bool
	isCountingUp()
	{
		return (TIM8->CR1 & TIM_CR1_DIR) == 0;
	}

	static inline bool
	isCountingDown()
	{
		return !isCountingUp();
	}

	static inline void
	enableOutput()
	{
		TIM8->BDTR |= TIM_BDTR_MOE;
	}

	static inline void
	disableOutput()
	{
		TIM8->BDTR &= ~(TIM_BDTR_MOE);
	}

	static inline bool
	isOutputEnabled()
	{
		return (TIM8->BDTR & TIM_BDTR_MOE);
	}
	/*
	 * Enable/Disable automatic set of MOE bit at the next update event
	 */
	static inline void
	setAutomaticUpdate(bool enable)
	{
		if(enable)
			TIM8->BDTR |= TIM_BDTR_AOE;
		else
			TIM8->BDTR &= ~TIM_BDTR_AOE;
	}

	static inline void
	setOffState(OffStateForRunMode runMode, OffStateForIdleMode idleMode)
	{
		uint32_t flags = TIM8->BDTR;
		flags &= ~(TIM_BDTR_OSSR | TIM_BDTR_OSSI);
		flags |= static_cast<uint32_t>(runMode);
		flags |= static_cast<uint32_t>(idleMode);
		TIM8->BDTR = flags;
	}

	static inline void
	setOutputIdleState(uint32_t channel, OutputIdleState idle,
							OutputIdleState idle_n = OutputIdleState::Reset)
	{
		uint32_t flags = TIM8->CR2;
		channel -= 1;
		flags &=  (static_cast<uint32_t>(OutputIdleState::Set) << (channel * 2))
				| (static_cast<uint32_t>(OutputIdleState::Set) << (channel * 2 + 1));
		flags |= (static_cast<uint32_t>(idle)   << (channel * 2));
		flags |= (static_cast<uint32_t>(idle_n) << (channel * 2 + 1));
		TIM8->CR2 = flags;
	}

	template<typename Signal>
	static void
	setOutputIdleState(OutputIdleState idle, OutputIdleState idle_n = OutputIdleState::Reset)
	{
		constexpr auto channel = signalToChannel<Peripheral::Tim8, Signal>();
		setOutputIdleState(channel, idle, idle_n);
	}

	/*
	 * Set Dead Time Value
	 *
	 * Different Resolution Depending on DeadTime[7:5]:
	 *     0xx =>  DeadTime[6:0]            * T(DTS)
	 *     10x => (DeadTime[5:0] + 32) *  2 * T(DTS)
	 *     110 => (DeadTime[4:0] + 4)  *  8 * T(DTS)
	 *     111 => (DeadTime[4:0] + 2)  * 16 * T(DTS)
	 */
	static inline void
	setDeadTime(uint8_t deadTime)
	{
		uint32_t flags = TIM8->BDTR;
		flags &= ~TIM_BDTR_DTG;
		flags |= deadTime;
		TIM8->BDTR = flags;
	}

	/*
	 * Set Dead Time Value
	 *
	 * Different Resolution Depending on DeadTime[7:5]:
	 *     0xx =>  DeadTime[6:0]            * T(DTS)
	 *     10x => (DeadTime[5:0] + 32) *  2 * T(DTS)
	 *     110 => (DeadTime[4:0] + 4)  *  8 * T(DTS)
	 *     111 => (DeadTime[4:0] + 2)  * 16 * T(DTS)
	 */
	static inline void
	setDeadTime(DeadTimeResolution resolution, uint8_t deadTime)
	{
		uint8_t bitmask;
		switch(resolution){
			case DeadTimeResolution::From0With125nsStep:
				bitmask = 0b01111111;
				break;
			case DeadTimeResolution::From16usWith250nsStep:
				bitmask = 0b00111111;
				break;
			case DeadTimeResolution::From32usWith1usStep:
			case DeadTimeResolution::From64usWith2usStep:
				bitmask = 0b00011111;
				break;
			default:
				bitmask = 0x00;
				break;
		}
		uint32_t flags = TIM8->BDTR;
		flags &= ~TIM_BDTR_DTG;
		flags |= (deadTime & bitmask) | static_cast<uint32_t>(resolution);
		TIM8->BDTR = flags;
	}

	static inline void
	setRepetitionCount(uint16_t repetitionCount)
	{
		TIM8->RCR = repetitionCount;
	}

public:
	static void
	configureInputChannel(uint32_t channel, uint8_t filter);

	template<typename Signal>
	static void
	configureInputChannel(uint8_t filter)
	{
		constexpr auto channel = signalToChannel<Peripheral::Tim8, Signal>();
		configureInputChannel(channel, filter);
	}

	static void
	configureInputChannel(uint32_t channel, InputCaptureMapping input,
			InputCapturePrescaler prescaler,
			InputCapturePolarity polarity, uint8_t filter,
			bool xor_ch1_3=false);

	template<typename Signal>
	static void
	configureInputChannel(InputCaptureMapping input,
			InputCapturePrescaler prescaler,
			InputCapturePolarity polarity, uint8_t filter,
			bool xor_ch1_3=false)
	{
		constexpr auto channel = signalToChannel<Peripheral::Tim8, Signal>();
		configureInputChannel(channel, input, prescaler, polarity, filter, xor_ch1_3);
	}


	static void
	configureOutputChannel(uint32_t channel, OutputCompareMode mode,
			Value compareValue, PinState out = PinState::Enable);
	// TODO: Maybe add some functionality from the configureOutput
	//       function below...

	template<typename Signal>
	static void
	configureOutputChannel(OutputCompareMode mode,
			Value compareValue, PinState out = PinState::Enable)
	{
		constexpr auto channel = signalToChannel<Peripheral::Tim8, Signal>();
		configureOutputChannel(channel, mode, compareValue, out);
	}

	static void
	configureOutputChannel(uint32_t channel, OutputCompareMode mode,
			Value compareValue, PinState out,
			OutputComparePolarity polarity, PinState out_n,
			OutputComparePolarity polarity_n = OutputComparePolarity::ActiveHigh,
			OutputComparePreload preload = OutputComparePreload::Disable);

	template<typename Signal>
	static void
	configureOutputChannel(OutputCompareMode mode,
			Value compareValue, PinState out,
			OutputComparePolarity polarity, PinState out_n,
			OutputComparePolarity polarity_n = OutputComparePolarity::ActiveHigh,
			OutputComparePreload preload = OutputComparePreload::Disable)
	{
		constexpr auto channel = signalToChannel<Peripheral::Tim8, Signal>();
		configureOutputChannel(channel, mode, compareValue, out, polarity, out_n, polarity_n, preload);
	}

	/*
	 * Configure Output Channel without changing the Compare Value
	 *
	 * Normally used to REconfigure the Output channel without touching
	 * the compare value. This can e.g. be useful for commutation of a
	 * bldc motor.
	 *
	 * This function probably won't be used for a one time setup but
	 * rather for adjusting the output setting periodically.
	 * Therefore it aims aims to provide the best performance possible
	 * without sacrificing code readability.
	 */
	static void
	configureOutputChannel(uint32_t channel, OutputCompareMode mode,
			PinState out, OutputComparePolarity polarity,
			PinState out_n,
			OutputComparePolarity polarity_n = OutputComparePolarity::ActiveHigh,
			OutputComparePreload preload = OutputComparePreload::Disable);

	template<typename Signal>
	static void
	configureOutputChannel(OutputCompareMode mode,
			PinState out, OutputComparePolarity polarity,
			PinState out_n,
			OutputComparePolarity polarity_n = OutputComparePolarity::ActiveHigh,
			OutputComparePreload preload = OutputComparePreload::Disable)
	{
		constexpr auto channel = signalToChannel<Peripheral::Tim8, Signal>();
		configureOutputChannel(channel, mode, out, polarity, out_n, polarity_n, preload);
	}

	/*
	 * Configure Output Channel width Mode/OutputPort uint
	 *
	 * This is the least typesafe way of doing this and should only
	 * be done if it provides a necessary performance
	 * (or more or less) laziness benefit.
	 * i.e. if you have specific mode/output uints precalculated and
	 * just want to load them as fast as possible.
	 *
	 * The "mode/output" uint contains four bits
	 * that describe the intended output setting:
	 * Bit0: Output Enabled/Disabled
	 * Bit1: Output Polarity
	 * Bit2: Complementary Output Enable/Disable
	 * Bit3: Complementary Output Polarity
	 *
	 * As well as Mode Information (Bit6-Bit4)
	 * which is just an OutputCompareMode constant ored with the
	 * port output quadruple specified above.
	 */
	static void
	configureOutputChannel(uint32_t channel, uint32_t modeOutputPorts);

	template<typename Signal>
	static void
	configureOutputChannel(uint32_t modeOutputPorts)
	{
		constexpr auto channel = signalToChannel<Peripheral::Tim8, Signal>();
		configureOutputChannel(channel, modeOutputPorts);
	}

	static inline void
	setCompareValue(uint32_t channel, Value value)
	{
		if(channel <= 4) {
			*(&TIM8->CCR1 + (channel - 1)) = value;
		} else if(channel == 5) {
			TIM8->CCR5 = value;
		} else if(channel == 6) {
			TIM8->CCR6 = value;
		}
	}

	template<typename Signal>
	static void
	setCompareValue(Value value)
	{
		constexpr auto channel = signalToChannel<Peripheral::Tim8, Signal>();
		setCompareValue(channel, value);
	}

	static inline Value
	getCompareValue(uint32_t channel)
	{
		return *(&TIM8->CCR1 + (channel - 1));
	}

	template<typename Signal>
	static inline Value
	getCompareValue()
	{
		constexpr auto channel = signalToChannel<Peripheral::Tim8, Signal>();
		return getCompareValue(channel);
	}

public:
	/**
	 * Enables or disables Interrupt Vectors.
	 *
	 * You must pass an or-conjuncted list of entries from the
	 * Interrupt enum. Interrupt vectors which fit to
	 * will be enabled (or disabled if \p enable = false).
	 *
	 * The Adanvced timers have four interrupt vectors:
	 * - UP (used by INTERRUPT_UPDATE)
	 * - BRK (used by INTERRUPT_BREAK)
	 * - TRG_COM (used by INTERRUPT_TRIGGER and INTERRUPT_COM)
	 * - CC (used by INTERRUPT_CAPTURE_COMPARE_1..3)
	 */
	static void
	enableInterruptVector(Interrupt interrupt, bool enable, uint32_t priority);

	static inline void
	enableInterrupt(Interrupt interrupt)
	{
		TIM8->DIER |= static_cast<uint32_t>(interrupt);
	}

	static inline void
	disableInterrupt(Interrupt interrupt)
	{
		TIM8->DIER &= ~static_cast<uint32_t>(interrupt);
	}

	static inline void
	enableDmaRequest(DmaRequestEnable dmaRequests)
	{
		TIM8->DIER |= static_cast<uint32_t>(dmaRequests);
	}

	static inline void
	disableDmaRequest(DmaRequestEnable dmaRequests)
	{
		TIM8->DIER &= ~static_cast<uint32_t>(dmaRequests);
	}

	static inline InterruptFlag
	getInterruptFlags()
	{
		return static_cast<InterruptFlag>(TIM8->SR);
	}

	static inline void
	acknowledgeInterruptFlags(InterruptFlag flags)
	{
		// Flags are cleared by writing a zero to the flag position.
		// Writing a one is ignored.
		TIM8->SR = ~static_cast<uint32_t>(flags);
	}
};

}	// namespace modm::platform

#endif // MODM_STM32_TIMER_8_HPP