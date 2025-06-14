/*
 * Copyright (c) 2013-2014, 2016, Kevin Läufer
 * Copyright (c) 2013-2018, Niklas Hauser
 * Copyright (c) 2016, Fabian Greif
 * Copyright (c) 2016, Sascha Schade
 * Copyright (c) 2017, Michael Thies
 * Copyright (c) 2021, Christopher Durand
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include <stdint.h>
#include <modm/architecture/utils.hpp>
#include <modm/architecture/interface/assert.h>
#include "../device.hpp"

// ----------------------------------------------------------------------------
// Default implementation of platform initialization
modm_weak void modm_initialize_platform(void) {}
extern void __modm_initialize_memory(void);
extern int main(void);

// ----------------------------------------------------------------------------
// Linker section start and end pointers
extern const uint64_t __table_copy_intern_start[];
extern const uint64_t __table_copy_intern_end[];

extern const uint64_t __table_zero_intern_start[];
extern const uint64_t __table_zero_intern_end[];

extern const uint64_t __table_copy_extern_start[];
extern const uint64_t __table_copy_extern_end[];

extern const uint64_t __table_zero_extern_start[];
extern const uint64_t __table_zero_extern_end[];

extern const uint32_t __vector_table_ram_start[];

// ----------------------------------------------------------------------------
// Linker section start and end pointers for function hooks
typedef void (* const FunctionPointer)(void);

extern const FunctionPointer __hardware_init_start[];
extern const FunctionPointer __hardware_init_end[];

extern const FunctionPointer __init_array_start[];
extern const FunctionPointer __init_array_end[];

// ----------------------------------------------------------------------------
// Calls every function pointer in the section range
static inline void
table_call(const FunctionPointer *const start, const FunctionPointer *const end)
{
	for (const FunctionPointer *entry = start; entry < end; entry++)
		(*entry)();
}

// Copies the section defined by a table of {loadaddr, dest start, dest end}
static inline void
table_copy(const uint64_t *const start, const uint64_t *const end)
{
	uint64_t **table = (uint64_t **)start;
	while(table < (uint64_t **)end)
	{
		const uint64_t *src  = table[0]; // load address
		      uint64_t *dest = table[1]; // destination start
		while (dest < table[2])          // destination end
			*(dest++) = *(src++);
		table += 3;
	}
}

// Zeros the section defined by a table of {start, end}
static inline void
table_zero(const uint64_t *const start, const uint64_t *const end)
{
	uint64_t **table = (uint64_t **)start;
	while(table < (uint64_t **)end)
	{
		uint64_t *dest = table[0]; // destination start
		while (dest < table[1])    // destination end
			*(dest++) = 0;
		table += 2;
	}
}

// ----------------------------------------------------------------------------
// Called by Reset_Handler in reset_handler.s
void __modm_startup(void)
{

	// Copy and zero all internal memory
	table_copy(__table_copy_intern_start, __table_copy_intern_end);
	table_zero(__table_zero_intern_start, __table_zero_intern_end);

	// Set the vector table location
	SCB->VTOR = (uint32_t)__vector_table_ram_start;

	// Enable trapping of divide by zero for UDIV/SDIV instructions.
	SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;

	// Call all hardware initialize hooks
	table_call(__hardware_init_start, __hardware_init_end);

	// Copy and zero all external memory
	table_copy(__table_copy_extern_start, __table_copy_extern_end);
	table_zero(__table_zero_extern_start, __table_zero_extern_end);

	// Initialize heap as implemented by the heap option
	__modm_initialize_memory();

	// Call all constructors of static objects
	table_call(__init_array_start, __init_array_end);

	// Call the application's entry point
	main();

	// If main exits, assert here in debug mode
	(void) modm_assert_continue_fail_debug(0,
			"main.exit", "The main() function returned!");
	// Otherwise reboot
	NVIC_SystemReset();
}