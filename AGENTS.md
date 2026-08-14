# UNIMOC Agent Notes

UNIMOC is a platform-independent C++23 motor-control library and firmware
for field-oriented control of multi-phase electric motors. One controller typically
drives one motor; multiple controllers can be networked through Cyphal/UAVCAN
over CAN-FD. Read `README.md` for the supported motor types and public
features.

## Project Layout

- `lib/control/`, `lib/observer/`, `lib/system/`, and `lib/units/` contain the
	portable control algorithms, observers, data types, and unit wrappers.
- `src/` contains the firmware application and runtime integrations such as
	current control, Cyphal, logging, and startup.
- `hardware/` contains hardware-specific implementations. Keep platform
	details out of the portable library where possible.
- `tests/` contains hosted GoogleTest coverage for the portable code.
- `lib/cymon/` and `lib/gitversion/` are submodules; do not modify them unless
	the task explicitly concerns those dependencies.

## Build And Test

Use the CMake presets and Ninja. The normal host validation workflow is:

```sh
cmake --preset "Hosted build with tests"
cmake --build --preset "Hosted build with tests"
ctest --preset "Hosted Test" --output-on-failure
```

For static analysis, use the corresponding `Hosted build with tests +
clang-tidy` configure/build and `Hosted Test + clang-tidy` test presets.
Firmware presets are `BatteryCaseController Debug` and `BatteryCaseController
Release`; they require the ARM GCC and modm dependencies described in
`README.md`.

## Coding Guidelines

- prefer #pragma once over include guards. remove include guards when applicable.
- General coding style is google code c++. Read `.clang-format` for additional infos.
- Run the repository's clang-format configuration when changing C++.
- File naming is allways snake_case for code.
- Use C++23
- Keep algorithms deterministic, portable, and free of HAL dependencies.
- Prefer small, focused changes. Preserve public APIs unless the task requires
	a breaking change, and avoid unrelated refactors.
- Add or update GoogleTest coverage in `tests/` for changes to control,
	observer, system, or unit behavior.
- Treat compiler warnings as errors and keep changes compatible with the
	project's strict warning set.
- Be especially careful with current limits, PWM, startup, NVM settings, and
	Cyphal register or subject IDs. Do not assume hardware behavior from hosted
	tests; document hardware-specific assumptions and validate firmware changes
	with the appropriate target build.

## Change Checklist

Before finishing a change:

1. Build and run the focused hosted tests, or the full hosted workflow when
	 practical.
2. Run clang-tidy for changes that affect C++ diagnostics or public headers.
3. Check the diff for accidental changes to generated files, build output, or
	 submodules.
4. Update `README.md` when user-visible behavior, configuration, build steps,
	 or hardware bring-up behavior changes.

## Skills

based on [mattpocock's skills](https://github.com/mattpocock/skills)
Skills are organized into bucket folders under skills/:
 - engineering/ — daily code work
 - productivity/ — daily non-code workflow tools





