# UNIMOC Agent Notes

UNIMOC is a platform-independent C++23 motor-control library and firmware
for field-oriented control of multi-phase electric motors. One drive typically
controls one motor; multiple drives can be networked through Cyphal/UAVCAN
over CAN-FD. Read `README.md` for the supported motor types and public
features, and read `CONTEXT.md` for the project's domain vocabulary.

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

Use the CMake presets and Ninja on both Windows and Linux hosts. The canonical
host validation workflow is documented in
[the hosted-build-test skill](.github/skills/engineering/hosted-build-test/SKILL.md).

### Active refactor validation scope

The system library is currently under active review and is not yet a
production baseline. Until the user explicitly expands this scope, treat only
the following as validated:

- `lib/units/units.hpp`
- `lib/system/three_phase_system.hpp`
- `tests/three_phase_system_test.cpp` and its `three_phase_system_test` target

For changes confined to this scope, use focused hosted validation first:

```sh
cmake --build --preset "Hosted build with tests" --target three_phase_system_test
ctest --preset "Hosted Test" -R "ThreePhaseTest" --output-on-failure
```

Do not run or require the full hosted workflow merely because an in-progress
system, observer, control, firmware, or integration file changed. Such code
may be compiled or tested when needed to investigate the specific change, but
failures outside the active scope are expected refactor work rather than a
regression in the validated baseline. Revisit this section whenever the user
declares another path or test production-valid.

The normal workflow is:

```sh
cmake --workflow --preset "Hosted configure, build and test"
```

For static analysis and formatting, use the `hosted-build-test` skill. It
covers the `clang-tidy` workflow preset and `clang-format` checks on both host
platforms.
Firmware presets are `BatteryCaseController Debug` and `BatteryCaseController
Release`; they require the ARM GCC and modm dependencies described in
`README.md`.

## Coding Guidelines

- prefer #pragma once over include guards. remove include guards when applicable.
- Use the Google C++ style configured in `.clang-format`.
- Run the repository's clang-format configuration when changing C++.
- Preserve the existing public-header naming convention; new files should match
	the convention of the directory they belong to.
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

## Git Safety

- Do not stage or commit changes unless the user explicitly requests it.
- Do not create branches, reset, checkout files, abort, or continue a merge or
  rebase unless the user explicitly requests that operation.
- Leave the working tree and merge state available for the user when a workflow
  reaches a git lifecycle boundary.

## Change Checklist

Before finishing a change:

1. Build and run the focused tests for the active refactor validation scope;
	 use the full hosted workflow only when the user expands the scope or the
	 change requires cross-library validation.
2. Run clang-tidy for changes that affect C++ diagnostics or public headers. Fix issues in the change or add a `NOLINT` comment with a justification.
3. Check the diff for accidental changes to generated files, build output, or
	 submodules.
4. Update `README.md` when user-visible behavior, configuration, build steps,
	 or hardware bring-up behavior changes.

## Skills

Based on [mattpocock's skills](https://github.com/mattpocock/skills), adapted for
VS Code Copilot and this C++23 project. Skills are organized under
`.github/skills/`:

- [Engineering](.github/skills/engineering/README.md) — daily code work.
- [Productivity](.github/skills/productivity/README.md) — daily non-code workflow tools.
- [Hosted build and test](.github/skills/engineering/hosted-build-test/SKILL.md) —
	Windows/Linux hosted validation, clang-tidy, and clang-format.

Use the repository's `SKILL.md` files as the source of truth for these
workflows.





