# TODO list for open topics

## Prioritized software-in-the-loop plan

See [docs/software_in_loop_plan.md](docs/software_in_loop_plan.md) for the
architecture, design decisions, implementation phases, risks, and acceptance
criteria.

### P0 - Establish the deterministic runtime

 - [ ] Define and test the virtual-time cycle contract around `CurrentControlIsr::on_jeoc()` and `SlowUpdate::run_once()`.
 - [ ] Introduce injectable ADC/PWM I/O for the current-control runtime while preserving the target hardware implementation.
 - [ ] Extract the current-control runtime into a reusable target independent of firmware `main.cpp`.
 - [ ] Add a HAL-free virtual clock and simulation runtime harness with reset, fast-step, slow-update, advance, and snapshot operations.

### P1 - Build the single-motor vertical slice

 - [ ] Implement a validated PMSM plant model using `NvmSettings`, existing frame types, and unit types.
 - [ ] Add plant tests for electrical dynamics, torque direction, pole-pair scaling, mechanics, friction, parameter validation, and repeatability.
 - [ ] Connect one host runtime and its ISR I/O to the PMSM plant instead of the zero-value host stubs.
 - [ ] Add a deterministic end-to-end C++ step-response test through the actual current-control runtime.
 - [ ] Add exact fast-loop/slow-loop cadence tests and finite-state checks for NaN/Inf and unstable output.

### P1 - Resolve the Cyphal transport path

 - [ ] Run a small Windows localhost Cyphal/UDP feasibility spike using `pycyphal`.
 - [ ] Select the C++ transport and DSDL generation approach for registers, setpoints, telemetry, and heartbeat.
 - [ ] Confirm one register exchange and one telemetry subject before designing the full simulator protocol.

### P2 - Add coordinated multi-motor simulation

 - [ ] Add a one-process simulator coordinator with N independent motor nodes and one shared virtual clock.
 - [ ] Define a barriered multi-motor step with coherent timestamps and deterministic ordering.
 - [ ] Add independent constant/viscous load models and tests for isolated motors, unique node IDs, reset, and repeatability.
 - [ ] Add a shared load interface suitable for shaft, wheel, battery, differential, and torque-vectoring models.

### P2 - Expose the simulator over Cyphal/UDP

 - [ ] Build a host simulator executable around the coordinator.
 - [ ] Map standard Cyphal register operations to the settings operation boundary.
 - [ ] Route torque, speed, position, and excitation setpoints into the normal controller input path.
 - [ ] Publish telemetry and heartbeat with explicit virtual-time semantics.
 - [ ] Add simulation-control commands for reset, fixed-cycle advance, run-until, snapshot, and clock status.
 - [ ] Add protocol and localhost end-to-end tests for single- and multi-node operation.

### P3 - Create the Python command-and-control GUI

 - [ ] Create a separate Python simulation package and dependency file; keep firmware `requirements.txt` focused on firmware tooling.
 - [ ] Implement a `pycyphal` client for node discovery, registers, setpoints, telemetry, reset, and virtual-time stepping.
 - [ ] Build a PySide6/pyqtgraph GUI for single-motor step responses and virtual-time control.
 - [ ] Add scenario definitions, run/pause/reset/step controls, CSV/JSON trace export, and replay.
 - [ ] Keep network I/O and long simulation batches outside the UI thread; support telemetry decimation.
 - [ ] Add headless Python protocol and smoke tests against the host simulator.

### P4 - Expand fidelity and application scope

 - [ ] Add ASM and EESM plants behind the common plant contract.
 - [ ] Add startup, observer, field-weakening, homing, and control-mode transition scenarios.
 - [ ] Add vehicle-level models for shared battery/DC-link behavior, wheels, differential load, road load, and torque vectoring.
 - [ ] Add fault injection, profiling, and large-N motor trace tooling.
 - [ ] Consider process-per-motor hosting only after the one-process coordinator is stable and a concrete isolation need exists.

### Validation and documentation

 - [ ] Add focused CMake/Ninja and CTest targets for plant, runtime, coordinator, and protocol tests.
 - [ ] Run clang-format and clang-tidy on changed C++ code.
 - [ ] Preserve the existing focused three-phase validation while SIL remains outside the active production scope.
 - [ ] Update `AGENTS.md` when SIL targets become part of the validated production scope.
 - [ ] Document simulator setup, launch commands, supported model fidelity, and GUI workflows in `README.md`.