/*
 *       __  ___   ________  _______  ______
 *      / / / / | / /  _/  |/  / __ \/ ____/
 *     / / / /  |/ // // /|_/ / / / / /
 *    / /_/ / /|  // // /  / / / /___
 *    \____/_/ |_/___/_/  /_/\____/\____/
 *
 *    @file nvm_settings.hpp
 *    @brief Typed persistent settings for UNIMOC.
 *
 *    This file is part of UNIMOC and is licensed under GPL-3.0-or-later.
 *    See the repository LICENSE file for details.
 */
#pragma once

#include <cstdint>
#include "ControlMode.hpp"
#include "MotorType.hpp"
#include "NodeIdentity.hpp"
#include "units.hpp"

// ============================================================================
// Standalone enums (declared before NvmSettings so they can be used as
// member types with in-class initialisers).
// ============================================================================

/**
 * @namespace unimoc global namespace
 */
namespace unimoc {
/**
 * @namespace system coordinate and motor type definitions
 */
namespace system {

/// Magic number stored at the start of every NvmSettings block.
/// Used to detect an uninitialised or corrupt NVM image.
inline constexpr uint32_t kNvmMagic = 0x554D4F43u;  // 'UMOC'

/// Layout version of the NvmSettings struct.
/// Increment whenever the struct layout changes incompatibly.
inline constexpr uint16_t kNvmVersion = 3u;

/**
 * @brief Aggregate of all persistent (NVM-backed) UNIMOC configuration.
 *
 * Every member that a user can configure via Cyphal registers and that must
 * survive a power cycle lives in this struct.  The hardware NVM driver
 * stores and retrieves this struct atomically (e.g., via a CRC-protected
 * flash page or EEPROM block).
 *
 * Cyphal register mapping
 * -----------------------
 * Each field corresponds to a named Cyphal register (see CyphalInterface.hpp
 * for the canonical register name strings).  The Cyphal register service
 * (`uavcan.register.Access`) is the write path; on successful write the
 * application flushes the updated NvmSettings to non-volatile storage.
 *
 * Node ID and plug-and-play
 * -------------------------
 * `node_id == 0` is the sentinel value meaning "not assigned".  When the
 * firmware boots with `node_id == 0` it initiates the Cyphal plug-and-play
 * allocation sequence (`uavcan.pnp.NodeIDAllocationData`) and stores the
 * allocated ID in this field for future boots.
 *
 * Default values
 * --------------
 * All fields are initialised to sensible defaults so that an unprogrammed
 * device boots and operates in a known state without manual configuration.
 */
struct NvmSettings {
  // =========================================================================
  // Header — must be first
  // =========================================================================

  /// Magic word — set to kNvmMagic by factory defaults.
  uint32_t magic{kNvmMagic};

  /// Layout version — set to kNvmVersion.
  uint16_t version{kNvmVersion};

  // =========================================================================
  // Node identification
  // =========================================================================

  /// Cyphal node ID [1..127].  0 = not assigned → use PnP allocation.
  /// Register: `uavcan.node.id`
  uint8_t node_id{0};

  /// Human-readable node identity (name + hw/sw versions).
  /// Register: `uavcan.node.description` (name field only over Cyphal).
  NodeIdentity identity{};

  // =========================================================================
  // System
  // =========================================================================

  /// Active motor type.
  /// Register: `unimoc.motor.type`
  MotorType motor_type{MotorType::PMSM};

  /// Motor pole-pair count.
  /// Register: `unimoc.motor.pole_pairs`
  uint8_t pole_pairs{1};

  /// Initial control mode after boot (overridden at runtime via Cyphal).
  /// Register: `unimoc.control.mode`
  ControlMode control_mode{ControlMode::TORQUE};

  // =========================================================================
  // Stator / winding parameters (all motor types)
  // =========================================================================

  /// Stator phase resistance [Ω].
  /// Register: `unimoc.motor.stator.R`
  unit::Resistance stator_r{unit::Resistance{0.1f}};

  /// Stator inductance [H] (L_q for IPMSM; average (L_d+L_q)/2 otherwise).
  /// Register: `unimoc.motor.stator.L`
  unit::Inductance stator_l{unit::Inductance{1e-3f}};

  // =========================================================================
  // Motor operating limits
  //
  // These are runtime-configurable limits; they must never exceed the
  // immutable capabilities supplied by the selected hardware profile.
  // =========================================================================

  /// Maximum motor (resultant stator vector) current [A].
  /// Hard upper bound: hardware profile's maximum motor current.
  /// Register: `unimoc.motor.limits.i_max`
  unit::Current motor_i_max{unit::Current{40.0f}};

  /// Maximum electrical angular velocity in the forward direction [rad/s].
  /// Register: `unimoc.motor.limits.omega_max`
  unit::AngularVelocity motor_omega_max{unit::AngularVelocity{2000.0f}};

  /// Maximum electrical angular velocity in the reverse direction [rad/s]
  /// (stored as a negative value).
  /// Register: `unimoc.motor.limits.omega_min`
  unit::AngularVelocity motor_omega_min{unit::AngularVelocity{-2000.0f}};

  /// Maximum battery discharge (drive) current [A].
  /// Hard upper bound: hardware profile's maximum drive current.
  /// Register: `unimoc.battery.limits.drive_current`
  unit::Current battery_drive_current_max{unit::Current{15.0f}};

  /// Maximum battery charge (regenerative braking) current [A].
  /// Hard upper bound: hardware profile's maximum charge current.
  /// Register: `unimoc.battery.limits.charge_current`
  unit::Current battery_charge_current_max{unit::Current{5.0f}};

  // =========================================================================
  // PMSM / EESM parameters
  // =========================================================================

  /// Permanent-magnet flux linkage ψ_PM [Wb].
  /// For EESM, this is the initial/nominal ψ_f used until the excitation
  /// observer provides a measured value.
  /// Register: `unimoc.motor.pmsm.flux_pm`
  unit::MagneticFlux flux_pm{unit::MagneticFlux{0.0f}};

  /// d-axis inductance L_d [H].
  /// Register: `unimoc.motor.pmsm.L_d`
  unit::Inductance l_d{unit::Inductance{1e-3f}};

  /// q-axis inductance L_q [H].
  /// Register: `unimoc.motor.pmsm.L_q`
  unit::Inductance l_q{unit::Inductance{1e-3f}};

  // =========================================================================
  // Asynchronous motor (ASM) parameters
  // =========================================================================

  /// Rotor resistance R_r [Ω].
  /// Register: `unimoc.motor.asm.R_r`
  unit::Resistance asm_r_r{unit::Resistance{0.3f}};

  /// Stator resistance R_s [Ω].
  /// Register: `unimoc.motor.asm.R_s`
  unit::Resistance asm_r_s{unit::Resistance{0.5f}};

  /// Stator self-inductance L_s [H].
  /// Register: `unimoc.motor.asm.L_s`
  unit::Inductance asm_l_s{unit::Inductance{50e-3f}};

  /// Rotor self-inductance L_r [H].
  /// Register: `unimoc.motor.asm.L_r`
  unit::Inductance asm_l_r{unit::Inductance{50e-3f}};

  /// Mutual (magnetising) inductance L_m [H].
  /// Register: `unimoc.motor.asm.L_m`
  unit::Inductance asm_l_m{unit::Inductance{47e-3f}};

  // =========================================================================
  // Mechanical observer (Kalman filter) parameters
  // =========================================================================

  /// Rotor + load inertia J [kg·m²].
  /// Used by MechanicalObserver::predict() to integrate the torque equation.
  /// Register: `unimoc.motor.mechanics.J`
  unit::Inertia motor_j{unit::Inertia{1e-4f}};

  /// Kalman filter process noise variance Q.
  /// Increase to let the filter track faster but noisier.
  /// Register: `unimoc.observer.mech.Q`
  float mech_obs_q{1e-5F};

  /// Kalman filter measurement noise variance R.
  /// Increase to smooth angle corrections at the cost of slower tracking.
  /// Register: `unimoc.observer.mech.R`
  float mech_obs_r{1e-4F};

  // =========================================================================
  // PMSM flux observer (PmsmFluxObserver) gains
  // =========================================================================

  /// d-axis anti-drift feedback gain C_d [1/s].
  /// Register: `unimoc.observer.pmsm_flux.C_d`
  unit::InverseTime pmsm_flux_obs_c_d{unit::InverseTime{50.0f}};

  /// q-axis anti-drift feedback gain C_q [1/s].
  /// Register: `unimoc.observer.pmsm_flux.C_q`
  unit::InverseTime pmsm_flux_obs_c_q{unit::InverseTime{1.0f}};

  // =========================================================================
  // ASM flux observer gains
  // =========================================================================

  /// Stator-current correction gain g_i [1/s].
  /// Register: `unimoc.observer.asm_flux.g_i`
  unit::InverseTime asm_obs_g_i{unit::InverseTime{500.0f}};

  /// Rotor-flux correction gain g_flux [Wb/(A·s)].
  /// Register: `unimoc.observer.asm_flux.g_flux`
  float asm_obs_g_flux{5000.0f};

  // =========================================================================
  // ASM flux controller
  // =========================================================================

  /// Proportional gain [A/Wb].
  /// Register: `unimoc.control.asm_flux.kp`
  float asm_flux_kp{10.0f};

  /// Integral gain [A/(Wb·s)].
  /// Register: `unimoc.control.asm_flux.ki`
  float asm_flux_ki{50.0f};

  /// Minimum d-axis current [A].
  /// Register: `unimoc.control.asm_flux.i_d_min`
  unit::Current asm_flux_i_d_min{unit::Current{0.0F}};

  /// Maximum d-axis current [A].
  /// Register: `unimoc.control.asm_flux.i_d_max`
  unit::Current asm_flux_i_d_max{unit::Current{10.0F}};

  // =========================================================================
  // Field weakening
  // =========================================================================

  /// Maximum voltage vector magnitude (normalised, range (0,1]).
  /// Register: `unimoc.control.fw.v_max`
  unit::DimensionlessRatio fw_v_max{unit::DimensionlessRatio{0.9F}};

  /// Integrator gain [A/(V·s)].
  /// Register: `unimoc.control.fw.ki`
  float fw_ki{10.0f};

  /// Most negative i_d allowed [A].
  /// Register: `unimoc.control.fw.i_d_min`
  unit::Current fw_i_d_min{unit::Current{-10.0F}};

  // =========================================================================
  // PWM frequency
  // =========================================================================

  /// PWM switching frequency for the current-control timer.
  ///
  /// The ISR fires at 2× this frequency (once at peak, once at trough of the
  /// centre-aligned timer) so the effective current-loop rate is 2 × f_pwm.
  /// The slow-update task runs at (2 × f_pwm) / 4 = f_pwm / 2.
  ///
  /// Register: `unimoc.control.pwm_frequency`
  unit::Frequency pwm_frequency{unit::Frequency{20000.0F}};

  // =========================================================================
  // Current controller (d/q-axis PI with cross-coupling feedforward)
  // =========================================================================

  /// d-axis proportional gain [V/A].
  /// Register: `unimoc.control.current.kp_d`
  float current_kp_d{1.0f};

  /// d-axis integral gain [V/(A·s)].
  /// Register: `unimoc.control.current.ki_d`
  float current_ki_d{100.0f};

  /// q-axis proportional gain [V/A].
  /// Register: `unimoc.control.current.kp_q`
  float current_kp_q{1.0f};

  /// q-axis integral gain [V/(A·s)].
  /// Register: `unimoc.control.current.ki_q`
  float current_ki_q{100.0f};

  /// Maximum voltage vector magnitude (normalised by V_dc, range (0, 1]).
  /// Limits the current controller output before it reaches the SVM
  /// modulator.  Typically set slightly below the SVM duty_max to preserve
  /// headroom for dead-time compensation and ADC sampling.
  /// Register: `unimoc.control.current.v_max`
  unit::DimensionlessRatio current_v_max{unit::DimensionlessRatio{0.9f}};

  // =========================================================================
  // SVM modulator
  // =========================================================================

  /// Minimum duty cycle (headroom for ADC + dead-time).
  /// Register: `unimoc.control.svm.duty_min`
  unit::DimensionlessRatio svm_duty_min{unit::DimensionlessRatio{0.05f}};

  /// Maximum duty cycle.
  /// Register: `unimoc.control.svm.duty_max`
  unit::DimensionlessRatio svm_duty_max{unit::DimensionlessRatio{0.95f}};

  // =========================================================================
  // Dead-time compensation
  // =========================================================================

  /// Gate-driver dead time [s].
  /// Register: `unimoc.control.dtc.dead_time`
  unit::Time dtc_dead_time{unit::Time{0.0f}};

  /// PWM switching frequency [Hz].
  /// Register: `unimoc.control.dtc.f_pwm`
  unit::Frequency dtc_f_pwm{unit::Frequency{10000.0f}};

  /// Phase-current zero-crossing threshold [A].
  /// Register: `unimoc.control.dtc.i_threshold`
  unit::Current dtc_i_threshold{unit::Current{0.1f}};

  // =========================================================================
  // HFI (high-frequency injection) observer
  // =========================================================================

  /// HFI injection voltage [V].
  /// Register: `unimoc.observer.hfi.v_inject`
  unit::Voltage hfi_v_inject{unit::Voltage{0.0f}};

  /// Angle error to PLL scaling gain [1/V].
  /// Register: `unimoc.observer.hfi.error_gain`
  float hfi_error_gain{1.0f};

  // =========================================================================
  // EESM excitation controller
  // =========================================================================

  /// Excitation mode: 0 = CurrentMode, 1 = FluxMode.
  /// Register: `unimoc.control.excitation.mode`
  uint8_t excitation_mode{0};

  /// Mutual inductance L_m [H] for flux↔current conversion.
  /// Register: `unimoc.control.excitation.L_m`
  unit::Inductance excitation_l_m{unit::Inductance{47e-3f}};

  /// Proportional gain [V/A].
  /// Register: `unimoc.control.excitation.kp`
  float excitation_kp{5.0f};

  /// Integral gain [V/(A·s)].
  /// Register: `unimoc.control.excitation.ki`
  float excitation_ki{50.0f};

  /// Minimum rotor excitation current [A].
  /// Register: `unimoc.control.excitation.i_f_min`
  unit::Current excitation_i_f_min{unit::Current{0.0f}};

  /// Maximum rotor excitation current [A].
  /// Register: `unimoc.control.excitation.i_f_max`
  unit::Current excitation_i_f_max{unit::Current{10.0f}};

  // =========================================================================
  // EESM excitation observer
  // =========================================================================

  /// Low-pass filter time constant [s].
  /// Register: `unimoc.observer.excitation.tau`
  unit::Time excitation_obs_tau{unit::Time{2e-3f}};

  /// Mutual inductance L_m [H] used by the observer.
  /// Register: `unimoc.observer.excitation.L_m`
  unit::Inductance excitation_obs_l_m{unit::Inductance{47e-3f}};

  // =========================================================================
  // Position controller
  // =========================================================================

  /// Position loop proportional gain [rad/s per rad].
  /// Register: `unimoc.control.pos.kp`
  float pos_kp_pos{10.0f};

  /// Speed loop proportional gain.
  /// Register: `unimoc.control.pos.kp_speed`
  float pos_kp_speed{5.0f};

  /// Speed loop integral gain.
  /// Register: `unimoc.control.pos.ki_speed`
  float pos_ki_speed{20.0f};

  /// Maximum mechanical angular velocity [rad/s].
  /// Register: `unimoc.control.pos.speed_limit`
  unit::AngularVelocity pos_speed_limit{unit::AngularVelocity{100.0f}};

  /// Maximum speed-demand rate of change [rad/s²].
  /// Register: `unimoc.control.pos.accel_limit`
  unit::AngularAcceleration pos_accel_limit{unit::AngularAcceleration{500.0f}};

  /// In-position position tolerance [rad].
  /// Register: `unimoc.control.pos.position_tolerance`
  unit::Angle pos_position_tolerance{unit::Angle{0.01f}};

  /// In-position speed tolerance [rad/s].
  /// Register: `unimoc.control.pos.speed_tolerance`
  unit::AngularVelocity pos_speed_tolerance{unit::AngularVelocity{1.0f}};

  /// Constant homing velocity [rad/s].
  /// Register: `unimoc.control.pos.homing_speed`
  unit::AngularVelocity pos_homing_speed{unit::AngularVelocity{5.0f}};

  // =========================================================================
  // ADC calibration (populated by the hardware startup aid)
  //
  // These values are measured by HwStartup and stored here for persistence.
  // The hardware-specific adc_read_injected() implementation should apply
  // them as:  i_cal = (i_raw - adc_offset_X) * adc_gain_X
  //           v_cal = v_raw * adc_gain_vdc
  // =========================================================================

  /// Phase-A current-sense ADC zero offset [A].
  /// Register: `unimoc.startup.adc_offset_a`
  unit::Current adc_offset_a{unit::Current{0.0f}};

  /// Phase-B current-sense ADC zero offset [A].
  /// Register: `unimoc.startup.adc_offset_b`
  unit::Current adc_offset_b{unit::Current{0.0f}};

  /// Phase-A current-sense ADC gain correction factor [dimensionless].
  /// Apply as: i_cal_a = (i_raw_a - adc_offset_a) * adc_gain_a.
  /// Register: `unimoc.startup.gain_a`
  unit::DimensionlessRatio adc_gain_a{unit::DimensionlessRatio{1.0f}};

  /// Phase-B current-sense ADC gain correction factor [dimensionless].
  /// Register: `unimoc.startup.gain_b`
  unit::DimensionlessRatio adc_gain_b{unit::DimensionlessRatio{1.0f}};

  /// DC-link voltage ADC gain correction factor [dimensionless].
  /// Apply as: v_cal = v_raw * adc_gain_vdc.
  /// Register: `unimoc.startup.gain_vdc`
  unit::DimensionlessRatio adc_gain_vdc{unit::DimensionlessRatio{1.0f}};

  // =========================================================================
  // Phase current balance correction
  //
  // Per-phase ADC gain correction factors, dimensionless (nominal value 1.0).
  // Applied to the raw phase-current ADC reading of each channel to
  // compensate for hardware gain/offset mismatch between the three current
  // sense paths.  Identified by CMD_MEASURE_BALANCE.
  // =========================================================================

  /// Phase-A ADC gain correction factor [dimensionless, ≈ 1.0].
  /// Register: `unimoc.motor.balance.gain_a`
  unit::DimensionlessRatio phase_balance_a{unit::DimensionlessRatio{1.0f}};

  /// Phase-B ADC gain correction factor [dimensionless, ≈ 1.0].
  /// Register: `unimoc.motor.balance.gain_b`
  unit::DimensionlessRatio phase_balance_b{unit::DimensionlessRatio{1.0f}};

  /// Phase-C ADC gain correction factor [dimensionless, ≈ 1.0].
  /// Register: `unimoc.motor.balance.gain_c`
  unit::DimensionlessRatio phase_balance_c{unit::DimensionlessRatio{1.0f}};

  // =========================================================================
  // Validation and safety clamping
  // =========================================================================

  /**
   * @brief Return true if the magic word and version match expected values.
   *
   * Call this after loading from NVM.  If it returns false the block is
   * uninitialised or corrupt — reset to defaults and re-save.
   */
  [[nodiscard]] constexpr bool IsValid() const noexcept { return magic == kNvmMagic && version == kNvmVersion; }

  /// Restore all fields to factory defaults.
  void ResetToDefaults() noexcept { *this = NvmSettings{}; }
};

}  // namespace system
}  // namespace unimoc
