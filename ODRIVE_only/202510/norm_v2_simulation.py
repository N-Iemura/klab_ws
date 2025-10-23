"""
Simulation harness for ``norm_v2.py`` that runs entirely offline.

This script mocks the minimal subset of the ODrive API that ``norm_v2`` relies on,
advances a lightweight planar two-motor mechanism model, and reuses the original
control logic without modification.  Use it when you want to try the controller
logic without access to physical hardware.

Usage:
    python norm_v2_simulation.py [--duration 12.0] [--keep all|csv|fig|none] [--seed 0]
"""

from __future__ import annotations

import argparse
import sys
import types
from pathlib import Path
from typing import Dict, Optional

import numpy as np
import math


# ==================================================================================
# Mock enums (mirror the pieces imported from odrive.enums)
# ==================================================================================


class AxisState:
    IDLE = 1
    CLOSED_LOOP_CONTROL = 8


class ControlMode:
    TORQUE_CONTROL = 1


def _build_enums_module() -> types.ModuleType:
    module = types.ModuleType("odrive.enums")
    module.AxisState = AxisState
    module.ControlMode = ControlMode
    module.__all__ = ["AxisState", "ControlMode"]
    return module


# ==================================================================================
# Simulated ODrive hardware representation
# ==================================================================================


class SimPosVelMapper:
    __slots__ = ("pos_rel", "vel")

    def __init__(self) -> None:
        self.pos_rel = 0.0
        self.vel = 0.0


class SimAxisController:
    __slots__ = ("_plant", "_index", "config")

    def __init__(self, plant: "MechanismPlant", index: Optional[int]) -> None:
        self._plant = plant
        self._index = index
        self.config = types.SimpleNamespace(control_mode=None)

    @property
    def input_torque(self) -> float:
        if self._index is None:
            return 0.0
        return float(self._plant.tau_cmd[self._index])

    @input_torque.setter
    def input_torque(self, value: float) -> None:
        if self._index is None:
            return
        self._plant.tau_cmd[self._index] = float(value)


class SimAxisConfig:
    __slots__ = ("motor",)

    def __init__(self) -> None:
        self.motor = types.SimpleNamespace(torque_constant=0.0)


class SimAxis:
    __slots__ = ("controller", "config", "pos_vel_mapper", "requested_state")

    def __init__(self, plant: "MechanismPlant", index: Optional[int]) -> None:
        self.controller = SimAxisController(plant, index)
        self.config = SimAxisConfig()
        self.pos_vel_mapper = SimPosVelMapper()
        self.requested_state = AxisState.IDLE
        plant.attach_axis(index, self)


class SimulatedDrive:
    __slots__ = ("axis0",)

    def __init__(self, plant: "MechanismPlant", index: Optional[int]) -> None:
        self.axis0 = SimAxis(plant, index)


class SimulatedODriveModule(types.ModuleType):
    """
    Minimal drop-in replacement for ``odrive`` that serves deterministic
    simulated devices keyed by serial number.
    """

    def __init__(self, plant: "MechanismPlant") -> None:
        super().__init__("odrive")
        self._plant = plant
        self._devices: Dict[str, SimulatedDrive] = {}

    def register(self, serial_number: str, drive: SimulatedDrive) -> None:
        self._devices[serial_number] = drive

    def find_any(self, serial_number: Optional[str] = None) -> SimulatedDrive:
        if serial_number is None:
            raise ValueError("Simulated find_any requires a serial_number.")
        try:
            return self._devices[serial_number]
        except KeyError as exc:
            raise RuntimeError(f"Unknown simulated serial: {serial_number}") from exc


# ==================================================================================
# Plant model
# ==================================================================================


class MechanismPlant:
    """
    Coarse planar two-motor mechanism model.

    The plant captures inertia, viscous friction, elastic restoring torques, and a
    load reflected through the mechanism matrix A(q).  Dynamics are intentionally
    simple; the goal is to provide a smooth target for the controller rather than
    faithfully replicate hardware.
    """

    def __init__(self, max_internal_dt: float = 0.001, noise_std: float = 1e-4) -> None:
        self.max_internal_dt = max_internal_dt
        self.noise_std = noise_std

        self.q = np.zeros(2, dtype=float)
        self.qdot = np.zeros(2, dtype=float)
        self.tau_cmd = np.zeros(2, dtype=float)

        self.axes: Dict[int, SimAxis] = {}
        self.output_axis: Optional[SimAxis] = None

        self.inertia = np.array([0.028, 0.016], dtype=float)
        self.joint_damping = np.array([0.14, 0.19], dtype=float)
        self.joint_stiffness = np.array([2.5, 1.9], dtype=float)

        self.output_stiffness = 9.0
        self.output_damping = 0.9

        self.theta_out = 0.0
        self.theta_out_vel = 0.0

        self.get_A = None  # set via configure()
        self.ideal_mode = False
        self.last_theta_ref = 0.0
        self.last_ref_time = 0.0
        self.first_order_tau: Optional[float] = None

    def configure(self, control_module: types.ModuleType) -> None:
        self.get_A = control_module.get_A

    def attach_axis(self, index: Optional[int], axis: SimAxis) -> None:
        if index is None:
            self.output_axis = axis
        else:
            self.axes[index] = axis

    def set_ideal_mode(self, flag: bool) -> None:
        self.ideal_mode = bool(flag)

    def set_first_order_tau(self, tau: Optional[float]) -> None:
        if tau is None or tau <= 0.0:
            self.first_order_tau = None
        else:
            self.first_order_tau = float(tau)

    def step(self, dt: float) -> None:
        if dt <= 0.0:
            return
        if self.ideal_mode:
            self._update_mappers()
            return
        steps = max(1, int(np.ceil(dt / self.max_internal_dt)))
        h = dt / steps
        for _ in range(steps):
            self._integrate(h)
        self._update_mappers()

    def _integrate(self, h: float) -> None:
        A = np.asarray(self.get_A(self.q), dtype=float).reshape(1, 2)
        At = A.T
        s = float(A @ At)
        if s < 1e-9:
            s = 1e-9

        theta_out = float(A @ self.q.reshape(2, 1))
        theta_out_vel = float(A @ self.qdot.reshape(2, 1))

        tau_output_load = -self.output_stiffness * theta_out - self.output_damping * theta_out_vel
        reaction = (At * (tau_output_load / s)).reshape(2)

        net_tau = self.tau_cmd + reaction
        net_tau -= self.joint_damping * self.qdot
        net_tau -= self.joint_stiffness * self.q

        qdd = net_tau / self.inertia
        self.qdot += qdd * h
        self.q += self.qdot * h

        self.theta_out = float(A @ self.q.reshape(2, 1))
        self.theta_out_vel = float(A @ self.qdot.reshape(2, 1))

    def _update_mappers(self) -> None:
        for idx, axis in self.axes.items():
            axis.pos_vel_mapper.pos_rel = float(self.q[idx] + np.random.normal(0.0, self.noise_std))
            axis.pos_vel_mapper.vel = float(self.qdot[idx] + np.random.normal(0.0, self.noise_std))

        if self.output_axis is not None:
            self.output_axis.pos_vel_mapper.pos_rel = float(self.theta_out + np.random.normal(0.0, self.noise_std))
            self.output_axis.pos_vel_mapper.vel = float(self.theta_out_vel + np.random.normal(0.0, self.noise_std))

    def enforce_output(self, theta_ref: float, elapsed_time: float) -> None:
        prev_theta = self.theta_out
        prev_q = self.q.copy()
        dt = elapsed_time - self.last_ref_time
        if dt < 0.0:
            dt = 0.0

        A = np.asarray(self.get_A(prev_q), dtype=float).reshape(1, 2)
        At = A.T
        s = float(A @ At)
        if s < 1e-9:
            s = 1e-9

        theta_target = theta_ref
        if self.first_order_tau is not None and dt > 0.0:
            alpha = math.exp(-dt / self.first_order_tau)
            theta_target = alpha * prev_theta + (1.0 - alpha) * theta_ref

        q_target = (At * (theta_target / s)).reshape(2)
        if dt <= 0.0:
            qdot_target = np.zeros(2, dtype=float)
            theta_out_vel = 0.0
        else:
            qdot_target = (q_target - prev_q) / dt
            theta_out_vel = (theta_target - prev_theta) / dt

        self.q = q_target
        self.qdot = qdot_target
        self.theta_out = theta_target
        self.theta_out_vel = theta_out_vel
        self.last_theta_ref = theta_target
        self.last_ref_time = elapsed_time
        self._update_mappers()


# ==================================================================================
# Simulated time base
# ==================================================================================


class SimulatedTime:
    """
    Lightweight replacement for the ``time`` module used by ``norm_v2``.
    Advances deterministically without sleeping and stops the loop once the
    requested simulation horizon elapses.
    """

    def __init__(self, plant: MechanismPlant, duration: float) -> None:
        self._plant = plant
        self._duration = duration
        self._now = 0.0

    def time(self) -> float:
        return self._now

    def sleep(self, dt: float) -> None:
        if dt > 0.0:
            self._plant.step(dt)
            self._now += dt
        if self._now >= self._duration:
            raise KeyboardInterrupt("simulation_finished")


# ==================================================================================
# Harness
# ==================================================================================


class SimulationHarness:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.plant = MechanismPlant()
        self.odrive_module = SimulatedODriveModule(self.plant)
        self.enums_module = _build_enums_module()
        self.control_module: Optional[types.ModuleType] = None
        self.original_modules = {
            "odrive": sys.modules.get("odrive"),
            "odrive.enums": sys.modules.get("odrive.enums"),
        }

    def __enter__(self) -> "SimulationHarness":
        sys.modules["odrive"] = self.odrive_module
        sys.modules["odrive.enums"] = self.enums_module
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        for key, module in self.original_modules.items():
            if module is None:
                sys.modules.pop(key, None)
            else:
                sys.modules[key] = module

    def load_control(self) -> None:
        script_dir = Path(__file__).resolve().parent
        if str(script_dir) not in sys.path:
            sys.path.insert(0, str(script_dir))

        import norm_v2 as control

        self.control_module = control
        self.plant.configure(control)

        serials = control.ODRIVE_SERIAL
        self.odrive_module.register(serials["motor0"], SimulatedDrive(self.plant, index=0))
        self.odrive_module.register(serials["motor1"], SimulatedDrive(self.plant, index=1))
        self.odrive_module.register(serials["output"], SimulatedDrive(self.plant, index=None))

    def patch_runtime(self) -> None:
        assert self.control_module is not None
        control = self.control_module

        control.time = SimulatedTime(self.plant, duration=self.args.duration)
        control.DATA_FILENAME_PREFIX = "norm2_sim"
        control.SAFETY_CONFIG['max_torque0'] = 100.0
        control.SAFETY_CONFIG['max_torque1'] = 100.0
        for cfg in control.MOTOR_PID.values():
            cfg['max_output'] = 100.0
        self.plant.set_ideal_mode(True)
        self.plant.set_first_order_tau(self.args.time_constant)

        original_generate = control.generate_output_step

        def generate_with_enforcement(elapsed_time: float) -> float:
            theta_ref = original_generate(elapsed_time)
            self.plant.enforce_output(theta_ref, elapsed_time)
            return theta_ref

        control.generate_output_step = generate_with_enforcement

        choice_lookup = {"all": "1", "fig": "2", "csv": "3", "none": "4"}
        default_choice = choice_lookup[self.args.keep]

        def auto_input(prompt: str = "") -> str:
            if not self.args.quiet and prompt:
                print(prompt + default_choice)
            return default_choice

        control.input = auto_input
        control.plt.show = lambda *_, **__: None

        if self.args.seed is not None:
            np.random.seed(self.args.seed)

    def run(self) -> None:
        assert self.control_module is not None
        control = self.control_module
        try:
            control.main()
        except KeyboardInterrupt as exc:
            if str(exc) != "simulation_finished":
                raise


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Offline simulator for norm_v2.")
    parser.add_argument("--duration", type=float, default=12.0, help="Simulated seconds before auto-stop.")
    parser.add_argument("--keep", choices=["all", "fig", "csv", "none"], default="all", help="What to keep after plotting.")
    parser.add_argument("--seed", type=int, default=42, help="Random seed for measurement noise.")
    parser.add_argument("--quiet", action="store_true", help="Less verbose auto-response output.")
    parser.add_argument("--time-constant", type=float, default=0.05, help="1st-order lag time constant for ideal output enforcement [s].")
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> None:
    args = parse_args(argv)
    with SimulationHarness(args) as harness:
        harness.load_control()
        harness.patch_runtime()
        harness.run()


if __name__ == "__main__":
    main()
