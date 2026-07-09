#!/usr/bin/env python3
"""PID-only control for two Dynamixel actuators (pan/tilt).

This module expects a face center (x, y) in pixel coordinates.
The PID error is calculated from image center to face center:
- pan (ID 1 by default): x-axis error
- tilt (ID 2 by default): y-axis error
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass

import dynamixel_sdk as dyn


# Protocol 2.0 register map (X-series default)
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

LEN_GOAL_POSITION = 4
LEN_PRESENT_POSITION = 4

OPERATING_MODE_POSITION = 3
TORQUE_DISABLE = 0
TORQUE_ENABLE = 1

PAN_ID = 2
TILT_ID = 1
MIN_POS = 0
MAX_POS = 4095
DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUDRATE = 57600


@dataclass
class ControllerConfig:
	port: str
	baudrate: int
	protocol_version: float = 2.0


@dataclass
class PIDAxisConfig:
	kp: float
	ki: float
	kd: float
	integral_limit: float = 500.0


PAN_PID = PIDAxisConfig(kp=0.08, ki=0.001, kd=0.02)
TILT_PID = PIDAxisConfig(kp=0.08, ki=0.001, kd=0.02)


class DynamixelController:
	def __init__(self, config: ControllerConfig) -> None:
		self.config = config
		self.port_handler = dyn.PortHandler(config.port)
		self.packet_handler = dyn.PacketHandler(config.protocol_version)
		self.connected = False

	def connect(self) -> None:
		if not self.port_handler.openPort():
			raise RuntimeError(f"Failed to open port: {self.config.port}")
		if not self.port_handler.setBaudRate(self.config.baudrate):
			raise RuntimeError(f"Failed to set baudrate: {self.config.baudrate}")
		self.connected = True

	def close(self, ids: list[int] | None = None) -> None:
		if ids:
			for dxl_id in ids:
				self._write1(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, quiet=True)
		if self.connected:
			self.port_handler.closePort()
			self.connected = False

	def ping(self, dxl_id: int) -> bool:
		_, comm_result, error = self.packet_handler.ping(self.port_handler, dxl_id)
		return comm_result == dyn.COMM_SUCCESS and error == 0

	def set_position_mode(self, ids: list[int]) -> None:
		for dxl_id in ids:
			self._write1(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
			self._write1(dxl_id, ADDR_OPERATING_MODE, OPERATING_MODE_POSITION)
			self._write1(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

	def set_goal_positions(self, goal_by_id: dict[int, int]) -> None:
		sync_write = dyn.GroupSyncWrite(
			self.port_handler,
			self.packet_handler,
			ADDR_GOAL_POSITION,
			LEN_GOAL_POSITION,
		)
		for dxl_id, position in goal_by_id.items():
			value = int(position)
			param = [
				dyn.DXL_LOBYTE(dyn.DXL_LOWORD(value)),
				dyn.DXL_HIBYTE(dyn.DXL_LOWORD(value)),
				dyn.DXL_LOBYTE(dyn.DXL_HIWORD(value)),
				dyn.DXL_HIBYTE(dyn.DXL_HIWORD(value)),
			]
			if not sync_write.addParam(dxl_id, param):
				raise RuntimeError(f"Failed to add sync write param for ID {dxl_id}")

		comm_result = sync_write.txPacket()
		if comm_result != dyn.COMM_SUCCESS:
			msg = self.packet_handler.getTxRxResult(comm_result)
			raise RuntimeError(f"SyncWrite failed: {msg}")
		sync_write.clearParam()

	def read_present_positions(self, ids: list[int]) -> dict[int, int]:
		sync_read = dyn.GroupSyncRead(
			self.port_handler,
			self.packet_handler,
			ADDR_PRESENT_POSITION,
			LEN_PRESENT_POSITION,
		)
		for dxl_id in ids:
			if not sync_read.addParam(dxl_id):
				raise RuntimeError(f"Failed to add sync read param for ID {dxl_id}")

		comm_result = sync_read.txRxPacket()
		if comm_result != dyn.COMM_SUCCESS:
			msg = self.packet_handler.getTxRxResult(comm_result)
			raise RuntimeError(f"SyncRead failed: {msg}")

		positions: dict[int, int] = {}
		for dxl_id in ids:
			if not sync_read.isAvailable(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
				raise RuntimeError(f"Present position is not available for ID {dxl_id}")
			positions[dxl_id] = sync_read.getData(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

		sync_read.clearParam()
		return positions

	def _write1(self, dxl_id: int, address: int, value: int, quiet: bool = False) -> None:
		comm_result, error = self.packet_handler.write1ByteTxRx(
			self.port_handler,
			dxl_id,
			address,
			value,
		)
		if quiet:
			return
		if comm_result != dyn.COMM_SUCCESS:
			msg = self.packet_handler.getTxRxResult(comm_result)
			raise RuntimeError(f"ID {dxl_id} write failed at addr {address}: {msg}")
		if error != 0:
			msg = self.packet_handler.getRxPacketError(error)
			raise RuntimeError(f"ID {dxl_id} packet error at addr {address}: {msg}")


class PIDControl:
	"""Pan/Tilt PID controller using face center error."""

	def __init__(
		self,
		deadband_px: int = 5,
		port: str = DEFAULT_PORT,
		baudrate: int = DEFAULT_BAUDRATE,
		min_pos: int = MIN_POS,
		max_pos: int = MAX_POS,
	) -> None:
		self.controller = DynamixelController(ControllerConfig(port=port, baudrate=baudrate))
		self.pan_id = PAN_ID
		self.tilt_id = TILT_ID
		self.pan_cfg = PAN_PID
		self.tilt_cfg = TILT_PID
		self.min_pos = min_pos
		self.max_pos = max_pos
		self.deadband_px = deadband_px

		self.goal_pan = 2048
		self.goal_tilt = 2048

		self._i_pan = 0.0
		self._i_tilt = 0.0
		self._prev_e_pan = 0.0
		self._prev_e_tilt = 0.0
		self._prev_t: float | None = None

	def initialize(self) -> None:
		self.controller.connect()
		self.controller.set_position_mode([self.pan_id, self.tilt_id])
		positions = self.controller.read_present_positions([self.pan_id, self.tilt_id])
		self.goal_pan = self._clamp(positions[self.pan_id])
		self.goal_tilt = self._clamp(positions[self.tilt_id])

	def reset(self) -> None:
		self._i_pan = 0.0
		self._i_tilt = 0.0
		self._prev_e_pan = 0.0
		self._prev_e_tilt = 0.0
		self._prev_t = None

	def close(self) -> None:
		self.controller.close([self.pan_id, self.tilt_id])

	def update(
		self,
		frame_width: int,
		frame_height: int,
		face_center_x: int,
		face_center_y: int,
		now: float | None = None,
	) -> dict[int, int]:
		frame_cx = frame_width * 0.5
		frame_cy = frame_height * 0.5

		e_pan = frame_cx - face_center_x
		e_tilt = frame_cy - face_center_y

		if abs(e_pan) < self.deadband_px:
			e_pan = 0.0
		if abs(e_tilt) < self.deadband_px:
			e_tilt = 0.0

		if now is None:
			now = time.monotonic()
		dt = 0.0 if self._prev_t is None else max(1e-3, now - self._prev_t)
		self._prev_t = now

		u_pan = self._pid_step(
			error=e_pan,
			prev_error=self._prev_e_pan,
			integral=self._i_pan,
			dt=dt,
			cfg=self.pan_cfg,
		)
		u_tilt = self._pid_step(
			error=e_tilt,
			prev_error=self._prev_e_tilt,
			integral=self._i_tilt,
			dt=dt,
			cfg=self.tilt_cfg,
		)

		self._i_pan = u_pan[1]
		self._i_tilt = u_tilt[1]
		self._prev_e_pan = e_pan
		self._prev_e_tilt = e_tilt

		# Pan maps to ID 2, tilt maps to ID 1.
		self.goal_pan = self._clamp(self.goal_pan + int(round(u_pan[0])))
		self.goal_tilt = self._clamp(self.goal_tilt - int(round(u_tilt[0])))

		goal_by_id = {self.pan_id: self.goal_pan, self.tilt_id: self.goal_tilt}
		self.controller.set_goal_positions(goal_by_id)
		return goal_by_id

	def _pid_step(
		self,
		error: float,
		prev_error: float,
		integral: float,
		dt: float,
		cfg: PIDAxisConfig,
	) -> tuple[float, float]:
		new_integral = integral + error * dt
		new_integral = max(-cfg.integral_limit, min(cfg.integral_limit, new_integral))
		derivative = 0.0 if dt <= 0.0 else (error - prev_error) / dt
		output = (cfg.kp * error) + (cfg.ki * new_integral) + (cfg.kd * derivative)
		return output, new_integral

	def _clamp(self, value: int) -> int:
		return max(self.min_pos, min(self.max_pos, value))


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(description="PID control for 2 Dynamixel actuators using face bbox")
	parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port, e.g. /dev/ttyACM0")
	parser.add_argument("--baudrate", type=int, default=57600, help="Dynamixel bus baudrate")
	parser.add_argument("--deadband-px", type=int, default=5, help="Ignore small center error under this pixel size")

	parser.add_argument(
		"--frame-size",
		nargs=2,
		type=int,
		default=[640, 480],
		metavar=("WIDTH", "HEIGHT"),
		help="Frame size in pixels",
	)
	parser.add_argument(
		"--face-center",
		nargs=2,
		type=float,
		metavar=("CX", "CY"),
		help="Single face center input (cx cy). Useful for one-shot test.",
	)
	return parser.parse_args()


def main() -> int:
	args = parse_args()
	selected_ids: list[int] = [PAN_ID, TILT_ID]
	pid: PIDControl | None = None
	try:
		pid = PIDControl(deadband_px=args.deadband_px, port=args.port, baudrate=args.baudrate)
		print(f"[INFO] Connecting to {args.port} @ {args.baudrate}...")
		pid.initialize()
		missing = [dxl_id for dxl_id in selected_ids if not pid.controller.ping(dxl_id)]
		if missing:
			raise RuntimeError(f"IDs not found/responding: {missing}")

		if not args.face_center:
			print("[WARN] --face-center is not provided. No PID update was applied.")
			print("[HINT] Example: --frame-size 640 480 --face-center 290 220")
			return 0

		frame_w, frame_h = args.frame_size
		face_cx, face_cy = args.face_center
		goal_by_id = pid.update(frame_w, frame_h, face_cx, face_cy)
		print(f"[INFO] PID updated goals: {goal_by_id}")
		time.sleep(0.1)
		positions = pid.controller.read_present_positions(selected_ids)
		print(f"[INFO] Present positions: {positions}")
		return 0
	except Exception as exc:  # noqa: BLE001
		print(f"[ERROR] {exc}", file=sys.stderr)
		return 1
	finally:
		if pid is not None:
			pid.controller.close(selected_ids)
		print("[INFO] Port closed.")


if __name__ == "__main__":
	raise SystemExit(main())
