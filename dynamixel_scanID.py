#!/usr/bin/env python3
"""Scan and control two Dynamixel actuators.

Usage examples:
  python pid_dynamixel.py --port /dev/ttyACM0 --baudrate 57600
  python pid_dynamixel.py --port /dev/ttyACM0 --ids 1 2 --goal-pos 1800 2400
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

DXL_MIN_ID = 1
DXL_MAX_ID = 252


@dataclass
class ControllerConfig:
	port: str
	baudrate: int
	protocol_version: float = 2.0


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

	def scan(self, start_id: int = DXL_MIN_ID, end_id: int = DXL_MAX_ID) -> list[int]:
		found: list[int] = []
		for dxl_id in range(start_id, end_id + 1):
			if self.ping(dxl_id):
				found.append(dxl_id)
		return found

	def set_position_mode(self, ids: list[int]) -> None:
		for dxl_id in ids:
			self._write1(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
			self._write1(dxl_id, ADDR_OPERATING_MODE, OPERATING_MODE_POSITION)
			self._write1(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)


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


def parse_args() -> argparse.Namespace:
	parser = argparse.ArgumentParser(description="Scan and control 2 Dynamixel actuators")
	parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port, e.g. /dev/ttyACM0")
	parser.add_argument("--baudrate", type=int, default=57600, help="Dynamixel bus baudrate")
	parser.add_argument(
		"--ids",
		nargs=2,
		type=int,
		metavar=("ID1", "ID2"),
		help="Two actuator IDs. If omitted, the script scans and picks first two IDs.",
	)
	
	parser.add_argument("--scan-start", type=int, default=DXL_MIN_ID, help="Scan start ID")
	parser.add_argument("--scan-end", type=int, default=DXL_MAX_ID, help="Scan end ID")
	return parser.parse_args()


def main() -> int:
	args = parse_args()
	config = ControllerConfig(port=args.port, baudrate=args.baudrate)
	controller = DynamixelController(config)

	selected_ids: list[int] = []
	try:
		print(f"[INFO] Connecting to {args.port} @ {args.baudrate}...")
		controller.connect()

		if args.ids:
			selected_ids = list(args.ids)
			missing = [dxl_id for dxl_id in selected_ids if not controller.ping(dxl_id)]
			if missing:
				raise RuntimeError(f"IDs not found/responding: {missing}")
		else:
			print(f"[INFO] Scanning IDs {args.scan_start}..{args.scan_end}...")
			found = controller.scan(args.scan_start, args.scan_end)
			print(f"[INFO] Found IDs: {found}")
			if len(found) < 2:
				raise RuntimeError("At least two actuators are required, but fewer than 2 were found.")
			selected_ids = found[:2]

		print(f"[INFO] Using IDs: {selected_ids}")
		controller.set_position_mode(selected_ids)


		time.sleep(0.3)
		positions = controller.read_present_positions(selected_ids)
		print(f"[INFO] Present positions: {positions}")
		return 0
	except Exception as exc:  # noqa: BLE001
		print(f"[ERROR] {exc}", file=sys.stderr)
		return 1
	finally:
		controller.close(selected_ids)
		print("[INFO] Port closed.")


if __name__ == "__main__":
	raise SystemExit(main())
