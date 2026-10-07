#!/usr/bin/env python3
"""Clean Unitree SDK2 Python SportClient UDP worker.

Run on the Go2 dock. This process intentionally does not import rclpy or source
ROS workspaces. It receives local UDP velocity packets from
go2_safe_cmd_vel_udp_bridge.py and calls:

    SportClient.Move(vx, 0.0, vyaw)

It does not call StopMove() or Damp(). Zero velocity is represented as
Move(0, 0, 0), which is the normal velocity-control stop command.
"""

import json
import math
import os
import select
import signal
import socket
import sys
import time
from typing import Optional, Tuple


REAL_SEND_CONFIRMATION = "I_UNDERSTAND_GO2_WILL_MOVE"


def env_bool(name: str, default: bool = False) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.lower() in {"1", "true", "yes", "on"}


def env_float(name: str, default: float) -> float:
    value = os.environ.get(name)
    if value is None or value == "":
        return default
    return float(value)


def env_int(name: str, default: int) -> int:
    value = os.environ.get(name)
    if value is None or value == "":
        return default
    return int(value)


def finite_or_zero(value: float) -> float:
    return value if math.isfinite(value) else 0.0


def clamp(value: float, limit: float) -> float:
    limit = abs(limit)
    return max(-limit, min(limit, value))


def log(message: str) -> None:
    print(f"[go2-unitree-worker] {message}", flush=True)


class Go2UnitreeSportUdpWorker:
    def __init__(self) -> None:
        self.iface = os.environ.get("UNITREE_IFACE", "eth0")
        self.host = os.environ.get("GO2_UDP_BIND_HOST", "127.0.0.1")
        self.port = env_int("GO2_UDP_BIND_PORT", 15001)
        self.max_linear_x = env_float("GO2_UNITREE_MAX_LINEAR_X", 0.10)
        self.max_angular_z = env_float("GO2_UNITREE_MAX_ANGULAR_Z", 0.30)
        self.input_timeout_s = env_float("GO2_UNITREE_INPUT_TIMEOUT_S", 0.30)
        self.publish_rate_hz = env_float("GO2_UNITREE_PUBLISH_RATE_HZ", 20.0)
        self.call_balance_stand_on_start = env_bool(
            "CALL_BALANCE_STAND_ON_START", False
        )
        self.call_classic_walk_on_start = env_bool(
            "CALL_CLASSIC_WALK_ON_START", False
        )
        self.ignore_classic_walk_start_failure = env_bool(
            "IGNORE_CLASSIC_WALK_START_FAILURE", True
        )
        self.classic_walk_off_on_shutdown = env_bool(
            "CLASSIC_WALK_OFF_ON_SHUTDOWN", True
        )
        self.switch_joystick_off_on_start = env_bool(
            "CALL_SWITCH_JOYSTICK_OFF_ON_START", False
        )
        self.restore_joystick_on_shutdown = env_bool(
            "RESTORE_JOYSTICK_ON_SHUTDOWN", True
        )
        self.real_send = env_bool("GO2_REAL_SEND", False)
        self.stop_after_sdk_error = env_bool("GO2_STOP_AFTER_FIRST_SDK_ERROR", True)
        self.send_zero_on_shutdown = env_bool("GO2_SEND_ZERO_ON_SHUTDOWN", False)
        self.joystick_switched_off = False
        self.classic_walk_enabled = False

        if self.publish_rate_hz <= 0:
            raise ValueError("GO2_UNITREE_PUBLISH_RATE_HZ must be > 0")
        if self.input_timeout_s <= 0:
            raise ValueError("GO2_UNITREE_INPUT_TIMEOUT_S must be > 0")
        if self.real_send:
            if os.environ.get("GO2_ALLOW_REAL_SEND") != REAL_SEND_CONFIRMATION:
                raise RuntimeError(
                    "GO2_REAL_SEND=1 requires "
                    f"GO2_ALLOW_REAL_SEND={REAL_SEND_CONFIRMATION}"
                )

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.host, self.port))
        self.sock.setblocking(False)

        self.client = None
        self.last_packet_time = None
        self.last_cmd = (0.0, 0.0)
        self.last_sent_cmd = None
        self.last_seq = None
        self.ever_received_packet = False
        self.ever_sent_nonzero = False
        self.ever_successful_nonzero = False

        if self.real_send:
            try:
                self._init_unitree_client()
            except Exception:
                self.shutdown()
                raise

        mode = "REAL" if self.real_send else "DRY-RUN"
        log(
            f"started mode={mode} bind={self.host}:{self.port} iface={self.iface} "
            f"max_x={self.max_linear_x:.3f} max_yaw={self.max_angular_z:.3f} "
            f"timeout={self.input_timeout_s:.3f}s "
            f"switch_joystick_off_on_start={self.switch_joystick_off_on_start} "
            f"balance_on_start={self.call_balance_stand_on_start} "
            f"classic_walk_on_start={self.call_classic_walk_on_start} "
            f"ignore_classic_walk_start_failure={self.ignore_classic_walk_start_failure} "
            f"stop_after_sdk_error={self.stop_after_sdk_error} "
            f"zero_on_shutdown={self.send_zero_on_shutdown}"
        )

    def _init_unitree_client(self) -> None:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.go2.sport.sport_client import SportClient

        log(f"ChannelFactoryInitialize(domain=0, iface={self.iface!r})")
        ChannelFactoryInitialize(0, self.iface)
        self.client = SportClient()
        self.client.SetTimeout(1.0)
        ret = self.client.Init()
        log(f"SportClient.Init() ret={ret}")

        if self.switch_joystick_off_on_start:
            if not hasattr(self.client, "SwitchJoystick"):
                raise RuntimeError(
                    "CALL_SWITCH_JOYSTICK_OFF_ON_START requested, "
                    "but SportClient has no SwitchJoystick method"
                )
            ret = self.client.SwitchJoystick(False)
            log(f"SwitchJoystick(False) ret={ret}")
            if ret != 0:
                raise RuntimeError(f"SwitchJoystick(False) failed with ret={ret}")
            self.joystick_switched_off = True

        if self.call_balance_stand_on_start:
            ret = self.client.BalanceStand()
            log(f"BalanceStand() ret={ret}")

        if self.call_classic_walk_on_start:
            if not hasattr(self.client, "ClassicWalk"):
                raise RuntimeError(
                    "CALL_CLASSIC_WALK_ON_START requested, "
                    "but SportClient has no ClassicWalk method"
                )
            ret = self.client.ClassicWalk(True)
            log(f"ClassicWalk(True) ret={ret}")
            if ret != 0:
                message = f"ClassicWalk(True) failed with ret={ret}"
                if self.ignore_classic_walk_start_failure:
                    log(message + "; continuing without marking classic walk enabled")
                    return
                raise RuntimeError(message)
            self.classic_walk_enabled = True

    def shutdown(self) -> None:
        if self.client is not None and self.classic_walk_off_on_shutdown and (
            self.classic_walk_enabled or self.call_classic_walk_on_start
        ):
            try:
                ret = self.client.ClassicWalk(False)
                log(f"ClassicWalk(False) ret={ret}")
            except Exception as exc:
                log(f"ClassicWalk(False) failed: {exc}")

        if (
            self.client is not None
            and self.joystick_switched_off
            and self.restore_joystick_on_shutdown
        ):
            try:
                ret = self.client.SwitchJoystick(True)
                log(f"SwitchJoystick(True) ret={ret}")
            except Exception as exc:
                log(f"SwitchJoystick(True) failed: {exc}")

    def recv_packets(self) -> None:
        while True:
            ready, _, _ = select.select([self.sock], [], [], 0.0)
            if not ready:
                return
            data, _ = self.sock.recvfrom(4096)
            packet = json.loads(data.decode("utf-8"))
            vx = clamp(finite_or_zero(float(packet.get("vx", 0.0))), self.max_linear_x)
            vyaw = clamp(
                finite_or_zero(float(packet.get("vyaw", 0.0))), self.max_angular_z
            )
            self.last_cmd = (vx, vyaw)
            self.last_packet_time = time.monotonic()
            self.last_seq = packet.get("seq")
            self.ever_received_packet = True

    def current_cmd(self) -> Optional[Tuple[float, float, bool]]:
        if self.last_packet_time is None:
            return None
        age = time.monotonic() - self.last_packet_time
        stale = age > self.input_timeout_s
        if stale:
            return 0.0, 0.0, True
        vx, vyaw = self.last_cmd
        return vx, vyaw, False

    def send_move(self, vx: float, vyaw: float, stale: bool) -> None:
        cmd = (round(vx, 4), round(vyaw, 4))
        if cmd == self.last_sent_cmd and stale:
            # Keep stale zero calls sparse. Fresh zero commands are still sent.
            return
        if cmd == (0.0, 0.0) and not self.ever_sent_nonzero:
            # Safety-first: do not touch the motion API with zero commands
            # before this worker has actually sent a nonzero velocity.
            return
        if stale and self.last_sent_cmd in (None, (0.0, 0.0)):
            # Do not call Unitree SDK just because the worker started with no
            # fresh input. A stale zero is only useful after a nonzero command.
            return

        if self.real_send:
            assert self.client is not None
            ret = self.client.Move(float(vx), 0.0, float(vyaw))
            sdk = f"Move ret={ret}"
        else:
            ret = 0
            sdk = "dry_run"

        self.last_sent_cmd = cmd
        if abs(vx) > 1e-6 or abs(vyaw) > 1e-6:
            self.ever_sent_nonzero = True
            if ret == 0:
                self.ever_successful_nonzero = True
        log(
            f"seq={self.last_seq} stale={stale} "
            f"Move(vx={vx:.3f}, vy=0.000, vyaw={vyaw:.3f}) {sdk}"
        )
        if self.real_send and ret != 0 and self.stop_after_sdk_error:
            raise RuntimeError(
                f"SportClient.Move failed with ret={ret}; "
                "stopping worker to avoid repeatedly sending rejected commands"
            )

    def run(self) -> None:
        period = 1.0 / self.publish_rate_hz
        while True:
            start = time.monotonic()
            self.recv_packets()
            cmd = self.current_cmd()
            if cmd is not None:
                vx, vyaw, stale = cmd
                self.send_move(vx, vyaw, stale)
            elapsed = time.monotonic() - start
            time.sleep(max(0.0, period - elapsed))


def main() -> None:
    def _handle_stop_signal(signum, _frame) -> None:
        raise KeyboardInterrupt(f"signal {signum}")

    signal.signal(signal.SIGTERM, _handle_stop_signal)
    signal.signal(signal.SIGINT, _handle_stop_signal)

    log(f"python={sys.executable}")
    log(f"PYTHONPATH={os.environ.get('PYTHONPATH', '<unset>')}")
    worker = None
    exit_code = 0
    try:
        worker = Go2UnitreeSportUdpWorker()
        worker.run()
    except RuntimeError as exc:
        log(str(exc))
        exit_code = 2
    except KeyboardInterrupt:
        if (
            worker.client is not None
            and worker.send_zero_on_shutdown
            and worker.ever_successful_nonzero
        ):
            log("interrupted; sending final Move(0,0,0)")
            try:
                ret = worker.client.Move(0.0, 0.0, 0.0)
                log(f"final Move(0,0,0) ret={ret}")
            except Exception as exc:
                log(f"final zero failed: {exc}")
        else:
            log(
                "interrupted; not sending final Move(0,0,0). "
                "Set GO2_SEND_ZERO_ON_SHUTDOWN=1 to enable it after a "
                "successful nonzero Move."
            )
    finally:
        if worker is not None:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
            worker.shutdown()
    if exit_code != 0:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
