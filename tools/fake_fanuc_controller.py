#!/usr/bin/env python3
"""Fake FANUC R30iB Mini Plus for testing fanuc_hw without a robot.

Speaks just enough EtherNet/IP (explicit messaging over TCP 44818) for
fanuc_eth_ip: RegisterSession, SendRRData with Get/Set_Attribute_Single on

  0x7E/1/1  current joint position  (10 REALs: [0, J1..J6, E1..E3], degrees, FANUC J2/J3 convention)
  0x7D/1/1  current cartesian pose   (10 REALs; zeros, not emulated)
  0x6B/1/n  R[n]                     (DINT; R[1] = MotionStatus, 1 = ROS)
  0x7C/1/n  PR[n] joint              (10 REALs, same layout as 0x7E)
  0x04/*/3  assemblies (DPM)         (accepted, ignored)

and stands in for ROS2.TP: while R[1] == 1 the arm tracks PR[1] at a capped
joint speed. Every PR[1] write is checked: the step from the current position
and the speed that step implies are logged, so a test run shows whether
ros2_control ever asks the real controller for a jump.

  python3 fake_fanuc_controller.py --host 127.0.0.1
  ros2 launch fanuc_control robot_bringup.launch.py robot_type:=crx10ia_l robot_ip:=127.0.0.1
"""
import argparse
import socket
import socketserver
import struct
import threading
import time

ENCAP = struct.Struct("<HHII8sI")  # command, length, session, status, context, options
REGISTER_SESSION, UNREGISTER_SESSION, SEND_RR_DATA = 0x65, 0x66, 0x6F
GET_ATTRIBUTE_SINGLE, SET_ATTRIBUTE_SINGLE = 0x0E, 0x10
OK, PATH_UNKNOWN, NOT_SUPPORTED, NOT_ENOUGH_DATA = 0x00, 0x05, 0x08, 0x13


class Controller:
    def __init__(self, joints_deg, speed_deg_s, step_warn_deg):
        self.lock = threading.Lock()
        self.joints = list(joints_deg)  # FANUC convention, degrees
        self.target = list(joints_deg)
        self.registers = {}
        self.speed = speed_deg_s
        self.step_warn = step_warn_deg
        self.pr_writes = 0
        self.max_step = 0.0
        self.big_steps = 0
        self.last_pr_time = None
        self.max_rate = 0.0

    def motion_loop(self, period=0.004):
        while True:
            time.sleep(period)
            with self.lock:
                if self.registers.get(1, 0) != 1:  # ROS2.TP only moves in ROS mode
                    continue
                cap = self.speed * period
                self.joints = [j + max(-cap, min(cap, t - j)) for j, t in zip(self.joints, self.target)]

    def read_joints(self):
        with self.lock:
            return [0.0, *self.joints, 0.0, 0.0, 0.0]

    def write_pr(self, values):
        now = time.monotonic()
        with self.lock:
            target = values[1:7]
            step = max(abs(t - j) for t, j in zip(target, self.joints))
            self.pr_writes += 1
            self.max_step = max(self.max_step, step)
            if self.last_pr_time is not None and now > self.last_pr_time:
                change = max(abs(a - b) for a, b in zip(target, self.target))
                self.max_rate = max(self.max_rate, change / (now - self.last_pr_time))
            self.last_pr_time = now
            if step > self.step_warn:
                self.big_steps += 1
                print(f"WARN PR[1] {step:.2f} deg away from current position "
                      f"(target {[round(t, 2) for t in target]})", flush=True)
            self.target = list(target)

    def status_line(self):
        with self.lock:
            return (f"R[1]={self.registers.get(1, 0)} joints={[round(j, 2) for j in self.joints]} "
                    f"PR writes={self.pr_writes} max step={self.max_step:.2f} deg "
                    f"max PR rate={self.max_rate:.1f} deg/s big steps={self.big_steps}")


def parse_path(path):
    """CIP logical segments -> {'class': c, 'instance': i, 'attribute': a}."""
    out, i = {}, 0
    names = {0x20: "class", 0x24: "instance", 0x30: "attribute"}
    while i < len(path):
        seg = path[i]
        kind, fmt = seg & 0xFC, seg & 0x03
        if kind not in names:
            raise ValueError(f"unsupported path segment 0x{seg:02x}")
        if fmt == 0:
            out[names[kind]] = path[i + 1]
            i += 2
        elif fmt == 1:
            out[names[kind]] = struct.unpack_from("<H", path, i + 2)[0]
            i += 4
        else:
            raise ValueError(f"unsupported segment format 0x{seg:02x}")
    return out


def handle_cip(ctrl, request):
    service, words = request[0], request[1]
    path, data = request[2:2 + 2 * words], request[2 + 2 * words:]
    try:
        p = parse_path(path)
    except ValueError:
        return service, PATH_UNKNOWN, b""
    cls, inst, attr = p.get("class"), p.get("instance"), p.get("attribute")
    if service == GET_ATTRIBUTE_SINGLE:
        if (cls, inst, attr) == (0x7E, 1, 1):
            return service, OK, struct.pack("<10f", *ctrl.read_joints())
        if (cls, inst, attr) == (0x7D, 1, 1):
            return service, OK, struct.pack("<10f", *([0.0] * 10))
        if cls == 0x6B and inst == 1:
            with ctrl.lock:
                return service, OK, struct.pack("<i", ctrl.registers.get(attr, 0))
        return service, PATH_UNKNOWN, b""
    if service == SET_ATTRIBUTE_SINGLE:
        if cls == 0x6B and inst == 1:
            if len(data) < 4:
                return service, NOT_ENOUGH_DATA, b""
            value = struct.unpack_from("<i", data)[0]
            with ctrl.lock:
                if ctrl.registers.get(attr) != value:
                    print(f"R[{attr}] = {value}", flush=True)
                ctrl.registers[attr] = value
            return service, OK, b""
        if cls == 0x7C and inst == 1:
            if len(data) < 40:
                return service, NOT_ENOUGH_DATA, b""
            values = struct.unpack_from("<10f", data)
            if attr == 1:
                ctrl.write_pr(values)
            return service, OK, b""
        if cls == 0x04:
            return service, OK, b""
        return service, PATH_UNKNOWN, b""
    return service, NOT_SUPPORTED, b""


def make_handler(ctrl):
    class Handler(socketserver.BaseRequestHandler):
        def recv_exact(self, n):
            buf = b""
            while len(buf) < n:
                chunk = self.request.recv(n - len(buf))
                if not chunk:
                    raise ConnectionError
                buf += chunk
            return buf

        def reply(self, command, session, context, body, status=0):
            self.request.sendall(ENCAP.pack(command, len(body), session, status, context, 0) + body)

        def handle(self):
            print(f"session from {self.client_address[0]}", flush=True)
            session = 0
            try:
                while True:
                    command, length, sess, _, context, _ = ENCAP.unpack(self.recv_exact(ENCAP.size))
                    body = self.recv_exact(length) if length else b""
                    if command == REGISTER_SESSION:
                        session = 0x1000 + threading.get_ident() % 0xFFFF
                        self.reply(command, session, context, body[:4] or struct.pack("<HH", 1, 0))
                    elif command == UNREGISTER_SESSION:
                        return
                    elif command == SEND_RR_DATA:
                        count = struct.unpack_from("<H", body, 6)[0]
                        offset, request = 8, None
                        for _ in range(count):
                            item_type, item_len = struct.unpack_from("<HH", body, offset)
                            if item_type == 0x00B2:
                                request = body[offset + 4:offset + 4 + item_len]
                            offset += 4 + item_len
                        service, status, data = handle_cip(ctrl, request)
                        cip = struct.pack("<BBBB", service | 0x80, 0, status, 0) + data
                        items = struct.pack("<IHH", 0, 0, 2) + struct.pack("<HH", 0, 0) \
                            + struct.pack("<HH", 0x00B2, len(cip)) + cip
                        self.reply(command, sess, context, items)
                    else:
                        self.reply(command, sess, context, b"", status=0x01)  # invalid command
            except (ConnectionError, OSError):
                pass
            finally:
                print(f"session from {self.client_address[0]} closed", flush=True)

    return Handler


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=44818)
    ap.add_argument("--joints", default="0,0,0,0,0,0", help="start joints, degrees (FANUC convention)")
    ap.add_argument("--speed", type=float, default=60.0, help="tracking speed cap, deg/s")
    ap.add_argument("--step-warn", type=float, default=5.0, help="warn when PR[1] is this far (deg) from current")
    ap.add_argument("--status-period", type=float, default=2.0)
    args = ap.parse_args()

    ctrl = Controller([float(v) for v in args.joints.split(",")], args.speed, args.step_warn)
    threading.Thread(target=ctrl.motion_loop, daemon=True).start()
    socketserver.ThreadingTCPServer.allow_reuse_address = True
    server = socketserver.ThreadingTCPServer((args.host, args.port), make_handler(ctrl))
    server.daemon_threads = True
    server.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"fake FANUC controller on {args.host}:{args.port}", flush=True)
    try:
        while True:
            time.sleep(args.status_period)
            print(ctrl.status_line(), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()


if __name__ == "__main__":
    main()
