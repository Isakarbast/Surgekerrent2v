#!/usr/bin/env python3
"""
sk2v_cli.py - talk to Surgekurrent2v firmware over the UART protocol.

    pip install pyserial
    ./sk2v_cli.py -p /dev/ttyUSB0 ping
    ./sk2v_cli.py -p /dev/ttyUSB0 telemetry --watch
    ./sk2v_cli.py -p /dev/ttyUSB0 detect        # WHEEL OFF THE GROUND
    ./sk2v_cli.py -p /dev/ttyUSB0 faults
    ./sk2v_cli.py -p /dev/ttyUSB0 clear
    ./sk2v_cli.py -p /dev/ttyUSB0 pas 3
    ./sk2v_cli.py -p /dev/ttyUSB0 profile race        # eco | sport | race | custom
    ./sk2v_cli.py -p /dev/ttyUSB0 profile-info
    ./sk2v_cli.py -p /dev/ttyUSB0 unlock --key-file secrets/unlock_key.hex
    ./sk2v_cli.py -p /dev/ttyUSB0 lock

Frame: 0xAA | len | cmd | payload | crc16_hi | crc16_lo  (see src/comms/uart_protocol.h)
"""
import argparse
import hashlib
import hmac
import math
import pathlib
import struct
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial missing: pip install pyserial")

SOF = 0xAA
CMD = dict(PING=0x01, GET_TELEMETRY=0x02, GET_CONFIG=0x03, SET_CONFIG=0x04, COMMIT_CONFIG=0x05,
           SAVE_CONFIG=0x06, LOAD_DEFAULTS=0x07, CLEAR_FAULTS=0x08, START_DETECTION=0x09,
           DETECT_STATUS=0x0A, APPLY_DETECTION=0x0B, GET_FAULT_LOG=0x0C, SET_PAS_LEVEL=0x0D,
           REBOOT=0x0E, PROFILE_INFO=0x10, SET_PROFILE=0x11, UNLOCK_CHALLENGE=0x12,
           UNLOCK_RESPONSE=0x13, LOCK_ADVANCED=0x14, SET_BOOT_PROFILE=0x15, NACK=0x7F)
NACK_REASONS = {1: "unknown command", 2: "bad length", 3: "motor not idle", 4: "bad crc",
                5: "flash write failed", 6: "invalid", 7: "release throttle first",
                8: "advanced mode locked", 9: "outside safety envelope", 10: "protected factory field",
                11: "authentication failed", 12: "vehicle must be stopped"}
PROFILES = ["eco", "sport", "race", "custom"]
# Must match env_violation_t in src/safety/safety_envelope.h
ENV_VIOLATIONS = ["ok", "not_finite", "motor_current", "regen_current", "battery_current",
                  "battery_regen", "abs_trip", "abs_trip_margin", "erpm", "ramp", "field_weakening",
                  "speed_limit", "throttle_expo", "modulation", "current_bandwidth", "controller_gains",
                  "motor_params", "pas_current", "invalid_enum"]
UNLOCK_ERRORS = ["ok", "no challenge / already used", "challenge expired", "bad signature",
                 "terms version outdated", "locked out (too many attempts)", "flash write failed"]
UNLOCK_DOMAIN = b"SK2V-UNLOCK-v1"


def unlock_mac(key: bytes, nonce: bytes, uid: bytes, terms_version: int) -> bytes:
    """Reference implementation for the app: HMAC-SHA256 over domain|nonce|uid|terms(le16)."""
    return hmac.new(key, UNLOCK_DOMAIN + nonce + uid + struct.pack("<H", terms_version),
                    hashlib.sha256).digest()
# Must match fault_id_t order in src/safety/fault_manager.h
FAULTS = ["none", "overcurrent_sw", "overcurrent_hw", "overvoltage", "undervoltage",
          "fet_overtemp", "motor_overtemp", "gate_driver", "current_sense", "hall", "encoder",
          "throttle", "observer", "stall", "watchdog_reset", "loop_overrun", "config",
          "temp_sensor", "detection", "numeric"]
MC_STATES = ["calibrating", "idle", "run", "detect", "fault"]
ANGLE_SRC = ["none", "hall", "observer", "encoder"]
DET_STATES = ["idle", "align", "resist_1", "resist_2", "ind_d", "ind_q", "enc_offset", "hall_fwd",
              "hall_rev", "flux_ramp", "flux_measure", "flux_ramp_down", "done", "error"]
DET_ERRORS = ["none", "R out of range", "L out of range", "hall codes invalid",
              "flux out of range", "aborted"]

TELEMETRY_FMT = "<I4BI15fHBB"
TELEMETRY_FIELDS = ["time_ms", "state", "angle_src", "pas_level", "flags", "faults", "vbus",
                    "ibatt", "id", "iq", "iq_target", "erpm", "speed_kmh", "modulation",
                    "temp_fet", "temp_motor", "throttle", "odometer_km", "trip_wh",
                    "iq_limit_drive", "iq_limit_regen", "loop_us_max", "hall_code", "profile"]


def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def frame(cmd: int, payload: bytes = b"") -> bytes:
    body = bytes([len(payload) + 1, cmd]) + payload
    c = crc16(body)
    return bytes([SOF]) + body + bytes([c >> 8, c & 0xFF])


class Link:
    def __init__(self, port, baud=115200):
        self.s = serial.Serial(port, baud, timeout=0.3)

    def request(self, name, payload=b""):
        cmd = CMD[name]
        self.s.reset_input_buffer()
        self.s.write(frame(cmd, payload))
        deadline = time.time() + 1.0
        while time.time() < deadline:
            b = self.s.read(1)
            if not b or b[0] != SOF:
                continue
            ln = self.s.read(1)[0]
            body = self.s.read(ln)
            crc = self.s.read(2)
            if len(body) != ln or len(crc) != 2 or crc16(bytes([ln]) + body) != (crc[0] << 8 | crc[1]):
                continue
            rcmd, data = body[0], body[1:]
            if rcmd == CMD["NACK"]:
                why, detail = data[1], (data[2] if len(data) > 2 else 0)
                msg = NACK_REASONS.get(why, why)
                if why == 9:
                    msg += f" ({ENV_VIOLATIONS[detail] if detail < len(ENV_VIOLATIONS) else detail})"
                elif why == 11:
                    msg += f" ({UNLOCK_ERRORS[detail] if detail < len(UNLOCK_ERRORS) else detail})"
                raise RuntimeError(f"refused: {msg}")
            if rcmd == (cmd | 0x80):
                return data
        raise TimeoutError("no reply")


def fault_list(mask):
    return [FAULTS[i] for i in range(1, len(FAULTS)) if mask & (1 << i)] or ["none"]


def telemetry(link):
    raw = link.request("GET_TELEMETRY")
    return dict(zip(TELEMETRY_FIELDS, struct.unpack(TELEMETRY_FMT, raw[:struct.calcsize(TELEMETRY_FMT)])))


def cmd_telemetry(link, args):
    while True:
        t = telemetry(link)
        print(f"[{PROFILES[t['profile']] if t['profile'] < 4 else '?'}] {MC_STATES[t['state']]:>11} src={ANGLE_SRC[t['angle_src']]:<8} "
              f"Vbus={t['vbus']:5.1f}V Ibat={t['ibatt']:6.1f}A Iq={t['iq']:6.1f}/{t['iq_target']:6.1f}A "
              f"Id={t['id']:6.1f}A {t['speed_kmh']:5.1f}km/h eRPM={t['erpm']:7.0f} mod={t['modulation']:.2f} "
              f"Tfet={t['temp_fet']:4.0f}C Tmot={t['temp_motor']:4.0f}C thr={t['throttle']:.2f} "
              f"lim={t['iq_limit_drive']:.0f}/{t['iq_limit_regen']:.0f}A loop={t['loop_us_max']}us "
              f"hall={t['hall_code']} faults={','.join(fault_list(t['faults']))}")
        if not args.watch:
            break
        time.sleep(0.2)


def cmd_detect(link, args):
    print("!! Motor detection spins the motor. Wheel must be OFF THE GROUND. !!")
    if not args.yes and input("Type 'yes' to continue: ").strip() != "yes":
        return
    link.request("START_DETECTION")
    while True:
        time.sleep(0.25)
        st, err, rs, ld, lq, flux, enc, *hall = struct.unpack("<BB5f8f", link.request("DETECT_STATUS"))
        print(f"\r  {DET_STATES[st]:<16}", end="", flush=True)
        if DET_STATES[st] in ("done", "error"):
            print()
            break
    if DET_STATES[st] == "error":
        sys.exit(f"Detection failed: {DET_ERRORS[err]}")
    print(f"  R  = {rs * 1000:.2f} mOhm\n  Ld = {ld * 1e6:.1f} uH\n  Lq = {lq * 1e6:.1f} uH\n"
          f"  flux = {flux * 1000:.3f} mWb")
    print("  hall table: " + " ".join(f"{c}:{math.degrees(a):.0f}" for c, a in enumerate(hall) if a >= 0))
    link.request("APPLY_DETECTION")
    link.request("SAVE_CONFIG")
    print("Applied and saved.")


def cmd_faults(link, args):
    t = telemetry(link)
    print("active:", ", ".join(fault_list(t["faults"])))
    data = link.request("GET_FAULT_LOG")
    n = data[0]
    entry = "<IB3x4f"
    size = struct.calcsize(entry)
    for i in range(n):
        tms, fid, vbus, cur, erpm, tfet = struct.unpack(entry, data[1 + i * size:1 + (i + 1) * size])
        print(f"  t={tms / 1000:9.3f}s {FAULTS[fid]:<16} Vbus={vbus:5.1f} Ipk={cur:6.1f} eRPM={erpm:7.0f} Tfet={tfet:4.0f}")


def cmd_profile_info(link, args):
    d = link.request("PROFILE_INFO")
    active, boot, unlocked, dev_key, t_acc, t_req = struct.unpack_from("<4B2H", d)
    env = struct.unpack_from("<9f", d, 8)
    par = struct.unpack_from("<10f", d, 8 + 36)
    print(f"profile: {PROFILES[active]}   boot: {PROFILES[boot]}   advanced: "
          f"{'UNLOCKED' if unlocked else 'locked'} (terms {t_acc}/{t_req})")
    if dev_key:
        print("WARNING: firmware uses the PUBLIC development unlock key")
    names = ["phase A", "abs trip A", "battery A", "battery regen A", "field weak. A", "eRPM",
             "ramp A/s", "speed km/h", "current BW Hz"]
    print("safety envelope (hard ceilings): " + ", ".join(f"{n} {v:g}" for n, v in zip(names, env)))
    pn = ["motor A", "regen A", "battery A", "battery regen A", "trip A", "eRPM", "ramp A/s",
          "field weak. A", "speed limit km/h", "throttle expo"]
    print("active: " + ", ".join(f"{n} {v:g}" for n, v in zip(pn, par)))


def cmd_unlock(link, args):
    key = bytes.fromhex(pathlib.Path(args.key_file).read_text().strip())
    print("Advanced mode removes the preset limits (still inside the hardware safety envelope).")
    print("The rider must accept your terms in the app. This CLI is for development only.")
    if not args.yes and input("Type 'I agree' to continue: ").strip() != "I agree":
        return
    ch = link.request("UNLOCK_CHALLENGE")
    nonce, uid, terms_required = ch[:16], ch[16:28], struct.unpack("<H", ch[28:30])[0]
    terms = args.terms if args.terms is not None else terms_required
    link.request("UNLOCK_RESPONSE", struct.pack("<H", terms) + unlock_mac(key, nonce, uid, terms))
    print(f"Advanced mode unlocked (uid {uid.hex()}, terms v{terms}).")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", required=True)
    ap.add_argument("-b", "--baud", type=int, default=115200)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("ping")
    t = sub.add_parser("telemetry")
    t.add_argument("--watch", action="store_true")
    d = sub.add_parser("detect")
    d.add_argument("--yes", action="store_true")
    sub.add_parser("faults")
    sub.add_parser("clear")
    p = sub.add_parser("pas")
    p.add_argument("level", type=int)
    sub.add_parser("save")
    sub.add_parser("defaults")
    sub.add_parser("reboot")
    pr = sub.add_parser("profile")
    pr.add_argument("name", choices=PROFILES)
    bp = sub.add_parser("boot-profile")
    bp.add_argument("name", choices=PROFILES[:3])
    sub.add_parser("profile-info")
    u = sub.add_parser("unlock")
    u.add_argument("--key-file", required=True)
    u.add_argument("--terms", type=int)
    u.add_argument("--yes", action="store_true")
    sub.add_parser("lock")
    args = ap.parse_args()

    link = Link(args.port, args.baud)
    if args.cmd == "ping":
        r = link.request("PING")
        print(f"{r[:4].decode()} firmware v{r[4]}.{r[5]}.{r[6]}")
        if len(r) > 7 and r[7] & 1:
            print("WARNING: development unlock key in use - not for production")
    elif args.cmd == "telemetry":
        cmd_telemetry(link, args)
    elif args.cmd == "detect":
        cmd_detect(link, args)
    elif args.cmd == "faults":
        cmd_faults(link, args)
    elif args.cmd == "clear":
        link.request("CLEAR_FAULTS")
        print("latched faults cleared")
    elif args.cmd == "pas":
        link.request("SET_PAS_LEVEL", bytes([args.level]))
    elif args.cmd == "save":
        link.request("SAVE_CONFIG")
    elif args.cmd == "defaults":
        link.request("LOAD_DEFAULTS")
    elif args.cmd == "reboot":
        link.request("REBOOT")
    elif args.cmd == "profile":
        link.request("SET_PROFILE", bytes([PROFILES.index(args.name)]))
        print(f"profile: {args.name}")
    elif args.cmd == "boot-profile":
        link.request("SET_BOOT_PROFILE", bytes([PROFILES.index(args.name)]))
    elif args.cmd == "profile-info":
        cmd_profile_info(link, args)
    elif args.cmd == "unlock":
        cmd_unlock(link, args)
    elif args.cmd == "lock":
        link.request("LOCK_ADVANCED")
        print("advanced mode locked")


if __name__ == "__main__":
    main()
