#!/usr/bin/env python3
"""
nso_gc_bridge.py - use the Nintendo Switch Online (Switch 2) GameCube controller
on Windows 10/11 over USB, as a virtual Xbox 360 controller, WITH rumble.

How it works
------------
1. The controller stays silent until it receives a few setup commands on the
   bulk endpoints of its vendor-specific USB interface (interface 1).
   -> done with pyusb/libusb. This needs the WinUSB driver on interface 1
      (one-time Zadig step, see README below).
2. Input reports (report id 0x0A) are read from the normal HID interface
   -> done with hidapi.
3. They are converted and fed to a virtual Xbox 360 pad (ViGEmBus / vgamepad).
4. When a game rumbles the virtual pad, the GameCube controller's ERM motor is
   driven with on/off HID output reports (report id 3), using the same
   error-diffusion scheme as the Linux kernel driver so different strengths
   feel different.

Protocol details come from the Linux kernel's hid-nintendo Switch 2 patches
(Vicki Pfau / Valve), the NSO GameCube BLE protocol notes by RyanCopley and
research by BlueRetro / ndeadly. This script is NOT tested against real hardware
by its author - use --dump / --test-rumble to verify on your machine.

Setup (one time)
----------------
  pip install -r requirements.txt          (vgamepad installs ViGEmBus; reboot if asked)
  Plug the controller in with USB-C.
  Run Zadig (https://zadig.akeo.ie), Options > List All Devices, pick the entry
  for the controller that says "Interface 1" (VID 057E, PID 2073) and install
  the WinUSB driver for ONLY that interface. Do NOT replace interface 0.
  python nso_gc_bridge.py
"""
import argparse
import math
import struct
import sys
import threading
import time

__version__ = "1.0.0"

VID, PID = 0x057E, 0x2073
GC_REPORT_ID = 0x0A          # input report id for the GameCube controller
RUMBLE_REPORT_ID = 0x03      # output report id used for rumble

# ---- command ids / subcommands (hid-nintendo.h) --------------------------------
CMD_FLASH, SUB_FLASH_READ = 0x02, 0x04
CMD_INIT, SUB_INIT_REPORT, SUB_INIT_USB = 0x03, 0x0A, 0x0D
CMD_LED, SUB_LED_PATTERN = 0x09, 0x07
CMD_FEATSEL, SUB_FEAT_MASK, SUB_FEAT_ENABLE = 0x0C, 0x02, 0x04
CMD_RUMBLE_INIT = 0x11

FEAT_BUTTONS, FEAT_ANALOG, FEAT_IMU, FEAT_RUMBLE = 0x01, 0x02, 0x04, 0x20

# flash addresses (address, size)
FLASH_FACTORY_L = (0x130A8, 9)
FLASH_FACTORY_R = (0x130E8, 9)
FLASH_TRIGGER = (0x13140, 2)
FLASH_USER_L = (0x1FC040, 11)
FLASH_USER_R = (0x1FC080, 11)
USER_CAL_MAGIC = 0xA1B2

# ---- Xbox 360 button masks (XInput) ----------------------------------------------
X_UP, X_DOWN, X_LEFT, X_RIGHT = 0x0001, 0x0002, 0x0004, 0x0008
X_START, X_BACK, X_LTHUMB, X_RTHUMB = 0x0010, 0x0020, 0x0040, 0x0080
X_LB, X_RB, X_GUIDE = 0x0100, 0x0200, 0x0400
X_A, X_B, X_X, X_Y = 0x1000, 0x2000, 0x4000, 0x8000

# ---- button mapping: (report byte index, bit mask) -> Xbox button ----------------
# Edit this table to remap. Byte indexes are positions in the 0x0A report
# (index 0 is the report id).
BUTTON_MAP = [
    (3, 0x01, X_B),        # B
    (3, 0x02, X_A),        # A
    (3, 0x04, X_Y),        # Y
    (3, 0x08, X_X),        # X
    (3, 0x20, X_RB),       # Z button (reported as "ZR")
    (3, 0x40, X_START),    # Start
    (4, 0x20, X_LB),       # "ZL" bit
    (5, 0x01, X_GUIDE),    # Home
    (5, 0x02, X_BACK),     # Capture
    (5, 0x10, X_RTHUMB),   # C (chat) button
]
DPAD_MAP = [(4, 0x08, X_UP), (4, 0x01, X_DOWN), (4, 0x04, X_LEFT), (4, 0x02, X_RIGHT)]
R_CLICK = (3, 0x10)        # digital click at end of right trigger travel
L_CLICK = (4, 0x10)        # digital click at end of left trigger travel


# =================================================================================
# Pure decoding helpers (no hardware needed, unit-testable)
# =================================================================================
class AxisCal:
    def __init__(self, neutral=0, negative=0, positive=0):
        self.neutral, self.negative, self.positive = neutral, negative, positive

    def valid(self):
        return bool(self.neutral and self.negative and self.positive)


def parse_stick_cal(d):
    """9 bytes -> (AxisCal x, AxisCal y) or None when the flash is blank."""
    if len(d) < 9 or all(b == 0xFF for b in d[:9]):
        return None
    x = AxisCal(d[0] | ((d[1] & 0x0F) << 8),
                d[6] | ((d[7] & 0x0F) << 8),
                d[3] | ((d[4] & 0x0F) << 8))
    y = AxisCal((d[1] >> 4) | (d[2] << 4),
                (d[7] >> 4) | (d[8] << 4),
                (d[4] >> 4) | (d[5] << 4))
    return x, y


class Calibration:
    def __init__(self):
        self.left = (AxisCal(), AxisCal())
        self.right = (AxisCal(), AxisCal())
        self.lt_zero = 32      # resting value is roughly 0x22
        self.rt_zero = 32


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def conv_axis(raw, cal):
    if cal.valid():
        v = raw - cal.neutral
        v = int(v * 32768 / (cal.negative if v < 0 else cal.positive))
    else:
        v = (raw - 2048) * 16
    return _clamp(v, -32767, 32767)


def conv_stick(b0, b1, b2, cals, deadzone):
    rx = b0 | ((b1 & 0x0F) << 8)
    ry = (b1 >> 4) | (b2 << 4)
    x = conv_axis(rx, cals[0])
    y = conv_axis(ry, cals[1])          # raw "up" is positive, same as XInput
    if deadzone > 0:
        mag = math.hypot(x, y) / 32767.0
        if mag < deadzone:
            return 0, 0
        scale = (min(mag, 1.0) - deadzone) / (1.0 - deadzone) / mag
        x, y = int(x * scale), int(y * scale)
    return _clamp(x, -32768, 32767), _clamp(y, -32768, 32767)


def conv_trigger(raw, zero):
    span = max(1, 232 - zero)
    v = _clamp((raw - zero) * 255 // span, 0, 255)
    return 0 if v < 4 else v      # ignore resting jitter


def decode_report(d, cal, deadzone=0.05):
    """Return (buttons, lt, rt, lx, ly, rx, ry) from a 0x0A report (d[0]==0x0A)."""
    buttons = 0
    for idx, mask, xb in BUTTON_MAP:
        if d[idx] & mask:
            buttons |= xb
    for idx, mask, xb in DPAD_MAP:
        if d[idx] & mask:
            buttons |= xb
    lt = conv_trigger(d[13], cal.lt_zero)
    rt = conv_trigger(d[14], cal.rt_zero)
    if d[L_CLICK[0]] & L_CLICK[1]:
        lt = 255
    if d[R_CLICK[0]] & R_CLICK[1]:
        rt = 255
    lx, ly = conv_stick(d[6], d[7], d[8], cal.left, deadzone)
    rx, ry = conv_stick(d[9], d[10], d[11], cal.right, deadzone)
    return buttons, lt, rt, lx, ly, rx, ry


UNIFIED_REPORT_ID = 0x05
# (byte index, mask, xbox) for the "unified" report layout (index 0 = report id)
UNIFIED_BUTTONS = [
    (5, 0x08, X_A), (5, 0x04, X_B), (5, 0x02, X_X), (5, 0x01, X_Y),
    (5, 0x80, X_RB),                 # Z button (reported as ZR)
    (7, 0x80, X_LB),                 # "ZL" bit
    (6, 0x02, X_START), (6, 0x10, X_GUIDE), (6, 0x20, X_BACK), (6, 0x40, X_RTHUMB),
    (7, 0x02, X_UP), (7, 0x01, X_DOWN), (7, 0x08, X_LEFT), (7, 0x04, X_RIGHT),
]


def decode_unified(d, cal, deadzone=0.05):
    buttons = 0
    for idx, mask, xb in UNIFIED_BUTTONS:
        if d[idx] & mask:
            buttons |= xb
    lt = conv_trigger(d[0x3D], cal.lt_zero)
    rt = conv_trigger(d[0x3E], cal.rt_zero)
    if d[7] & 0x40:
        lt = 255
    if d[5] & 0x40:
        rt = 255
    lx, ly = conv_stick(d[11], d[12], d[13], cal.left, deadzone)
    rx, ry = conv_stick(d[14], d[15], d[16], cal.right, deadzone)
    return buttons, lt, rt, lx, ly, rx, ry


def build_cmd(cmd, sub, payload=b""):
    # [cmd, 0x91 (host->device | OK), 0x00 (USB transport), sub, 0, len, 0, 0] + payload
    return bytes([cmd, 0x91, 0x00, sub, 0x00, len(payload), 0x00, 0x00]) + payload


# =================================================================================
# USB configuration interface (bulk endpoints on the vendor-specific interface)
# =================================================================================
class UsbConfig:
    def __init__(self):
        import usb.core
        import usb.util
        self.usb = usb
        try:
            import libusb_package
            backend = libusb_package.get_libusb1_backend()
        except Exception:
            backend = None

        notes = []
        found_any = False
        for dev in usb.core.find(find_all=True, idVendor=VID, idProduct=PID, backend=backend) or []:
            found_any = True
            tag = "node bus=%s addr=%s" % (dev.bus, dev.address)
            stage = "reading configuration"
            try:
                try:
                    cfg = dev.get_active_configuration()
                except Exception as e1:
                    notes.append("%s: get_active_configuration failed (%s)" % (tag, e1))
                    stage = "set_configuration"
                    dev.set_configuration()
                    cfg = dev.get_active_configuration()
                ifs = [(i.bInterfaceNumber, hex(i.bInterfaceClass)) for i in cfg]
                notes.append("%s: interfaces (number, class) = %s" % (tag, ifs))
                for intf in cfg:
                    if intf.bInterfaceClass != 0xFF:
                        continue
                    stage = "finding endpoints on interface %d" % intf.bInterfaceNumber
                    ep_out = usb.util.find_descriptor(
                        intf, custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress)
                        == usb.util.ENDPOINT_OUT and usb.util.endpoint_type(e.bmAttributes)
                        == usb.util.ENDPOINT_TYPE_BULK)
                    ep_in = usb.util.find_descriptor(
                        intf, custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress)
                        == usb.util.ENDPOINT_IN and usb.util.endpoint_type(e.bmAttributes)
                        == usb.util.ENDPOINT_TYPE_BULK)
                    if ep_out is None or ep_in is None:
                        notes.append("%s: vendor interface %d has no bulk IN/OUT pair" % (tag, intf.bInterfaceNumber))
                        continue
                    stage = "claiming interface %d" % intf.bInterfaceNumber
                    usb.util.claim_interface(dev, intf.bInterfaceNumber)
                    self.dev, self.ep_out, self.ep_in = dev, ep_out, ep_in
                    self.intf_num = intf.bInterfaceNumber
                    return
                if not any(i.bInterfaceClass == 0xFF for i in cfg):
                    notes.append("%s: no vendor-specific (0xFF) interface visible on this node" % tag)
            except Exception as e:   # try the next candidate node
                notes.append("%s: FAILED while %s -> %s" % (tag, stage, e))
        if not found_any:
            raise RuntimeError("USB device 057E:2073 not visible to libusb. Is the controller plugged in "
                               "with USB-C, and is the WinUSB driver installed on its Interface 1 (Zadig)?")
        raise RuntimeError("Could not open the controller's vendor interface.\n  " + "\n  ".join(notes) +
                           "\n  See the troubleshooting steps for 'Access denied': WinUSB must be the driver on "
                           "Interface 1, and no other program (Steam, Dolphin, another bridge) may be using it.")

    def write(self, data, timeout=500):
        return self.ep_out.write(data, timeout)

    def _read(self, timeout):
        try:
            return bytes(self.ep_in.read(64, timeout))
        except self.usb.core.USBError as e:
            if getattr(e, "errno", None) in (110, 10060) or "timeout" in str(e).lower():
                return None
            raise

    def transact(self, cmd, sub, payload=b"", timeout=600):
        self.write(build_cmd(cmd, sub, payload))
        end = time.time() + timeout / 1000.0
        while time.time() < end:
            r = self._read(100)
            if r and len(r) >= 8 and r[0] == cmd and r[3] == sub:
                return r
        return None

    def flash_read(self, addr, size):
        msg = bytes([size, 0x7E, 0, 0]) + struct.pack("<I", addr)
        r = self.transact(CMD_FLASH, SUB_FLASH_READ, msg)
        if not r or not (r[1] & 0x01) or len(r) < 16 + size:
            return None
        body = r[8:]
        if body[0] != size or struct.unpack("<I", body[4:8])[0] != addr:
            return None
        return body[8:8 + size]


def initialize_controller(cfg, player, minimal=False):
    """Run the setup sequence. Returns a Calibration."""
    cal = Calibration()
    led_mask = {1: 0x01, 2: 0x03, 3: 0x05, 4: 0x06}.get(player, 0x01)

    if not minimal:
        def stick(addr_size, user=False):
            d = cfg.flash_read(*addr_size)
            if not d:
                return None
            if user:
                if struct.unpack("<H", d[:2])[0] != USER_CAL_MAGIC:
                    return None
                d = d[2:]
            return parse_stick_cal(d)

        for side, fac, usr in (("left", FLASH_FACTORY_L, FLASH_USER_L), ("right", FLASH_FACTORY_R, FLASH_USER_R)):
            c = stick(usr, True) or stick(fac)
            if c:
                setattr(cal, side, c)
        d = cfg.flash_read(*FLASH_TRIGGER)
        if d and d[0] != 0xFF and d[1] != 0xFF:
            cal.lt_zero, cal.rt_zero = d[0], d[1]
        print("calibration: left=%s right=%s triggers zero=%d/%d" % (
            "factory/user" if cal.left[0].valid() else "default",
            "factory/user" if cal.right[0].valid() else "default", cal.lt_zero, cal.rt_zero))

        cfg.transact(CMD_FEATSEL, SUB_FEAT_MASK,
                     bytes([FEAT_BUTTONS | FEAT_ANALOG | FEAT_IMU | FEAT_RUMBLE, 0, 0, 0]))
        cfg.transact(CMD_FEATSEL, SUB_FEAT_ENABLE,
                     struct.pack("<I", FEAT_BUTTONS | FEAT_ANALOG | FEAT_RUMBLE))
        cfg.transact(CMD_RUMBLE_INIT, 0x01)

    cfg.transact(CMD_INIT, SUB_INIT_REPORT, struct.pack("<I", GC_REPORT_ID))
    cfg.transact(CMD_LED, SUB_LED_PATTERN, bytes([led_mask, 0, 0, 0, 0, 0, 0, 0]))
    if not minimal:
        cfg.transact(CMD_INIT, SUB_INIT_USB, bytes([0x01, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]))
    return cal


# =================================================================================
# Rumble
# =================================================================================
class Rumble:
    """The GC controller has an on/off motor, so intensity is made by rapid
    pulse-density modulation (same error-diffusion as the Linux kernel driver)."""
    OFF, ON, STOP = 0, 1, 2

    def __init__(self, send_report, interval=0.004):
        self.send_report = send_report
        self.interval = interval
        self.amp = 0                      # 0..65535
        self._evt = threading.Event()
        self._stop = False
        self._seq = 0
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def set_level(self, amp):
        self.amp = _clamp(int(amp), 0, 65535)
        self._evt.set()

    def close(self):
        self.amp = 0
        self._stop = True
        self._evt.set()

    def _emit(self, state):
        buf = bytearray(64)
        buf[0] = RUMBLE_REPORT_ID
        buf[1] = 0x50 | self._seq
        buf[2] = state
        self._seq = (self._seq + 1) & 0x0F
        self.send_report(bytes(buf))

    def _run(self):
        err, active = 0, False
        while not self._stop:
            amp = self.amp
            if amp == 0:
                if active:
                    self._emit(self.STOP)
                    active, err = False, 0
                self._evt.wait()
                self._evt.clear()
                continue
            active = True
            if err < amp:
                self._emit(self.ON)
                err += 65535 - amp
            else:
                self._emit(self.OFF)
                err -= amp
            time.sleep(self.interval)


# =================================================================================
# Main bridge
# =================================================================================
def open_hid(index=0):
    import hid
    devs = [d for d in hid.enumerate(VID, PID)]
    if not devs:
        return None
    devs.sort(key=lambda d: (d.get("interface_number", 0), str(d.get("path"))))
    for i, d in enumerate(devs):
        print("  HID #%d: interface=%s usage_page=0x%04X usage=0x%04X%s" % (
            i, d.get("interface_number"), d.get("usage_page", 0), d.get("usage", 0),
            "   <-- using this" if i == index else ""))
    h = hid.device()
    h.open_path(devs[min(index, len(devs) - 1)]["path"])
    return h


def run(args):
    import vgamepad as vg

    if sys.platform == "win32":
        try:      # 1 ms timer so the rumble pulses are not stretched to 15 ms
            import ctypes
            ctypes.windll.winmm.timeBeginPeriod(1)
        except Exception:
            pass

    try:
        pad = vg.VX360Gamepad()
    except Exception as e:
        print("Could not create the virtual Xbox 360 controller: %s\n"
              "The ViGEmBus driver is probably not installed. Install it from\n"
              "https://github.com/nefarius/ViGEmBus/releases (ViGEmBus_Setup_x64.msi), reboot, and try again." % e)
        raise SystemExit(1)
    state = {"hid": None, "cfg": None, "warned": False}

    def send_report(buf):
        h = state["hid"]
        try:
            if h is not None and h.write(buf) >= 0:
                return
        except Exception:
            pass
        cfg = state["cfg"]            # experimental fallback: same bytes over the bulk endpoint
        if cfg is not None:
            try:
                cfg.write(buf)
                return
            except Exception:
                pass
        if not state["warned"]:
            state["warned"] = True
            print("warning: rumble packet could not be sent (neither HID nor bulk accepted it)")

    rumble = Rumble(send_report, interval=args.rumble_interval_ms / 1000.0)
    if not args.no_rumble:
        # vgamepad checks the parameter NAMES, so they must match exactly
        def on_rumble(client, target, large_motor, small_motor, led_number, user_data):
            rumble.set_level(max(large_motor, small_motor >> 1) * 257)
        pad.register_notification(callback_function=on_rumble)

    print("NSO GameCube bridge running. Ctrl+C to quit.")
    try:
        while True:
            cal = Calibration()
            cfg = None
            h = None
            try:
                if not args.no_init:
                    cfg = UsbConfig()
                    cal = initialize_controller(cfg, args.player, args.minimal_init)
                    print("controller initialised")
                state["cfg"] = cfg
                h = open_hid(args.hid_index)
                if h is None:
                    raise RuntimeError("HID interface of 057E:2073 not found")
                state["hid"] = h
                print("HID opened - move a stick or press a button")
                if args.test_rumble:
                    for _ in range(2):
                        rumble.set_level(65535); time.sleep(0.4)
                        rumble.set_level(0); time.sleep(0.3)

                last, last_t, got_any, t0 = None, 0, False, time.time()
                count, count_t, seen_ids = 0, time.time(), set()
                while True:
                    d = h.read(64, 200)
                    now = time.time()
                    if not d:
                        if not got_any and now - t0 > 4:
                            print("no input reports received from the HID interface in 4 s. "
                                  "Try: --minimal-init, a different --hid-index, replug, close Steam, "
                                  "or --no-init if another tool already initialised the controller.")
                            t0 = now + 3600
                        continue
                    d = bytes(d)
                    count += 1
                    if d[0] not in seen_ids:
                        seen_ids.add(d[0])
                        print("receiving report id 0x%02X, %d bytes: %s" % (d[0], len(d), d[:20].hex(" ")))
                    if args.dump and now - last_t > 0.2:
                        last_t = now
                        print("rid=0x%02X len=%d  %s" % (d[0], len(d), d[:24].hex(" ")))
                    if now - count_t > 3 and args.dump:
                        print("  (%.0f reports/s)" % (count / (now - count_t)))
                        count, count_t = 0, now
                    if d[0] == GC_REPORT_ID and len(d) >= 15:
                        vals = decode_report(d, cal, args.deadzone)
                    elif d[0] == UNIFIED_REPORT_ID and len(d) >= 0x3F:
                        vals = decode_unified(d, cal, args.deadzone)
                    else:
                        continue
                    got_any = True
                    if vals == last:
                        continue
                    last = vals
                    b, lt, rt, lx, ly, rx, ry = vals
                    r = pad.report
                    r.wButtons, r.bLeftTrigger, r.bRightTrigger = b, lt, rt
                    r.sThumbLX, r.sThumbLY, r.sThumbRX, r.sThumbRY = lx, ly, rx, ry
                    pad.update()
            except KeyboardInterrupt:
                raise
            except Exception as e:
                print("connection problem: %s\nretrying in 2 s ..." % e)
            finally:
                rumble.set_level(0)
                state["hid"] = None
                state["cfg"] = None
                for obj in (h,):
                    try:
                        obj and obj.close()
                    except Exception:
                        pass
                if cfg is not None:
                    try:
                        cfg.usb.util.dispose_resources(cfg.dev)
                    except Exception:
                        pass
            time.sleep(2)
    except KeyboardInterrupt:
        print("\nbye")
    finally:
        rumble.close()
        time.sleep(0.05)
        pad.reset()
        pad.update()


def _pause_if_frozen():
    if getattr(sys, "frozen", False):
        try:
            input("\nPress Enter to close...")
        except Exception:
            pass


def main():
    p = argparse.ArgumentParser(description="NSO GameCube controller (USB) -> virtual Xbox 360 pad with rumble")
    p.add_argument("--player", type=int, default=1, choices=[1, 2, 3, 4], help="player LED pattern")
    p.add_argument("--deadzone", type=float, default=0.05, help="radial stick deadzone 0..0.5 (default 0.05)")
    p.add_argument("--no-rumble", action="store_true", help="disable rumble forwarding")
    p.add_argument("--rumble-interval-ms", type=float, default=4.0, help="rumble pulse period (default 4)")
    p.add_argument("--test-rumble", action="store_true", help="buzz twice after connecting")
    p.add_argument("--minimal-init", action="store_true", help="only send report-format + LED commands")
    p.add_argument("--no-init", action="store_true", help="skip the USB setup commands (diagnostics)")
    p.add_argument("--hid-index", type=int, default=0, help="which HID entry to open (see the list printed at start)")
    p.add_argument("--dump", action="store_true", help="print raw input reports (diagnostics)")
    p.add_argument("--version", action="version", version="nso_gc_bridge " + __version__)
    args = p.parse_args()
    try:
        run(args)
    except SystemExit:
        _pause_if_frozen()
        raise
    except Exception:
        import traceback
        traceback.print_exc()
        _pause_if_frozen()
        sys.exit(1)


if __name__ == "__main__":
    main()
