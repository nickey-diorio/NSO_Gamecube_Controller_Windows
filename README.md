# NSO GameCube Controller Bridge (Windows, USB)

Makes the **Nintendo Switch Online (Switch 2) GameCube controller** work on Windows 10/11 over
**USB-C**, as a virtual **Xbox 360 controller**, **with rumble**. Works with Dolphin, Steam and any
XInput game.

> Unofficial. Not affiliated with or endorsed by Nintendo. Use at your own risk.
> Wired USB only (no Bluetooth). Tested on one controller / one PC so far - reports welcome.

## One-time setup

1. **Install ViGEmBus** (virtual controller driver): download `ViGEmBus_Setup_x64.msi` from
   <https://github.com/nefarius/ViGEmBus/releases>, install, reboot.
2. **Install WinUSB on the controller's vendor interface** with [Zadig](https://zadig.akeo.ie):
   plug the controller in with USB-C, open Zadig, *Options > List All Devices*, choose the entry
   for the controller that says **Interface 1** (VID 057E, PID 2073), select **WinUSB**, click
   *Install/Replace Driver*, then unplug and replug the controller.
   Do **not** change Interface 0 - that is the normal HID interface the program reads input from.
   (Needed because the controller stays silent until it receives setup commands on Interface 1.)

## Use

Run `NSO-GC-Bridge.exe` (or `python nso_gc_bridge.py`) and leave the window open while you play.
Start it **before** launching games/emulators. Close other controller software (Steam Input for
Switch controllers, other bridges) so nothing else holds the controller.

Dolphin: *Controllers > GameCube Port 1 > Standard Controller > Configure*, device
`XInput/0/Gamepad`.

| GameCube | Xbox |
|---|---|
| A / B / X / Y | A / B / X / Y |
| Z | RB |
| L / R analog | LT / RT (the digital click forces full press) |
| "ZL" bit | LB |
| Start | Start |
| Home | Guide |
| Capture | Back |
| C button | R3 |
| D-pad, both sticks | D-pad, both sticks |

Edit `BUTTON_MAP` at the top of `nso_gc_bridge.py` to remap.

### Options

```
--player 1-4            player LED pattern
--deadzone 0.05         radial stick deadzone
--no-rumble             disable rumble
--rumble-interval-ms 4  rumble pulse period (lower = finer strength steps)
--test-rumble           buzz twice after connecting
--minimal-init          send only report-format + LED setup commands
--hid-index N           pick a different HID entry if several are listed
--dump                  print raw input reports (diagnostics)
```

## Troubleshooting

| Symptom | Try |
|---|---|
| "Could not create the virtual Xbox 360 controller" | Install ViGEmBus, reboot |
| "Access denied" / "could not open vendor interface" | WinUSB on **Interface 1** (Zadig); close Steam/other tools; replug |
| "no input reports received" | `--minimal-init`, `--hid-index 1`, replug |
| Game reacts twice | Hide the physical controller with [HidHide](https://github.com/nefarius/HidHide) |
| Windows SmartScreen / antivirus warning | Common for unsigned PyInstaller exes; build it yourself (below) or run the .py |

## Build the exe yourself

`build_exe.bat` on Windows (needs Python 3.9+), or push a tag like `v1.0.0` and let the included
GitHub Actions workflow build and attach `NSO-GC-Bridge.exe` to a release.

## How it works

Setup commands go to the vendor interface's bulk endpoints (libusb), input reports (id `0x0A`)
are read from the HID interface, converted, and sent to a ViGEm virtual Xbox 360 pad. The
GameCube controller's on/off rumble motor is driven with HID output reports (id `3`), pulsed by
error diffusion so different strengths feel different.

## Credits

Protocol knowledge from the community: the Linux kernel `hid-nintendo` Switch 2 patches (Vicki
Pfau), [NSO-GameCube-Controller-Pairing-App](https://github.com/RyanCopley/NSO-GameCube-Controller-Pairing-App)
(RyanCopley, BLE notes), [BlueRetro](https://github.com/darthcloud/BlueRetro) and
[ndeadly/switch2_controller_research](https://github.com/ndeadly/switch2_controller_research).
Built with [vgamepad](https://github.com/yannbouteiller/vgamepad), ViGEmBus, hidapi and PyUSB.
