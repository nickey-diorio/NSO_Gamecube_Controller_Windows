@echo off
REM Builds dist\NSO-GC-Bridge.exe  (run this on Windows, in this folder)
python -m pip install -r requirements.txt pyinstaller
python -m PyInstaller --noconfirm --onefile --console --name NSO-GC-Bridge ^
  --collect-all vgamepad --collect-all libusb_package ^
  --hidden-import hid --hidden-import usb.backend.libusb1 ^
  nso_gc_bridge.py
echo.
echo Done. Your program is: dist\NSO-GC-Bridge.exe
pause
