"""Source integration guard; not a hardware startup test."""
from pathlib import Path
root=Path(__file__).resolve().parents[1]
base=root/'firmware/BUSA_Display_N4R8'
h=(base/'BUSA_GuitionHardware.h').read_text()
ino=(base/'BUSA_Display_N4R8.ino').read_text()
u=(base/'BUSA_TouchUI.h').read_text()
assert 'void beginDisplay()' in h, 'LCD startup still combined with optional peripherals'
display=h[h.index('void beginDisplay()'):h.index('void beginPeripherals()')]
assert display.index('lcd_.begin(LCD_CLOCK_HZ)') < display.index('backlight(true)') < display.index('canvas_.begin(GFX_SKIP_OUTPUT_BEGIN)'), 'Physical LCD and backlight must precede canvas allocation'
assert 'SD.begin(' not in display and 'touch_.begin(' not in display
assert 'psramFound()' in display and 'ESP.getPsramSize()' in display
setup=ino[ino.index('void setup()'):ino.index('void loop()')]
assert setup.index('displayHardware.beginDisplay()') < setup.index('displayHardware.beginPeripherals()') < setup.index('interface.begin()')
assert 'displaySession.begin()' in setup and 'UART' in setup
assert 'FIRST_FRAME' in h and 'LVGL' in u
assert 'BUSA-DISPLAY-N4R8-OTA1-' in ino, 'OTA family marker must be retained in the running version'
assert 'verifyRollbackLater()' in ino, 'Application must defer automatic rollback acceptance until health verification'
print('PASS: source-level boot isolation, diagnostics and OTA identity')
