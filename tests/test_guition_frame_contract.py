"""Static integration guard for the GUITION full-frame display path."""
from pathlib import Path
import re
base = Path(__file__).resolve().parents[1] / 'firmware/BUSA_Display_N4R8'
h = (base / 'BUSA_GuitionHardware.h').read_text()
u = (base / 'BUSA_TouchUI.h').read_text()
# The factory demo uses the driver's native rotation and inversion.
assert not re.search(r'void\s+setRotation\s*\(', h), 'Stock NV3041A rotation is overridden'
assert 'invertDisplay(false);' in h, 'Stock inversion absent'
assert 'Arduino_Canvas' in h and 'MALLOC_CAP_SPIRAM' in h, 'Full-frame PSRAM canvas missing'
assert 'canvas_.fillScreen(BLACK);' in h and 'canvas_.flush();' in h, 'Initial black frame missing'
assert 'Arduino_GFX& panel(){return canvas_;}' in h, 'LVGL bypasses framebuffer'
flush = u[u.index('static void flush('):u.index('static void readTouch(')]
assert re.search(r'if\(lv_disp_flush_is_last\(d\)\)\{self->hw_\.present\(\);', flush), 'Full frame not presented at final LVGL flush'
assert flush.index('hw_.panel().draw16bitRGBBitmap') < flush.index('hw_.present()') < flush.index('lv_disp_flush_ready(d)')
assert 'LV_COLOR_16_SWAP==0' in u, 'RGB565 swap invariant missing'
assert u.count('"affineC0"') == 3 and '"affine"' not in u, 'Old touch calibration reused after panel rotation'
print('PASS: GUITION canvas, startup, rotation, color and calibration integration')
