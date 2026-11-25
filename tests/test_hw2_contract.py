from pathlib import Path
root=Path(__file__).resolve().parents[1]
p=root/'firmware/BUSA_1M_WIFI_3v_GOOGLE'
m=(p/'BUSA_Main.h').read_text()
link=(p/'BUSA_DisplayLink.h').read_text()
u=(root/'firmware/BUSA_Display_N4R8/BUSA_TouchUI.h').read_text()
assert all(x not in m for x in ('Adafruit_ST7789','Adafruit_ILI9341','SPI.begin(', 'TFT_CS','startScreen();','renderScreen(now)','keypadAction('))
assert '#include "BUSA_History.h"' in m and 'v13Collect(now)' in m
assert 'busaUartStatusJSON()' in link and 'String snapshot=busaUartStatusJSON()' in link
status=m[m.index('String statusJSON(){'):m.index('String statusJSON(){')+25000]
assert '\\"keypad_input\\"' not in status
assert 'busaKeyEvents.begin(esp_random())' in m and 'publishPhysicalKey' in m
assert 'keyCursor_.consume' in u and 'keyCursor_.reset()' in u and 'keySaveReference' in u
assert 'storeTempReference(sensor,field[0]-' in m
assert 'lv_obj_scroll_to_view_recursive(obj,LV_ANIM_OFF)' in u
print('PASS: GUITION-only rendering, history and UART-only keypad contract')
