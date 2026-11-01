"""BUSA V15.3: identical local/online browser shell and truthful pin report."""
from pathlib import Path
import re
root=Path("firmware/BUSA_1M_WIFI_3v_GOOGLE")
main=(root/"BUSA_Main.h").read_text()
online=Path("online/index.html").read_text()
status=(root/"BUSA_V15_Status.h").read_text()
assert '15v5-N16R8-STABLE-LOCAL' in main
assert 'FW V15.5 N16R8' in main
assert 'FW V14.1 LIGHT' not in main
assert 'id="themeSwitch"' in main and 'id="themeSwitch"' in online
assert "localStorage.getItem('busa-pwa-dark')!=='0'" in main
assert "localStorage.getItem('busa-pwa-dark')!=='0'" in online
assert 'data-quick-local="wifi"' not in main
assert 'class="localShortcuts"' not in main
assert 'class="quickMenu"' not in online
assert 'class="connectionStrip"' not in online
assert 'id="appHealth"' in main and 'id="statusText"' in online
for name in ('Главная','Температуры','Отбор','Графики','События','Настр.','Кальк.'):
    assert f'<span>{name}</span>' in main and f'<span>{name}</span>' in online, name
for label in ('ИНТ','ЛОК','ДАН'):
    assert label in main and label in online
for var in ('TFT_CS','TFT_DC','TFT_RST','TFT_MOSI','TFT_SCK'):
    assert f'String({var})' in status, var
assert '"spi_mosi":11' not in status
assert "id=\"themeSwitch\"" in main
# A monitor's connection status remains accessible in Settings, not a full-width banner.
assert 'id="appHealth"' in main[main.index('data-section="5"'):main.index('data-section="6"')]
print("PASS: V15.3 full monitor, matching phone shell, no top connection banner, correct TFT GPIO")
