"""Pinned build fixes. Fail closed if Arduino 2.0.17 source differs; idempotent."""
from pathlib import Path
import sys

def patch(core):
    core=Path(core)
    assert 'version=2.0.17' in (core/'platform.txt').read_text(), 'Only Arduino ESP32 2.0.17 supported'
    p=core/'libraries/WiFi/src/WiFiGeneric.cpp'
    src=p.read_text()
    old='else if(first_connect) {'
    new='else if(first_connect && WiFi.getAutoReconnect()) {'
    if new not in src:
        assert src.count(old)==1, 'Unexpected WiFi first-connect handler'
        p.write_text(src.replace(old,new))
    p=core/'libraries/WebServer/src/WebServer.h'
    src=p.read_text()
    # Seconds-compatible limits: server is synchronous. Bound slow ordinary
    # request/chunk waits; OTA has a separate 180 s overall browser timeout.
    for name,old,new in [('HTTP_MAX_DATA_WAIT',5000,2000),('HTTP_MAX_POST_WAIT',5000,2000),
                         ('HTTP_MAX_SEND_WAIT',5000,2000),('HTTP_MAX_CLOSE_WAIT',2000,250)]:
        marker=f'#define {name} {old}'
        patched=f'#define {name} {new}'
        if patched not in src:
            assert src.count(marker)==1, f'Unexpected {name}'
            src=src.replace(marker,patched)
    p.write_text(src)
    print('Patched Arduino 2.0.17: disabled-auto-reconnect respected on first failure; bounded HTTP waits')

if __name__=='__main__':patch(sys.argv[1])
