"""Compatibility entry point: FIX3 is canonical source, never a hidden build patch.
Historical source rewriting is intentionally retired. Older workflows must not
silently turn an audited checkout into different firmware.
"""
from validate_ble_ota import validate

if __name__=='__main__':
    validate()
