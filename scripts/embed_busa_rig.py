#!/usr/bin/env python3
"""Reproducible approved-illustration asset embed for local BUSA Wi-Fi UI."""
from pathlib import Path
from PIL import Image
import base64
import hashlib
import re

root = Path(__file__).resolve().parent.parent
reference = root / "assets" / "BUSA_approved_reference_20261103.jpg"
target = root / "assets" / "BUSA_approved_rig_276.webp"
header = root / "firmware" / "BUSA_1M_WIFI_3v_GOOGLE" / "BUSA_Main.h"

expected_sha = "13242c6c0de16649d6fdf5d252330a7d1cd348f54f9050d65ef7d7fd8818893d"
actual_sha = hashlib.sha256(reference.read_bytes()).hexdigest()
if actual_sha != expected_sha:
    raise SystemExit("STOP: approved image file differs from user-approved screenshot")

with Image.open(reference) as img:
    if img.size != (709, 1536):
        raise SystemExit(f"STOP: unexpected approved screenshot geometry: {img.size}")
    # Exact cropped apparatus from approved image; coordinates verified by
    # independent pixel matching of the user's approved crop.
    rig = img.crop((333, 370, 609, 1017)).convert("RGB")
    if rig.size != (276, 647):
        raise SystemExit("STOP: wrong installation crop")
    rig.save(target, format="WEBP", quality=85, method=6)

uri = "data:image/webp;base64," + base64.b64encode(target.read_bytes()).decode("ascii")
s = header.read_text(encoding="utf-8")
image_pattern = r'data:image/(?:jpeg|webp);base64,[A-Za-z0-9+/=]+'
s, count = re.subn(image_pattern, uri, s, count=1)
if count != 1:
    raise SystemExit(f"STOP: expected exactly one apparatus image; found {count}")

version_old = 'KEYRESTORE-APPROVED-UI-20261103'
version_new = 'KEYRESTORE-APPROVED-UI-RIG-ICONS-FIX-20261103'
if version_old in s:
    s = s.replace(version_old, version_new, 1)
if version_new not in s:
    raise SystemExit("STOP: firmware version identifier mismatch")

for i in range(4):
    if f"busaIconDetail{i}" not in s:
        raise SystemExit(f"STOP: approved T{i+1} detailed icon missing")
if "PRIMARY_AND_BACKUP_OK" not in s:
    raise SystemExit("STOP: keypad NVS primary/backup guard missing")
if 'prefs.clear()' in s or 'nvs_flash_erase' in s:
    raise SystemExit("STOP: unexpected NVS erase")
header.write_text(s, encoding="utf-8")

print("Approved BUSA artwork embedded; WebP bytes:", target.stat().st_size)
print("Source:", header, "PNG crop 276x647 -> self-contained WebP")
