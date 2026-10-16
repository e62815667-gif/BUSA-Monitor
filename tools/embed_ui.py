"""Build the same UI into the controller. No runtime CDN or network dependencies."""
from pathlib import Path
import sys
root = Path(__file__).resolve().parents[1]
html = (root / "index.html").read_text()
for name in ("lab-core.js", "lab-app.js"):
    html = html.replace(f'<script src="./{name}"></script>',
                        "<script>" + (root / name).read_text() + "</script>")
# Local HTTP cannot install a service worker; HTTPS distribution retains the manifest.
html = html.replace('<link rel="manifest" href="./manifest.webmanifest">', "")
delimiter = "BUSA_LAB_UI"
assert ")" + delimiter + '"' not in html
Path(sys.argv[1]).write_text('#pragma once\nstatic const char LAB_PAGE[] PROGMEM = R"'
                           + delimiter + "(" + html + ")" + delimiter + '";\n')

