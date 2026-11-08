# BUSA modular carrier — concept, 08.11.2026

User decisions: ESP32 and four-channel sound-relay block are replaceable modules. Sensor connections are modular. Add a dedicated connector for a future pulse-output collection flowmeter. Drawings are conceptual placement/connection views, not a verified electrical schematic or manufacturing layout.

The ESP32-S3 N16R8 development module plugs into female headers. Exact module model, pin count, header spacing, USB clearance and antenna keep-out must follow its drawing before placement is frozen. External relay module plugs into a labelled keyed harness. The PCF8575 keypad interface, power conversion and input protection belong on the carrier unless exact chosen parts require a separate footprint/module.

Existing firmware pins preserved:

| Function | GPIO |
| --- | --- |
| Water temperature | 16 |
| Column temperature | 17 |
| Cube temperature | 18 |
| Alarm temperature | 21 |
| Cooling-water pulses | 15 |
| Four sound-relay controls | 7 / 6 / 5 / 4 |
| Keypad PCF8575 SDA / SCL | 40 / 41 |
| TFT CS / DC / RST / MOSI / SCK | 10 / 11 / 12 / 13 / 14 |
| Buzzer | 2 |
| Collection pulse input, proposed only | 1 |

GPIO1 is a provisional free input according to the current firmware. Check exposure and other functions on the exact selected module before freezing the schematic. No collection ISR or GPIO configuration has been added to firmware. Do not use GPIO19/20 reserved for native USB, flash/PSRAM signals, or boot-strapping pins as casual spares.

Collection connector concept: V_SENSOR, GND, PULSE, clearly keyed. Sensor power voltage, current, output type (open collector versus push-pull), pulse rate and calibration must follow the selected sensor datasheet. Provide a configurable conditioning/protection stage before the 3.3 V GPIO. Never connect an unspecified powered pulse output directly to ESP32. Cooling-water input is a separate independent connector and channel.

Four individual temperature connectors, TFT harness, keyed 9-wire matrix-keypad connector, buzzer connector, DC input, relay control harness, test pads, mounting holes. Match wire lengths/connectors and sensor grounding/pull-ups to the actual installation. Include reverse-polarity protection, appropriate fuse, transient protection, regulator and local decoupling based on confirmed supply/module specifications. Exact board size, relay-module interface, supply rating and connector pitch remain open.

User-facing relay signal names: «ТЭН откл.», «Кран закрыт», «Кран открыт», «Авария». These are explanatory labels for sound notifications. Preserve existing sound-only output semantics.

## User corrections and verified reference status

Selected concept: the second modular connection illustration. All carrier cable connectors must face upward, with vertical insertion perpendicular to the board. ESP32 and the four-channel sound-relay assembly stay replaceable.

The user requires drawings to show their actual hardware. Earlier generated modules and minor PCB components were illustrative placeholders and must not be presented as the actual installed parts.

Verified source references found on connected Drive:

- Actual ESP32 photo: https://drive.google.com/file/d/1SnPyNz7T9ATxajYS5eNu6CzVbDZB5M-i/view . ESP32-S3 N16R8, 44-pin development board, TWO USB connectors at the upper end, antenna at the lower end. The earlier single-USB render is incorrect.
- Display identity: GMT028-05 V1.1; archive contains an approved textual connection diagram, not a photo of the physical module.
- Keyboard identity: membrane matrix 5×4, 20 keys, nine-wire ribbon. Current source faceplate: F1/F2/#/*; 1/2/3/UP; 4/5/6/DOWN; 7/8/9/ESC; LEFT/0/RIGHT/ENT. Archive text confirms legends and membrane construction. The earlier raised generic A–H keypad is incorrect. Exact appearance requires its actual photo.
- Relay markings recorded from historical user photo: four SONGLE SRD-12VDC-SL-C relays. The archive image is a connection diagram; it is not evidence of module dimensions/connector positions.
- Cooling flowmeter source identity YF-S401. Physical photo/dimensions not found in searched project folders.
- Future collection sensor: user selected pulse-output flowmeter type, no exact model confirmed. Reserve a labelled top-entry connector; do not depict an invented physical sensor as the selected device.

Historical October connection documents contain outdated GPIO assignments. Their part descriptions/photos are useful references; retain the current firmware pin map above rather than copying those old wires into the new board.

Needed before a faithful all-real-components illustration: photos of the actual membrane keyboard, display module, relay assembly and cooling flowmeter. A clear overall photo plus close-ups where markings are unreadable is sufficient for the visual stage; physical footprint dimensions will still be required for routing.
