# BUSA modular carrier — concept, 08.11.2026

User decisions: ESP32 and four-channel sound-relay block are replaceable modules. Sensor connections are modular. Add a dedicated connector for a future pulse-output collection flowmeter. Drawings are conceptual placement/connection views, not a verified electrical schematic or manufacturing layout.

The ESP32-S3 N16R8 development module plugs into female headers. Exact module model, pin count, header spacing, USB clearance and antenna keep-out must follow its drawing before placement is frozen. External relay module plugs into a labelled keyed harness. Final user decision: the existing PCF8575 keyboard-expander module plugs into female sockets on the carrier, like ESP32. Power conversion and input protection are carrier functions to be specified separately.

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

User subsequently supplied actual photos of the ESP32, PCF8575 module, relay block, TFT, temperature probes, current mint-green membrane keypad, and white YF-S401. Physical footprint dimensions are still required for routing.

## Final modular arrangement after supplied photos

- User briefly considered a soldered PCF8575, then explicitly chose the complete PCF8575 module plugged into the carrier like ESP32. This latest choice supersedes the chip-only proposal.
- Reuse the actual short rectangular blue PCF8575 module from the photo. Its two eight-I/O pin banks and separate five-pin I2C edge need matching sockets; measure its actual holes/pitch before producing a footprint.
- Current keyboard is the light mint-green flat membrane 5×4 with yellow F1/F2, red #/*/Esc/Ent and white number/arrow keys. Earlier black 4×4 hardware in the group photo is obsolete.
- Four temperature probes connect directly to separate ESP32 inputs (GPIO16,17,18,21 in the current firmware), without DS2482 or another temperature bridge. PCF8575 is for the keypad only.
- PCA9685 light driver is removed. Retain the actual blue four-channel SONGLE SRD-12VDC-SL-C sound-relay board as a separate removable module.
- User's flowmeter close-up confirms a white/translucent YF-S401, four lid screws, opposed ribbed hose barbs, red/yellow/black leads, label range 0.3–6 L/min. This image is a visual reference, not a confirmed voltage/output-level specification.
- The future collection sensor remains a reserved separate input; its exact model is unconfirmed. Do not substitute a generic pictured sensor as the selected part.
- Latest concept image: /workspace/generated_images/exec-7fd3c63c-110a-46d3-9c94-385c8f5aa191.png. It illustrates the corrected module identities and upward-facing carrier connectors. Component footprints, pin counts, trace routing and electrical ratings must come from measured hardware/datasheets, not from this rendering.

## Electrical R1 project

User requested a checked electrical schematic after identifying the invented third DC power wire and incorrect relay rendering. Actual screen VCC confirmed by user: 3.3 V. R1 is now in hardware/schematic_r1: native KiCad project, exported netlist, 0 ERC violations, independent netlist-to-firmware checks and a real injected-output-conflict negative control. It replaces the illustration as the source of proposed electrical connections. See its README for outstanding relay JD-VCC, 3.3 V load, footprint and collection-model measurements. It is not bench approval or manufacturing Gerbers. Firmware is unchanged.
