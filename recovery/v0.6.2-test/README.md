# Recovery provenance — BUSA-1M №001
Source: Google Drive archive `БУСА-1М_001_v0.6.2_TEST_прошивка_приложение_инструкции.zip`.
Recovered original file: `recovery/v0.6.2-test/BusaLogic.h`.
SHA-256 (archive member): `94de493fd8baf2d8d0e1ea0555fd1cbdcf65d568cf2c435be42aee2ce2d7d741`.

Other source members identified in the ZIP but **not yet committed to this recovery folder**:
- `BUSA_1M_001_ESP32S3_v0_6_2_TEST.ino` — SHA-256 `be037cf970b2023a62310fa0df1d55bb4479a714f23376c1219ee1991745675c`
- `LocalPage.h` — SHA-256 `59908361a8d23fade31a278e733964177a74e519ff375b921e397644131efc66`
- `app.js` — SHA-256 `0332f66dd257e15be1005e78672e73d6eea58fd0749fd752a7c0bd4bd0b56227`

These are v0.6.2-test sources, **not a validated v0.7.2 implementation**. Original `BusaAutomation.h` and full T1–T4/K1–K4 algorithm implementation are not established by this recovery.

No physical relay output should be activated. The existing v0.6.2-test firmware on hardware must remain unchanged until target compilation and independent safety verification.
