# BUSA approved home motion implementation plan

> **For agentic workers:** Use executing-plans with the explicitly requested parallel Android and verification agents.

**Goal:** Ship the approved home illustration and requested fluid motion in CUBE1 firmware and the identical Android UI.

**Architecture:** Keep the existing firmware HTML as the UI source of truth. Embed one text-free image and scoped SVG/CSS animation; generate Android assets from that same HTML.

**Tech Stack:** Arduino ESP32 2.0.17, ESP32-S3 N16R8, existing Java Android SDK build and GitHub Actions.

**Spec:** The owner's approved image SHA256 `e146a94034177af6d9cf81ac37a6b729e8a2a1a7634db84982fea23883f644b7` and subsequent request for motion in the cube, hoses, coil, column and small glass. Base commit `8a643b9a771d01f02133bc812f8f6df094c63e41`, successful run `44890393687`.

## Global constraints

- Change presentation and release identifiers only; preserve hardware, sensor, network, sound, calculator and update behavior.
- Preserve all existing tests and compare normalized backend to its original digest.
- No physical flashing, main merge or overwrite of confirmed release binaries.
- Use neutral BUSA wording in new source and UI.
- Actual data gates animation; absent data stays absent. No mock measurements in shipped code.
- Release APK must retain the existing signer and remain non-debuggable.

## Review focus

- Loss of data pauses motion and clears real readouts.
- Motion follows the visible hoses and glass without moving metal or packing.
- Static artwork and readout nodes survive polling without replacement or flicker.
- Hidden pages and reduced-motion preferences pause animation.
- Both compiled images and packaged Android assets match the reviewed source.

### Task 1: Shared home artwork and motion

Files: `web/assets/holographic-apparatus.webp`, `web/home-motion.js`, `web/home-motion.css`, `web/design.js`, `ci/generate_design.py`, generated `BUSA_Main.h`, new browser regression.

- [x] Add a browser regression for required motion layers and active/offline/hidden/reduced states; baseline failed with absent motion.
- [x] Embed the text-free approved artwork, align scoped motion to it, retain original telemetry bindings.
- [x] Generate design and Android assets; browser regression and existing UI preservation checks passed before independent review.

### Task 2: Release builds

Files: firmware VERSION, Android manifest/workflow, firmware workflow/packager, normalized-version preservation test.

- [x] Assign MOTION1 identifiers without changing normalized backend.
- [x] Add this isolated branch to both existing workflow triggers; retain all board/library versions.
- [ ] Run all available local checks, push the isolated branch, and run GitHub Actions builds.
- [ ] Verify compiled OTA/USB layout, complete partitions, hashes and source commit; verify APK signature and asset parity.

### Task 3: Delivery

- [ ] Independently review the focused diff and preserve confirmed baseline binaries.
- [ ] Provide new OTA BIN, USB BIN and installable signed APK with exact checksums and build links.
- [ ] Explicitly report any unavailable signing key or hardware checks; do not label unsigned/debug artifacts as installable releases.

## Independent review fixes

The reviewer found legacy receiver effects continuing offline and non-integral dash cycles. Added rendered-style regressions reproduced both defects before fixing them. The same regression exposed frozen cube effects after data loss; parent visibility is now gated by the existing data classes. Background-tab pausing is explicitly handled by the passive motion module. No controller changes were required.
