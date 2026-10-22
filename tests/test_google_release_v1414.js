'use strict';
// BUSA-1M-001: offline tests for read-only Google update catalogue.
// Never contacts Google, never stores real credentials, never writes MCU flash.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');

const source = fs.readFileSync('google/BUSA_GoogleSheets_Receiver.gs', 'utf8');
const properties = {
  BUSA_SPREADSHEET_ID: 'test-sheet',
  BUSA_TOKEN: 'a'.repeat(64),
  BUSA_VIEW_TOKEN: 'b'.repeat(64)
};
const appended = [];
const tab = { getLastRow: () => 1 + appended.length, appendRow(row) { appended.push(row); } };
const mockProperties = {
  getProperty(key) { return Object.prototype.hasOwnProperty.call(properties, key) ? properties[key] : null; },
  setProperty(key, value) { properties[key] = String(value); },
  setProperties(obj, deleteAll) {
    if (deleteAll) for (const key of Object.keys(properties)) delete properties[key];
    for (const [key, value] of Object.entries(obj)) properties[key] = String(value);
  }
};
const context = {
  PropertiesService: { getScriptProperties: () => mockProperties },
  SpreadsheetApp: { openById(id) {
    assert.equal(id, 'test-sheet');
    return { getSheetByName: name => name === 'BUSA_telemetry' ? tab : null };
  }},
  LockService: { getScriptLock: () => ({ tryLock: () => true, releaseLock() {} }) },
  ContentService: {
    MimeType: { JSON: 'application/json', JAVASCRIPT: 'application/javascript' },
    createTextOutput(body) { return { value: body, setMimeType() { return this; } }; }
  },
  Date, JSON, String, Number, Boolean, Array, Object, RegExp, Math, Error
};
vm.runInNewContext(source, context, { filename: 'BUSA_GoogleSheets_Receiver.gs' });

const initial = { ...properties };
assert.equal(context.busaReleaseMetadata_(), null, 'catalogue must start empty');
context.setupBusaReadOnlyReleaseV1414DualThemes();
for (const key of ['BUSA_SPREADSHEET_ID', 'BUSA_TOKEN', 'BUSA_VIEW_TOKEN']) {
  assert.equal(properties[key], initial[key], key + ' must not be changed');
}
assert.equal(properties.BUSA_RELEASE_VERSION, 'V14.14');
assert.equal(properties.BUSA_RELEASE_TARGET, 'ESP32-S3');
assert.equal(properties.BUSA_RELEASE_BUILD_FLASH, '8388608');
assert.equal(properties.BUSA_RELEASE_APP_BYTES, '1128096');
assert.match(properties.BUSA_RELEASE_APP_SHA256, /^[0-9a-f]{64}$/);
const metadata = context.busaReleaseMetadata_();
assert.equal(metadata.app_size_bytes, 1128096);
assert.equal(metadata.build_flash_bytes, 8388608);
assert.equal(metadata.app_sha256, properties.BUSA_RELEASE_APP_SHA256);

const packet = {
  token: initial.BUSA_TOKEN, device_id: 'BUSA-1M-001',
  kind: 'monitor_diagnostics', version: 3,
  telemetry: {
    device: 'BUSA-1M-001', firmware: '14v14-N8R2-DUAL-THEMES',
    temperatures: [{ c: 25 }, { c: 26 }, { c: 27 }, { c: 28 }],
    diagnostics: {
      onewire: [{ result: 'OK' }, { result: 'OK' }, { result: 'OK' }, { result: 'OK' }],
      keyboard: { scan_error: 0 }, wifi: { sta_connected: true },
      pca9685: { ack: true }, esp32: { free_heap_bytes: 240000 },
      i2c: { addresses: [32, 64, 112] }
    },
    modules: { pcf8575: true }, water: { pulses: 0, calibrated: false },
    clock: { datetime: '23.10.2026 02:00:00' }
  }
};
function request(value) {
  const response = context.doPost({ postData: { contents: JSON.stringify(value) } });
  return JSON.parse(response.value);
}
const result = request(packet);
assert.equal(result.stored, true, 'valid upload must be acknowledged');
assert.equal(result.release.version, 'V14.14');
assert.equal(result.release.app_sha256, properties.BUSA_RELEASE_APP_SHA256);
assert.equal(Object.hasOwn(result, 'command'), false, 'no remote command channel');
assert.equal(Object.hasOwn(result.release, 'download_url'), false, 'no download channel');
assert.equal(appended.length, 1);
assert.equal(appended[0][1], 'BUSA-1M-001');
assert.equal(appended[0][2], '14v14-N8R2-DUAL-THEMES');
assert.ok(!appended[0][21].includes(initial.BUSA_TOKEN), 'never store write token in telemetry');
assert.ok(!appended[0][21].includes(initial.BUSA_VIEW_TOKEN), 'never store view token');

const invalid = JSON.parse(JSON.stringify(packet));
invalid.token = '0'.repeat(64);
assert.equal(request(invalid).stored, false, 'wrong token must be rejected');
assert.equal(appended.length, 1, 'unauthorized packet must not be stored');
assert.equal(properties.BUSA_TOKEN, initial.BUSA_TOKEN);
assert.equal(properties.BUSA_VIEW_TOKEN, initial.BUSA_VIEW_TOKEN);
assert.equal(properties.BUSA_SPREADSHEET_ID, initial.BUSA_SPREADSHEET_ID);

console.log('PASS: V14.14 read-only release metadata, telemetry ACK, token preservation, no remote install');
