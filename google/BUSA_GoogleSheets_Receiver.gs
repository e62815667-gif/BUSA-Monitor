/**
 * BUSA-1M-001: one-way, read-only telemetry receiver for a user-owned Google Sheet.
 * Set up as a BOUND Apps Script (Google Sheets > Extensions > Apps Script).
 * The ESP32 sends data every 30 seconds by HTTPS. No remote device commands.
 * Do not share the 64-hex-character token or publish it in GitHub.
 */
const BUSA_TAB = 'BUSA_telemetry';
const BUSA_HEADERS = [
  'Получено (UTC)', 'Устройство', 'Прошивка', 'Время ESP32 (UTC)',
  'T1 Вода, °C', 'T2 Колонна, °C', 'T3 Куб, °C', 'T4 Верх, °C',
  'Ошибка T1', 'Ошибка T2', 'Ошибка T3', 'Ошибка T4',
  'PCF8575 онлайн', 'Ошибка клавиатуры', 'I2C адреса', 'PCA9685 онлайн',
  'YF-S401 импульсы', 'YF-S401 калиброван', 'Wi-Fi STA', 'Свободная память',
  'Общий статус', 'Диагностика (JSON)'
];

/**
 * RUN ONCE (with your own Google account). Then open
 * Apps Script > Project Settings > Script properties to copy BUSA_TOKEN.
 * Secret is not logged or saved in the table.
 */
function setupBusaSheet() {
  const sheet = SpreadsheetApp.getActiveSpreadsheet();
  if (!sheet) throw new Error('Create this script from Google Sheets > Extensions.');
  const cfg = PropertiesService.getScriptProperties();
  cfg.setProperty('BUSA_SPREADSHEET_ID', sheet.getId());
  if (!cfg.getProperty('BUSA_TOKEN')) {
    const token = Utilities.getUuid().replace(/-/g, '') +
                  Utilities.getUuid().replace(/-/g, '');
    cfg.setProperty('BUSA_TOKEN', token);
  }
  const tab = sheet.getSheetByName(BUSA_TAB) || sheet.insertSheet(BUSA_TAB);
  if (tab.getLastRow() === 0) {
    tab.appendRow(BUSA_HEADERS);
    tab.setFrozenRows(1);
  }
}

function doPost(e) {
  try {
    if (!e || !e.postData || !e.postData.contents ||
        e.postData.contents.length > 12000) {
      return busaJson_({stored: false, error: 'Invalid request'});
    }
    const data = JSON.parse(e.postData.contents);
    const cfg = PropertiesService.getScriptProperties();
    const expected = cfg.getProperty('BUSA_TOKEN');
    if (!expected || typeof data.token !== 'string' ||
        data.token !== expected || data.device_id !== 'BUSA-1M-001' ||
        data.kind !== 'monitor_diagnostics' || data.version !== 3) {
      return busaJson_({stored: false, error: 'Unauthorized'});
    }
    const payload = data.telemetry;
    if (!payload || typeof payload !== 'object' ||
        payload.device !== 'BUSA-1M-001') {
      return busaJson_({stored: false, error: 'Malformed telemetry'});
    }
    const id = cfg.getProperty('BUSA_SPREADSHEET_ID');
    if (!id) throw new Error('Run setupBusaSheet once');
    const temp = Array.isArray(payload.temperatures) ? payload.temperatures : [];
    const diag = payload.diagnostics || {};
    const oneWire = Array.isArray(diag.onewire) ? diag.onewire : [];
    const keyboard = diag.keyboard || {};
    const modules = payload.modules || {};
    const water = payload.water || {};
    const wifi = diag.wifi || {};
    const pca = diag.pca9685 || {};
    const chip = diag.esp32 || {};
    const clock = payload.clock || {};
    const errs = [0,1,2,3].map(i => String(oneWire[i]?.result || 'NO_DATA'));
    const hasErrors = errs.some(x => x !== 'OK') ||
      Number(keyboard.scan_error || 0) !== 0 ||
      !Boolean(modules.pcf8575) || !Boolean(pca.ack);
    const row = [
      new Date(), payload.device, String(payload.firmware || ''),
      String(clock.datetime || ''),
      ...[0,1,2,3].map(i => temp[i] && typeof temp[i].c === 'number' ? temp[i].c : ''),
      ...errs,
      Boolean(modules.pcf8575), Number(keyboard.scan_error || 0),
      (diag.i2c?.addresses || []).join(','),
      Boolean(pca.ack),
      Number(water.pulses || 0), Boolean(water.calibrated),
      Boolean(wifi.sta_connected),
      Number(chip.free_heap_bytes || 0),
      hasErrors ? 'ТРЕБУЕТ ПРОВЕРКИ' : 'ДАННЫЕ ПОЛУЧЕНЫ',
      JSON.stringify(payload)
    ];
    const lock = LockService.getScriptLock();
    if (!lock.tryLock(10000)) throw new Error('Receiver busy');
    try {
      const sheet = SpreadsheetApp.openById(id);
      const tab = sheet.getSheetByName(BUSA_TAB) || sheet.insertSheet(BUSA_TAB);
      if (tab.getLastRow() === 0) { tab.appendRow(BUSA_HEADERS); tab.setFrozenRows(1); }
      tab.appendRow(row);
    } finally {
      lock.releaseLock();
    }
    return busaJson_({stored: true});
  } catch (err) {
    // Do not log raw requests: they contain the token.
    return busaJson_({stored: false, error: String(err.message || 'Receiver error')});
  }
}

function busaJson_(obj) {
  return ContentService.createTextOutput(JSON.stringify(obj))
    .setMimeType(ContentService.MimeType.JSON);
}
