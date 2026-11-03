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
  // READ-only key is different from the device's WRITE key. Never embed
  // either key into the public GitHub Pages HTML.
  if (!cfg.getProperty('BUSA_VIEW_TOKEN')) {
    const token = Utilities.getUuid().replace(/-/g, '') +
                  Utilities.getUuid().replace(/-/g, '');
    cfg.setProperty('BUSA_VIEW_TOKEN', token);
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
    if (data.kind === 'alarm_threshold_command') {
      return busaQueueAlarmCommand_(data, cfg);
    }
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
      // The device's next authenticated telemetry confirms an applied command.
      const ack = payload.remote_alarm;
      if (ack && typeof ack.id === 'string' && /^[a-f0-9]{24}$/.test(ack.id)) {
        const raw = cfg.getProperty('BUSA_REMOTE_COMMAND');
        if (raw) {
          try {
            const cmd = JSON.parse(raw);
            if (cmd.id === ack.id && (ack.state === 'applied' || ack.state === 'failed')) {
              let verified = ack.state === 'applied';
              if (verified) {
                const ref = Array.isArray(payload.display_references) ? payload.display_references : [];
                verified = ref.length === 4 && cmd.on.every((n,i) =>
                  typeof ref[i].target === 'number' && Math.abs(ref[i].target - n) < 0.051 &&
                  typeof ref[i].notice === 'number' && Math.abs(ref[i].notice - cmd.off[i]) < 0.051);
              }
              cfg.setProperty('BUSA_REMOTE_STATUS', JSON.stringify({
                id:cmd.id, state:verified?'applied':'failed',
                updated:Date.now(), detail:verified?'Confirmed by ESP32 telemetry':'Device save not confirmed'
              }));
              cfg.deleteProperty('BUSA_REMOTE_COMMAND');
            }
          } catch (_) { /* Leave the pending command for expiry. */ }
        }
      }
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


/**
 * Remote configuration of SOUND NOTIFICATION thresholds only.
 * The VIEW token authorizes configuration; the ESP32 TOKEN is used solely
 * to retrieve a queued command. No remote relay, actuator or GPIO command.
 */
function busaCommandStatus_(cfg) {
  const raw = cfg.getProperty('BUSA_REMOTE_STATUS');
  if (!raw) return null;
  try {
    const state=JSON.parse(raw), queued=cfg.getProperty('BUSA_REMOTE_COMMAND');
    if (state.state==='queued' && queued && Date.now()-state.updated>300000) {
      state.state='expired'; state.detail='ESP32 has not confirmed within 5 minutes';
      cfg.setProperty('BUSA_REMOTE_STATUS',JSON.stringify(state));
      cfg.deleteProperty('BUSA_REMOTE_COMMAND');
    }
    return {id:state.id,state:state.state,updated:state.updated,detail:state.detail||''};
  } catch (_) { return null; }
}
function busaQueueAlarmCommand_(data,cfg) {
  const expected=cfg.getProperty('BUSA_VIEW_TOKEN');
  if (!expected || data.token !== expected || data.device_id!=='BUSA-1M-001' ||
      data.version!==1 || !/^[a-f0-9]{24}$/.test(String(data.id||''))) {
    return busaJson_({ok:false,error:'Unauthorized configuration request'});
  }
  const on=data.on,off=data.off;
  const valid=n=>typeof n==='number'&&Number.isFinite(n)&&n>=0&&n<=125&&
                 Math.abs(n*10-Math.round(n*10))<0.0001;
  if (!Array.isArray(on)||!Array.isArray(off)||on.length!==4||off.length!==4||
      !on.every(valid)||!off.every(valid)||
      !on.every((n,i)=>Math.abs(n-off[i])>=0.099)) {
    return busaJson_({ok:false,error:'Invalid sound thresholds'});
  }
  const lock=LockService.getScriptLock();
  if (!lock.tryLock(5000))return busaJson_({ok:false,error:'Busy'});
  try {
    const state=busaCommandStatus_(cfg);
    if (state?.state==='queued' && state.id!==data.id) {
      return busaJson_({ok:false,error:'Another configuration is pending'});
    }
    if (state?.id===data.id)return busaJson_({ok:true,state:state.state,id:data.id});
    cfg.setProperty('BUSA_REMOTE_COMMAND',JSON.stringify({
      id:data.id,on:on,off:off,created:Date.now()
    }));
    cfg.setProperty('BUSA_REMOTE_STATUS',JSON.stringify({
      id:data.id,state:'queued',updated:Date.now(),detail:'Waiting for ESP32'
    }));
    return busaJson_({ok:true,id:data.id,state:'queued'});
  } finally {lock.releaseLock();}
}

/**
 * Read-only live mobile dashboard feed. JSONP is used because an HTTPS GitHub
 * Pages website cannot directly contact the local HTTP ESP32, while Google
 * Apps Script ContentService cross-origin fetch does not reliably return CORS
 * headers. Scripts are loaded only from a user-entered script.google.com URL.
 *
 * Auth: SECOND, read-only BUSA_VIEW_TOKEN. NEVER use BUSA_TOKEN in the app.
 * Endpoint: GET /exec?mode=latest&readToken=<64 hex chars>&callback=busaViewReceive
 * Optional: callback is ALWAYS fixed; arbitrary JavaScript is forbidden.
 */
function doGet(e) {
  const callback = 'busaViewReceive';
  try {
    const q = (e && e.parameter) || {};
    const cfg = PropertiesService.getScriptProperties();
    if (q.mode === 'device_command') {
      const deviceKey=cfg.getProperty('BUSA_TOKEN');
      if (!deviceKey || q.deviceToken!==deviceKey || !/^[a-fA-F0-9]{64}$/.test(q.deviceToken||'')) {
        return busaJson_({ok:false,error:'Unauthorized'});
      }
      const status=busaCommandStatus_(cfg);
      const raw=cfg.getProperty('BUSA_REMOTE_COMMAND');
      let pending=null;
      if (raw && status?.state==='queued') {
        try {
          const c=JSON.parse(raw);
          pending={id:c.id,on:c.on,off:c.off};
        } catch (_) {}
      }
      return busaJson_({ok:true,command:pending});
    }
    const expected = cfg.getProperty('BUSA_VIEW_TOKEN');
    if (!expected || typeof q.readToken !== 'string' ||
        !/^[a-fA-F0-9]{64}$/.test(q.readToken) ||
        q.readToken !== expected || q.mode !== 'latest' ||
        (q.callback && q.callback !== callback)) {
      return busaScript_({ok:false,error:'Доступ к телеметрии не разрешён'});
    }
    const id = PropertiesService.getScriptProperties().getProperty('BUSA_SPREADSHEET_ID');
    if (!id) return busaScript_({ok:false,error:'Таблица не настроена'});
    const sheet=SpreadsheetApp.openById(id).getSheetByName(BUSA_TAB);
    if (!sheet || sheet.getLastRow()<2) {
      return busaScript_({ok:true,latest:null,history:[],message:'Ещё нет записей'});
    }
    // Fetch a short time-bounded sample only, never an entire archive.
    const end = sheet.getLastRow();
    const start = Math.max(2,end-29);
    const rows = sheet.getRange(start,1,end-start+1,BUSA_HEADERS.length).getValues();
    const sample = [];
    for(let i=0;i<rows.length;i++){
      const row=rows[i];
      const raw=row[21]; // detailed telemetry is stored in column 22
      if (typeof raw !== 'string' || raw.length>10000) continue;
      try{
        const packet=JSON.parse(raw);
        if (!packet || packet.device!=='BUSA-1M-001') continue;
        const stamp=row[0] instanceof Date?row[0].getTime():null;
        sample.push({at:stamp,telemetry:packet});
      }catch(ignore){}
    }
    return busaScript_({
      ok:true,
      latest:sample.length?sample[sample.length-1]:null,
      command_status:busaCommandStatus_(PropertiesService.getScriptProperties()),
      history:sample.map(v=>({
        at:v.at,
        temperatures:(Array.isArray(v.telemetry.temperatures)?
           v.telemetry.temperatures.map(t=>typeof t.c==='number'?t.c:null):[null,null,null,null]),
        pulses:v.telemetry.water && typeof v.telemetry.water.pulses==='number'?
           v.telemetry.water.pulses:null
      }))
    });
  } catch(err) {
    return busaScript_({ok:false,error:'Не удалось прочитать таблицу'});
  }
}
function busaScript_(data) {
  // Fixed callback prevents script injection through callback parameters.
  const js='busaViewReceive('+JSON.stringify(data)+');';
  return ContentService.createTextOutput(js)
    .setMimeType(ContentService.MimeType.JAVASCRIPT);
}
