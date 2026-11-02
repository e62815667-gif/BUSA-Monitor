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
  // Independent administrator secret; never place in the published page or ESP32.
  if (!cfg.getProperty('BUSA_ADMIN_TOKEN')) {
    const token = Utilities.getUuid().replace(/-/g, '') +
                  Utilities.getUuid().replace(/-/g, '');
    cfg.setProperty('BUSA_ADMIN_TOKEN', token);
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
    // Acknowledgement is sent in the next authenticated ESP32 telemetry POST.
    // One pending command at a time; expiry prevents stale commands after outages.
    const ack=payload.cloud_control_ack;
    const props=PropertiesService.getScriptProperties();
    let pending=null;
    try{pending=JSON.parse(props.getProperty('BUSA_PENDING_COMMAND')||'null')}catch(ignore){}
    if(ack && pending && ack.id===pending.id){
      props.setProperty('BUSA_LAST_COMMAND_RESULT',JSON.stringify({
        id:pending.id,kind:pending.kind,result:String(ack.result||'received').slice(0,80),at:Date.now()
      }));
      props.deleteProperty('BUSA_PENDING_COMMAND');
      pending=null;
    }
    if(pending && Date.now()-pending.created>600000){
      props.setProperty('BUSA_LAST_COMMAND_RESULT',JSON.stringify({
        id:pending.id,kind:pending.kind,result:'expired',at:Date.now()
      }));
      props.deleteProperty('BUSA_PENDING_COMMAND');
      pending=null;
    }
    return busaJson_({stored:true,command:pending||null});
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
    if(q.mode==='adminCommand'||q.mode==='adminStatus'){
      return busaAdminCommand_(q);
    }
    const expected = PropertiesService.getScriptProperties().getProperty('BUSA_VIEW_TOKEN');
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


/**
 * The cloud administrator key is NEVER the BUSA_TOKEN or BUSA_VIEW_TOKEN.
 * Each command is explicitly requested and expires after 10 minutes.
 * The ESP32 receives commands only inside its authenticated HTTPS telemetry
 * response and validates every field again before changing any local setting.
 */
function busaAdminCommand_(q){
  const fixed='busaAdminReceive';
  try {
    const cfg=PropertiesService.getScriptProperties();
    const secret=cfg.getProperty('BUSA_ADMIN_TOKEN');
    if(!secret||typeof q.adminToken!=='string'||!/^[a-fA-F0-9]{64}$/.test(q.adminToken)||
       q.adminToken!==secret||(q.callback&&q.callback!==fixed)){
      return busaAdminScript_({ok:false,error:'Требуется отдельный ключ администратора'});
    }
    const lock=LockService.getScriptLock();
    if(!lock.tryLock(8000))return busaAdminScript_({ok:false,error:'Команда занята'});
    try{
      let pending=null;
      try{pending=JSON.parse(cfg.getProperty('BUSA_PENDING_COMMAND')||'null')}catch(ignore){}
      if(pending && Date.now()-pending.created>600000){
        cfg.deleteProperty('BUSA_PENDING_COMMAND');pending=null;
      }
      let last=null;
      try{last=JSON.parse(cfg.getProperty('BUSA_LAST_COMMAND_RESULT')||'null')}catch(ignore){}
      if(q.mode==='adminStatus')return busaAdminScript_({ok:true,pending:pending?{
        id:pending.id,kind:pending.kind,created:pending.created
      }:null,last:last||null});
      if(pending)return busaAdminScript_({ok:false,error:'Предыдущая команда ещё не подтверждена'});
      const kind=String(q.kind||'');
      const cmd={id:Utilities.getUuid(),kind,created:Date.now()};
      const finite=(v,min,max)=>{const n=Number(v);if(!Number.isFinite(n)||String(v).trim()===''||n<min||n>max)throw new Error('Значение вне диапазона');return n};
      if(kind==='threshold'){
        cmd.index=finite(q.index,0,3);
        if(!Number.isInteger(cmd.index))throw new Error('Номер датчика');
        cmd.on=finite(q.on,0,125);cmd.off=finite(q.off,0,125);
        if(Math.abs(cmd.on-cmd.off)<0.1)throw new Error('Пороги должны различаться');
      }else if(kind==='calibration'){
        cmd.index=finite(q.index,0,3);
        if(!Number.isInteger(cmd.index))throw new Error('Номер датчика');
        cmd.offset=finite(q.offset,-10,10);
      }else if(kind==='theme'){
        cmd.theme=finite(q.theme,0,2);
        if(!Number.isInteger(cmd.theme))throw new Error('Номер темы');
      }else if(kind==='ota'){
        // Only the dedicated public owner-controlled GitHub Pages release path.
        const allowed='https://e62815667-gif.github.io/BUSA-Monitor/online/firmware/BUSA_APP_OTA.bin';
        if(q.url!==allowed)throw new Error('Неверный адрес прошивки');
        if(!/^[a-fA-F0-9]{64}$/.test(String(q.sha256||'')))throw new Error('SHA-256');
        cmd.size=finite(q.size,60000,3000000);
        if(!Number.isInteger(cmd.size))throw new Error('Размер BIN');
        cmd.url=allowed;cmd.sha256=q.sha256.toLowerCase();
        cmd.version=String(q.version||'').slice(0,100);
        if(!/^14v7-N16R8-/.test(cmd.version))throw new Error('Версия не совместима');
      }else throw new Error('Недопустимая команда');
      cfg.setProperty('BUSA_PENDING_COMMAND',JSON.stringify(cmd));
      return busaAdminScript_({ok:true,queued:{id:cmd.id,kind:cmd.kind}});
    }finally{lock.releaseLock();}
  }catch(err){return busaAdminScript_({ok:false,error:String(err.message||'Ошибка')});}
}
function busaAdminScript_(obj){
  return ContentService.createTextOutput('busaAdminReceive('+JSON.stringify(obj)+');')
    .setMimeType(ContentService.MimeType.JAVASCRIPT);
}
