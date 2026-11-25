const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const rows=[];const cfg={getProperty:k=>({BUSA_TOKEN:'test',BUSA_SPREADSHEET_ID:'sheet'}[k]||''),setProperty(){},deleteProperty(){}};
const tab={getLastRow:()=>1,appendRow:r=>rows.push(r)};
const context=vm.createContext({PropertiesService:{getScriptProperties:()=>cfg},SpreadsheetApp:{openById:()=>({getSheetByName:()=>tab})},LockService:{getScriptLock:()=>({tryLock:()=>true,releaseLock(){}})},ContentService:{MimeType:{JSON:'json'},createTextOutput:s=>({setMimeType:()=>JSON.parse(s)})}});
vm.runInContext(fs.readFileSync('google/BUSA_GoogleSheets_Receiver.gs','utf8'),context);
const post=pcf=>vm.runInContext('doPost('+JSON.stringify({postData:{contents:JSON.stringify({kind:'monitor_diagnostics',device_id:'BUSA-1M-001',version:3,token:'test',telemetry:{device:'BUSA-1M-001',temperatures:[],modules:{pcf8575:pcf},diagnostics:{onewire:[0,1,2,3].map(()=>({result:'OK'})),keyboard:{scan_error:0}}}})}})+')',context);
assert.equal(post(true).stored,true);assert.equal(rows[0].length,22);assert.equal(rows[0][15],'');assert.equal(rows[0][20],'ДАННЫЕ ПОЛУЧЕНЫ');
assert.equal(post(false).stored,true);assert.equal(rows[1][20],'ТРЕБУЕТ ПРОВЕРКИ');
console.log('PASS: receiver ignores absent module while retaining historical column and real keyboard errors');
