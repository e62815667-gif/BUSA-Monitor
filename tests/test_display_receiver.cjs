const fs=require('node:fs');const vm=require('node:vm');const assert=require('node:assert/strict');
const ctx=vm.createContext({});
vm.runInContext(fs.readFileSync('google/BUSA_GoogleSheets_Receiver.gs','utf8'),ctx);
const result=vm.runInContext(`
  (() => {
    const p={device:'BUSA-1M-001',firmware:'main',display_diagnostics:{
      firmware:'BUSA-DISPLAY-1',link:'CONNECTED',age_ms:15,uptime_ms:0,
      reset_reason:1,cpu_mhz:240,free_ram:0,min_ram:100,psram_total:8388608,
      fps_milli:29970,sd_total_bytes:8589934592,sd_free_bytes:0,touch_errors:null
    }};
    return {row:busaDisplayRow_(p,1234),headers:BUSA_DISPLAY_HEADERS,
      old:busaDisplayRow_({device:p.device},1234),
      invalid:busaDisplayRow_({...p,display_diagnostics:{firmware:'=1+1',link:'fake',free_ram:-1}},1234)};
  })()
`,ctx);
assert.equal(result.headers.length,34);assert.equal(result.row.length,34);
assert.equal(result.row[0],1234);assert.equal(result.row[3],'BUSA-DISPLAY-1');
assert.equal(result.row[4],'CONNECTED');assert.equal(result.row[6],0);
assert.equal(result.row[9],0);assert.equal(result.row[22],29.97);
assert.equal(result.row[24],'');assert.equal(result.row[26],8589934592);
assert.equal(result.row[27],0);assert.equal(result.old,null);
assert.equal(result.invalid[3],'');assert.equal(result.invalid[4],'UNKNOWN');assert.equal(result.invalid[9],'');
console.log('PASS display telemetry mapping and unknown values');
