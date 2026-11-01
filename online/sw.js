const CACHE='busa-monitor-v15.6-module-bootfix';
self.addEventListener('install',e=>{e.waitUntil(caches.open(CACHE).then(c=>c.addAll(['./','./index.html','./manifest.webmanifest'])).then(()=>self.skipWaiting()));});
self.addEventListener('activate',e=>{e.waitUntil(caches.keys().then(keys=>Promise.all(keys.filter(k=>k.startsWith('busa-monitor-')&&k!==CACHE).map(k=>caches.delete(k)))).then(()=>self.clients.claim()));});
self.addEventListener('fetch',e=>{
 const r=e.request,u=new URL(r.url);if(r.method!=='GET'||u.origin!==self.location.origin)return;
 // Never cache telemetry, API replies or release catalogues.
 if(u.search||u.pathname.includes('/api/')||u.pathname.endsWith('.json'))return;
 e.respondWith(fetch(r).then(response=>{if(response.ok){const copy=response.clone();e.waitUntil(caches.open(CACHE).then(c=>c.put(r,copy)));}return response;}).catch(()=>caches.match(r).then(v=>v||Response.error())));
});
