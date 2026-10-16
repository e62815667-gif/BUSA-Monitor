const CACHE='busa-monitor-v072-lab1';
const ASSETS=['./','./index.html','./lab-core.js','./lab-app.js','./manifest.webmanifest'];
self.addEventListener('install',e=>e.waitUntil(caches.open(CACHE).then(c=>c.addAll(ASSETS)).then(()=>self.skipWaiting())));
self.addEventListener('activate',e=>e.waitUntil(caches.keys().then(keys=>Promise.all(keys.filter(k=>k.startsWith('busa-monitor-')&&k!==CACHE).map(k=>caches.delete(k)))).then(()=>self.clients.claim())));
self.addEventListener('fetch',e=>{
 const u=new URL(e.request.url);
 // Never cache or synthesize any API response, including errors.
 if(e.request.method!=='GET'||u.origin!==self.location.origin||u.pathname.includes('/api/'))return;
 if(!ASSETS.some(p=>new URL(p,self.location.href).href===u.href))return;
 e.respondWith(fetch(e.request).then(r=>{if(r.ok){const copy=r.clone();e.waitUntil(caches.open(CACHE).then(c=>c.put(e.request,copy)));}return r;}).catch(()=>caches.match(e.request).then(r=>r||Response.error())));
});

