const CACHE='busa-monitor-calc1-20261105';
const ASSETS=['./','./index.html','./manifest.webmanifest','./calculator.js?v=CALC1','./calculator.css?v=CALC1'];
self.addEventListener('install',event=>event.waitUntil(caches.open(CACHE).then(cache=>cache.addAll(ASSETS)).then(()=>self.skipWaiting())));
self.addEventListener('activate',event=>event.waitUntil(caches.keys().then(keys=>Promise.all(keys.filter(key=>key.startsWith('busa-monitor-')&&key!==CACHE).map(key=>caches.delete(key)))).then(()=>self.clients.claim())));
self.addEventListener('fetch',event=>{
 const req=event.request;if(req.method!=='GET')return;
 const url=new URL(req.url);if(url.origin!==self.location.origin)return;
 // Never cache controller APIs or mistake a calculated number for telemetry.
 if(url.pathname.includes('/api/'))return;
 event.respondWith((async()=>{
  const cache=await caches.open(CACHE);
  try{const response=await fetch(req);if(response.ok)await cache.put(req,response.clone());return response;}
  catch(e){const hit=await cache.match(req);if(hit)return hit;if(req.mode==='navigate'){const page=await cache.match('./index.html');if(page)return page;}return Response.error();}
 })());
});
