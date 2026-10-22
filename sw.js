const CACHE="busa-monitor-v14.14-approved-six-screens-20261023";
const ASSETS=["./","./index.html","./manifest.webmanifest"];
self.addEventListener("install",event=>{
  event.waitUntil(caches.open(CACHE).then(cache=>cache.addAll(
    ASSETS.map(url=>new Request(url,{cache:"reload"}))
  )).then(()=>self.skipWaiting()));
});
self.addEventListener("activate",event=>{
  event.waitUntil(caches.keys().then(keys=>Promise.all(
    keys.filter(key=>key.startsWith("busa-monitor-")&&key!==CACHE)
        .map(key=>caches.delete(key))
  )).then(()=>self.clients.claim()));
});
self.addEventListener("fetch",event=>{
  const req=event.request;
  if(req.method!=="GET"||new URL(req.url).origin!==self.location.origin)return;
  event.respondWith(fetch(req,{cache:"no-store"}).then(response=>{
    if(response.ok){const clone=response.clone();event.waitUntil(
      caches.open(CACHE).then(cache=>cache.put(req,clone)))}
    return response;
  }).catch(async()=>{
    const cached=await caches.match(req);
    if(cached)return cached;
    if(req.mode==="navigate")return await caches.match("./index.html")||Response.error();
    return Response.error();
  }));
});