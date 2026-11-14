/* Read-only presentation bindings; controller API and calculator stay separate. */
(()=>{
 'use strict';
 const pairs=[0,2,1,3].map(i=>[document.getElementById('t'+i),document.querySelector('[data-legend-temp="'+i+'"]')]);
 const sync=()=>{for(const [source,target] of pairs)if(source&&target&&target.textContent!==source.textContent)target.textContent=source.textContent;};
 const observer=new MutationObserver(sync);
 for(const [source] of pairs)if(source)observer.observe(source,{childList:true,characterData:true,subtree:true});
 sync();
})();
