/* MOTION1: passive presentation, using the existing measured-data classes. */
(()=>{
 'use strict';
 const stage=document.getElementById('rigArtStage');
 const svg=stage.querySelector('.waterMotion');
 svg.setAttribute('viewBox','0 0 1024 1536');
 svg.innerHTML=`<defs>
  <clipPath id="homeCubeLiquidClip"><path d="M255 1316H558V1392L529 1427H284L255 1392Z"/></clipPath>
  <clipPath id="homeColumnClip"><path d="M353 682H455V784H353ZM354 888H453V944H354Z"/></clipPath>
  <clipPath id="homeSmallGlassClip"><rect x="687" y="640" width="48" height="68"/></clipPath>
 </defs>
 <path class="waterTrace drain" data-motion="water-left" d="M18 1144H76Q142 1144 142 1076V331Q142 188 283 188H517"/>
 <path class="waterTrace transfer" data-motion="water-link" d="M684 367H878Q951 367 951 442V689Q951 759 881 759H787"/>
 <path class="waterTrace inlet" data-motion="water-out" d="M788 913H1003"/>
 <path class="coilTrace" data-motion="coil" d="M695 782C753 783 754 793 695 801S638 814 695 821S754 833 695 841S638 853 695 861S754 873 695 881S638 894 695 902S754 914 695 925"/>
 <path class="productTrace" d="M718 995L748 1256"/>
 <path class="columnFilm" data-motion="column" clip-path="url(#homeColumnClip)" d="M359 682V784M449 682V784M359 888V944M448 888V944"/>
 <path class="capillaryDrop" data-motion="small-glass" clip-path="url(#homeSmallGlassClip)" d="M711 640V708"/>
 <path class="cubeConvection" data-motion="cube-convection" clip-path="url(#homeCubeLiquidClip)" d="M298 1423C265 1380 307 1350 356 1320M515 1322C555 1360 514 1404 465 1425M383 1424C423 1390 358 1355 409 1320"/>
 <path class="cubeSurfaceMotion" data-motion="cube-surface" d="M268 1310C330 1299 477 1299 546 1310C478 1321 331 1321 268 1310Z"/>`;
 let coil='M688 783';
 for(let i=0;i<7;i++){
  const y=783+i*22;
  coil+='C751 '+(y-3)+' 751 '+(y+9)+' 688 '+(y+15)+'C672 '+(y+17)+' 672 '+(y+20)+' 688 '+(y+22);
 }
 svg.querySelector('.coilTrace').setAttribute('d',coil);
 stage.querySelector('.approvedRigArt').alt='БУСА: утверждённый рисунок главного экрана';
 stage.dataset.homeMotion='1';
 const syncVisibility=()=>stage.classList.toggle('homeMotionPaused',document.hidden);
 document.addEventListener('visibilitychange',syncVisibility);
 syncVisibility();
})();
