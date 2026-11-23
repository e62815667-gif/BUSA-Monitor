"""Exercise shipping home motion, data loss and immutable live artwork."""
from pathlib import Path
import copy, json, os, shutil, urllib.parse
from playwright.sync_api import sync_playwright

root=Path(__file__).resolve().parents[1]
source=(root/'firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h').read_text()
html=source.split('R"HTMLBUSA(',1)[1].split(')HTMLBUSA"',1)[0]
base=json.loads((root/'tests/fixtures/ui10-status.json').read_text())
base['collection']={'sensor_present':True,'ml_per_min':24,'total_ml':1234,'duration_ms':3600000}
base['water']['calibrated']=True
roles=['water-left','water-link','water-out','coil','column','small-glass','cube-convection','cube-surface']
shots=Path(os.environ.get('BUSA_BROWSER_ARTIFACTS',str(root.parent/'home-motion-screens')))
shots.mkdir(parents=True,exist_ok=True)

with sync_playwright() as p:
    browser=(p.webkit.launch() if os.environ.get('BUSA_TEST_BROWSER')=='webkit' else
             p.chromium.launch(executable_path=shutil.which('chromium'),args=['--no-sandbox']))
    for width,height in [(320,568),(390,780),(844,390)]:
        page=browser.new_page(viewport={'width':width,'height':height},locale='ru-RU')
        errors=[];posts=[];offline=False;tick=0;fixture=copy.deepcopy(base);motion_problems=[]
        page.on('pageerror',lambda e:errors.append(str(e)))
        def route(r):
            global tick
            u=urllib.parse.urlsplit(r.request.url)
            if r.request.method=='POST':posts.append(u.path)
            if u.path=='/':r.fulfill(content_type='text/html',body=html)
            elif u.path in ['/calculator.js','/calculator.css']:r.fulfill(body=(root/u.path[1:]).read_bytes())
            elif u.path=='/api/status' and not offline:
                tick+=1;fixture['uptime_ms']=10000+tick*2000
                fixture['water']['total_l_estimate']=12.32+tick*.04
                r.fulfill(content_type='application/json',body=json.dumps(fixture))
            elif u.path=='/api/web-events':r.fulfill(content_type='application/json',body='{"boot_id":1,"events":[]}')
            else:r.fulfill(status=503,body='{}')
        page.route('**/*',route);page.goto('http://busa-motion.test/')
        page.wait_for_function('connected && !pollBusy && document.documentElement.dataset.viewportReady==="1"')
        page.evaluate('update()')
        page.wait_for_function('document.getElementById("rigArtStage").classList.contains("hasCooling")')
        posts.clear()  # Startup clock synchronization belongs to the existing controller.
        assert page.locator('#rigArtStage[data-home-motion="1"]').count()==1,'Required home motion is absent'
        for role in roles:
            node=page.locator('[data-motion="'+role+'"]')
            assert node.count()==1,role
            assert node.evaluate('e=>getComputedStyle(e).animationPlayState')=='running',role
        for selector in ['[data-motion="water-left"]','[data-motion="coil"]','.productTrace']:
            loop=page.locator(selector).evaluate(r'''e=>{
                const period=getComputedStyle(e).strokeDasharray.split(/[,\s]+/).reduce((s,n)=>s+parseFloat(n),0);
                const frames=e.getAnimations()[0].effect.getKeyframes();
                const delta=parseFloat(frames[frames.length-1].strokeDashoffset)-parseFloat(frames[0].strokeDashoffset);
                return {period,delta};
            }''')
            if abs(loop['delta']/loop['period']-round(loop['delta']/loop['period']))>.001:
                motion_problems.append(('loop seam',selector,loop))
        page.evaluate('Object.defineProperty(document,"hidden",{value:true,configurable:true});document.dispatchEvent(new Event("visibilitychange"))')
        if page.locator('[data-motion="water-left"]').evaluate('e=>getComputedStyle(e).animationPlayState')!='paused':
            motion_problems.append('Motion continues in a background tab')
        page.evaluate('Object.defineProperty(document,"hidden",{value:false,configurable:true});document.dispatchEvent(new Event("visibilitychange"))')
        image=page.locator('.approvedRigArt')
        assert image.evaluate('e=>e.complete && e.naturalWidth===1024')
        page.evaluate('window.motionArt=document.querySelector(".approvedRigArt");window.artChanges=[];new MutationObserver(rs=>artChanges.push(...rs.map(r=>r.type))).observe(motionArt,{attributes:true,childList:true,subtree:true})')
        if width==390:
            # Freeze animation times explicitly; compare the same live frame at two phases.
            def frame(ms):
                page.evaluate('(ms)=>document.getElementById("rigArtStage").getAnimations({subtree:true}).forEach(a=>{a.pause();a.currentTime=ms})',ms)
                box=page.locator('#rigArtStage').bounding_box()
                metal={'x':box['x']+box['width']*.55,'y':box['y']+box['height']*.15,
                       'width':box['width']*.05,'height':box['height']*.15}
                return page.locator('#rigArtStage').screenshot(),page.screenshot(clip=metal)
            a=frame(150);b=frame(1250)
            assert a[0]!=b[0],'No visible fluid motion'
            assert a[1]==b[1],'Metal body flickers'
            page.screenshot(path=str(shots/'home-active.png'))
        for _ in range(3):page.evaluate('update()')
        page.wait_for_timeout(60)
        assert page.evaluate('motionArt===document.querySelector(".approvedRigArt") && artChanges.length===0')
        page.locator('[data-page="1"]').click()
        for role in roles:assert page.locator('[data-motion="'+role+'"]') .evaluate('e=>getComputedStyle(e).animationPlayState')=='paused',role
        page.locator('[data-page="0"]').click()
        page.emulate_media(reduced_motion='reduce')
        for role in roles:assert page.locator('[data-motion="'+role+'"]') .evaluate('e=>getComputedStyle(e).animationName')=='none',role
        page.emulate_media(reduced_motion='no-preference')
        offline=True
        for _ in range(3):page.evaluate('update()')
        page.wait_for_timeout(60)
        for role in roles:
            node=page.locator('[data-motion="'+role+'"]')
            assert node.evaluate('e=>getComputedStyle(e).animationPlayState')=='paused',role
            assert float(node.evaluate('e=>getComputedStyle(e).opacity'))==0,role
        assert page.locator('#t2').inner_text()=='Н/Д'
        for container in ['.collectionLiquid','.boilerBoil']:
            if float(page.locator(container).evaluate('e=>getComputedStyle(e).opacity'))!=0:
                motion_problems.append(('liquid effects remain visible offline',container))
            for child in page.locator(container).locator(':scope > *').all():
                if child.evaluate('e=>getComputedStyle(e).animationName')!='none' and child.evaluate('e=>getComputedStyle(e).animationPlayState')!='paused':
                    motion_problems.append(('liquid motion continues offline',container))
        assert not motion_problems,motion_problems
        assert not errors,errors
        assert not posts,posts
        page.close()
    browser.close()
print('PASS home motion: 8 layers, visible motion, stable metal/artwork, hidden/offline/reduced states, no device writes')
