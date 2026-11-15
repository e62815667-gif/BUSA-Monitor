"""Approved design runs the real local UI; fixtures are not device measurements."""
from pathlib import Path
import copy,hashlib,json,os,re,shutil,subprocess,urllib.parse
from playwright.sync_api import sync_playwright

root=Path(__file__).resolve().parents[1]
source=(root/'firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h').read_text()
html=source.split('R"HTMLBUSA(',1)[1].split(')HTMLBUSA"',1)[0]
for js in re.findall(r'<script>(.*?)</script>',html,re.S):
    subprocess.run(['node','--check'],input=js.encode(),check=True)
base=json.loads((root/'tests/fixtures/ui10-status.json').read_text())
base['collection']={'sensor_present':True,'ml_per_min':24,'total_ml':1234,'duration_ms':3600000}
base['water']['calibrated']=True
base['sound_alerts']['channels'][1]['sound_on']=True
base['sound_alerts']['channels'][3]['sound_on']=False
shots=Path(os.environ.get('BUSA_BROWSER_ARTIFACTS',str(root.parent/'ui16/screens')))
shots.mkdir(parents=True,exist_ok=True)
names={0:'home',1:'temperatures',2:'water',3:'graphs',7:'calculator',5:'settings'}

with sync_playwright() as p:
    browser=(p.webkit.launch() if os.environ.get('BUSA_TEST_BROWSER')=='webkit' else
             p.chromium.launch(executable_path=shutil.which('chromium'),args=['--no-sandbox']))
    for width,height in [(320,568),(375,600),(375,667),(390,636),(390,744),(390,780),(430,932),(844,390)]:
        fixture=copy.deepcopy(base);offline=False;posts=[];errors=[];tick=0
        page=browser.new_page(viewport={'width':width,'height':height},locale='ru-RU')
        page.on('pageerror',lambda e:errors.append(str(e)))
        def route(r):
            global tick
            u=urllib.parse.urlsplit(r.request.url)
            if r.request.method=='POST':posts.append(u.path)
            if u.path=='/':r.fulfill(content_type='text/html',body=html)
            elif u.path in ['/calculator.css','/calculator.js']:
                r.fulfill(body=(root/u.path[1:]).read_bytes())
            elif u.path=='/api/status' and not offline:
                tick+=1;fixture['uptime_ms']=10000+tick*2000
                fixture['water']['total_l_estimate']=12.32+tick*.04
                r.fulfill(content_type='application/json',body=json.dumps(fixture))
            elif u.path=='/api/web-events':r.fulfill(content_type='application/json',body='{"boot_id":1,"events":[]}')
            elif u.path=='/api/chart-history':
                minutes=int(urllib.parse.parse_qs(u.query)['minutes'][0])
                values=[26.4,78.6,96.2,80]
                history={'ok':True,'minutes':minutes,'temperature':[[v-.4+(i%7)*.07 for i in range(36)] for v in values],
                         'collection_ml_min':[24]*36,'cooling_lph':[72]*36,'collection_sensor':True,'cooling_calibrated':True}
                r.fulfill(content_type='application/json',body=json.dumps(history))
            else:r.fulfill(status=503,body='{}')
        page.route('**/*',route);page.goto('http://busa-ui16.test/')
        page.wait_for_function('connected&&!pollBusy&&document.documentElement.dataset.viewportReady==="1"')
        page.evaluate('update()');page.wait_for_timeout(50);posts.clear()
        chrome=None
        for section,name in names.items():
            page.locator(f'[data-page="{section}"]').click();page.wait_for_timeout(90)
            assert not page.evaluate('scrollY||document.documentElement.scrollWidth>innerWidth+1||document.documentElement.scrollHeight>innerHeight+1')
            assert page.locator('.vpPager').count()==0
            frame=page.locator('main').bounding_box()
            assert page.locator('main').evaluate('(e)=>e.scrollTop')==0
            for sel in ['.appHeader h1','.netIndicators','.appTabs button.on']:
                e=page.locator(sel);r=e.bounding_box()
                assert r and 0<=r['y'] and r['y']+r['height']<=height+1,(width,height,section,sel,r)
                assert e.evaluate('e=>{const r=e.getBoundingClientRect();return !!document.elementFromPoint(r.x+r.width/2,r.y+r.height/2)?.closest(".appHeader,.appTabs")}')
            style=page.locator('.appHeader').evaluate('e=>{const s=getComputedStyle(e);return [s.backgroundColor,s.padding,s.fontSize,e.getBoundingClientRect().height]}')
            if chrome is None:chrome=style
            assert style==chrome,(width,height,section,'header changed',chrome,style)
            active=page.locator('.appTabs button.on')
            assert active.evaluate('e=>getComputedStyle(e).backgroundImage')!='none'
            assert page.locator('.appTabs button').count()==6
            nav=page.locator('.appTabs button').evaluate_all('(es)=>es.map(e=>({label:e.innerText,width:e.clientWidth,scroll:e.scrollWidth,font:getComputedStyle(e).font}))')
            assert all(e['scroll']<=e['width']+1 for e in nav),(width,height,nav)
            if section==0:
                assert page.locator('#relayBadge1').get_attribute('data-state')=='on'
                assert page.locator('#relayBadge1').evaluate('e=>getComputedStyle(e).backgroundImage')!=page.locator('#relayBadge0').evaluate('e=>getComputedStyle(e).backgroundImage')
                assert page.locator('.approvedTemp small:visible,.cubeSetBelow:visible').count()==0
                assert page.locator('#homeCoolingRate').inner_text()=='72,0 л/ч'
                assert page.locator('#homeWaterUsed').inner_text()=='12,4 л'
                assert page.locator('#homeCollectionRate').inner_text()=='24,0'
                assert page.locator('#collectionTotal').inner_text()=='1,2 л'
                metrics=page.locator('.holoMetrics').bounding_box(); relays=page.locator('.relayIndicators').bounding_box()
                assert metrics['y']+metrics['height']<=relays['y']+1
                assert relays['y']+relays['height']<=page.locator('.appTabs').bounding_box()['y']+1
                for sel in ['.approvedTempTitle','.approvedTemp strong[id]','.collectedOnGlass','.holoMetric','.relayBadge']:
                    bounds=page.locator(sel).evaluate_all('(es)=>es.map(e=>{const r=e.getBoundingClientRect();return {id:e.id,text:e.innerText,left:r.left,right:r.right,width:e.clientWidth,scroll:e.scrollWidth,font:getComputedStyle(e).font};})')
                    assert all(e['left']>=-1 and e['right']<=width+1 and e['scroll']<=e['width']+1 for e in bounds),(width,height,sel,bounds)
                assert page.locator('.rigSheen:visible,.rigCoolantGlow:visible,.jacketTrace:visible').count()==0
                assert page.locator('#rigArtStage').evaluate('e=>e.classList.contains("hasCooling")&&e.classList.contains("hasCollection")')
                page.evaluate('window.savedHoloRig=document.querySelector(".approvedRigArt");window.rigMutations=[];window.rigObserver=new MutationObserver(rs=>rigMutations.push(...rs.map(r=>r.type)));rigObserver.observe(savedHoloRig,{attributes:true,childList:true,subtree:true});')
                for _ in range(3):page.evaluate('update()')
                page.wait_for_timeout(30)
                assert page.evaluate('savedHoloRig===document.querySelector(".approvedRigArt")&&rigMutations.length===0')
            if section==2:
                assert page.locator('.collectionPanel').inner_text().count('Текущий отбор')==1
                assert page.locator('#collectionTotal').inner_text()=='1,2 л'
                assert page.locator('#collectionTime').inner_text()=='01:00'
                assert page.locator('section[data-section="2"] canvas').count()==0
                assert page.locator('.hasCollectionFlow .glassBubble').first.evaluate('e=>getComputedStyle(e).animationPlayState')=='running'
                assert page.locator('.hasCoolingFlow .coolingPipe').count()==1
            if section==1:
                for i,color in [(0,'rgb(56, 189, 248)'),(2,'rgb(244, 200, 74)'),(1,'rgb(83, 212, 134)'),(3,'rgb(255, 100, 115)')]:
                    assert page.locator('#tempLive'+str(i)).evaluate('e=>getComputedStyle(e).color')==color
            if section==3:
                assert page.locator('[data-chart]').all_inner_texts()==['Температура','Отбор','Охлаждение']
                assert page.locator('[data-minutes]').all_inner_texts()==['5 мин','10 мин','15 мин','30 мин','1 ч']
                for i in [0,2,1,3]:
                    assert page.locator(f'[data-legend-temp="{i}"]').inner_text()==page.locator('#t'+str(i)).inner_text()
                canvas=page.locator('#chartCanvas').bounding_box()
                assert canvas['height']<=canvas['width']*440/640+2
                assert page.locator('.chartCadence').inner_text()=='Обновление 2 с'
            if section==5:
                assert page.locator('#settingsMenu>.approvedMenuItem:visible').count()==9
            if width==390 and height==780:
                page.screenshot(path=str(shots/(name+'.png')),animations='disabled')
        assert not posts,('navigation must not command the device',posts)
        # Values in the new legend must follow actual polling and clear on loss.
        fixture['temperatures'][0]['c']=27.3;page.evaluate('update()');page.wait_for_timeout(30)
        assert page.locator('[data-legend-temp="0"]').inner_text()=='27,3°C'
        offline=True
        for _ in range(3):page.evaluate('update()')
        page.wait_for_timeout(30)
        assert page.locator('[data-legend-temp="0"]').inner_text()=='Н/Д'
        assert not page.locator('#rigArtStage').evaluate('e=>e.classList.contains("hasCooling")||e.classList.contains("hasCollection")||e.classList.contains("hasTemperature")')
        assert page.locator('.glassBubble').first.evaluate('e=>getComputedStyle(e).animationPlayState')=='paused'
        assert page.locator('.hasCoolingFlow').count()==0
        assert page.locator('#soundStatus1').inner_text()=='Нет связи'
        # All three calculators retain arithmetic and reachable reset/results.
        page.locator('[data-page="7"]').click();page.wait_for_timeout(60)
        for i,fields in [(0,{'v':'1','from':'96','to':'40'}),(1,{'t':'20','reading':'40'}),(2,{'v':'10','strength':'40','out':'80','head':'10','tail':'10'})]:
            page.locator('#calc-tab-'+str(i)).click()
            for key,value in fields.items():page.locator(f'#calc-{i}-{key}').fill(value)
            panel=page.locator('#calc-panel-'+str(i));panel.locator('[type="submit"]').click()
            assert not panel.locator('.calcError').inner_text()
            assert panel.locator('.calcResult').inner_text()
            panel.locator('[type="reset"]').click();page.wait_for_timeout(20)
            assert panel.locator('input').first.input_value()==''
        assert not page.evaluate('scrollY') and page.locator('main').evaluate('(e)=>e.scrollTop')==0
        assert not errors,errors;page.close()
    browser.close()
print('PASS UI16: six screens/eight sizes, stable chrome, 9 settings, no pager/window scroll, live legend/connection loss, correct sensor colours and calculator behaviour')
