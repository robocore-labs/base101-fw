"""Headless live UI integration against a PTY. Never opens physical serial."""
import json
import os
from pathlib import Path
import pty
import subprocess
import tempfile
import threading
import time
from playwright.sync_api import sync_playwright
from calibration_emulator import TelemetryLink,server
master,slave=pty.openpty();process=subprocess.Popen(['/tmp/base101-calibration-protocol-test','--emulate'],stdin=master,stdout=master);os.close(master)
storage=tempfile.TemporaryDirectory(prefix='base101-ui-trials-');link=TelemetryLink(os.ttyname(slave),storage.name,Path(storage.name)/"calibration.h")
http=server.http.server.ThreadingHTTPServer(('127.0.0.1',0),server.Handler);http.link=link
thread=threading.Thread(target=http.serve_forever,daemon=True);thread.start()
def wait(condition,timeout=5):
 end=time.monotonic()+timeout
 while not condition() and time.monotonic()<end:time.sleep(.02)
 assert condition(),link.snapshot()
def enable(page):
 wait(lambda: not link.snapshot()['status']['motion_enabled'])
 page.wait_for_function("!document.querySelector('#enable').disabled && document.querySelector('#enable').textContent==='Enable controls'")
 page.locator('#enable').click();page.wait_for_function("!document.querySelector('.dpad .forward').disabled")
def hold(page,selector):
 el=page.locator(selector);el.scroll_into_view_if_needed();box=el.bounding_box();page.mouse.move(box['x']+box['width']/2,box['y']+box['height']/2);page.mouse.down()
try:
 wait(lambda:link.snapshot()['connected'])
 with sync_playwright() as p:
  browser=p.chromium.launch(args=['--no-sandbox']);page=browser.new_page(viewport={'width':1440,'height':1000});errors=[];page.on('pageerror',lambda e:errors.append(str(e)))
  page.goto(f'http://127.0.0.1:{http.server_port}/');enable(page);hold(page,'.dpad .forward');wait(lambda:link.snapshot()['status']['motion_enabled']);page.mouse.up();wait(lambda:not link.snapshot()['status']['motion_enabled']);page.keyboard.press('Escape')
  page.wait_for_function("!document.querySelector('#apply').disabled")
  page.locator('#param-ax').evaluate("el=>{el.value=.9;el.dispatchEvent(new Event('input',{bubbles:true}))}");page.locator('#apply').click();wait(lambda:link.snapshot()['status']['parameters']['ax']==.9)
  page.locator('#confirm').check();enable(page);page.wait_for_function("!document.querySelector('#single-trial').disabled");page.locator('#single-trial').click();wait(lambda:link.snapshot()['trial'] is not None);page.wait_for_timeout(350);assert link.snapshot()['trial'];page.locator('#trial').click();wait(lambda:link.snapshot()['trial'] is None);wait(lambda:not link.snapshot()['status']['motion_enabled']);assert not link.results[-1]['complete']
  enable(page);page.wait_for_function("!document.querySelector('#single-trial').disabled");page.locator('#single-trial').click();wait(lambda:link.snapshot()['trial'] is not None);wait(lambda:link.snapshot()['trial'] is None,6);page.mouse.up();assert link.results[-1]['complete'];assert link.results[-1]['analysis']['valid'],link.results[-1]['analysis']
  page.wait_for_function("!document.querySelector('[data-review=good]').disabled");page.locator('[data-review=tune]').click();wait(lambda:link.results[-1]['observation']=='tune');page.locator('[data-review=good]').click();wait(lambda:link.results[-1]['observation']=='good')
  page.wait_for_function("!document.querySelector('#results-export').disabled")
  with page.expect_download() as downloaded:page.locator('#results-export').click()
  data=json.loads(Path(downloaded.value.path()).read_text());assert len(data['results'])==2 and data['results'][-1]['complete']
  # One click runs the whole selected step; shorten only the test fixtures.
  import copy
  original_recipes=server.RECIPES;originals={rid:server.BY_ID[rid] for rid in ('forward','backward')}
  short=[]
  for rid,r in originals.items():
   r=copy.deepcopy(r);r['duration_s']=.35;r['stages'][0]['duration_s']=.35;short.append(r)
  server.RECIPES=short+[r for r in original_recipes if r['step']!=0];server.BY_ID.update({r['id']:r for r in short})
  try:
   enable(page);page.locator('#trial').click();wait(lambda:link.snapshot()['sequence'] is not None)
   wait(lambda:link.snapshot()['sequence'] is None,6)
   assert link.snapshot()['last_sequence']['complete'],link.snapshot()['last_sequence']
   assert len(link.results)==4 and all(r['complete'] for r in link.results[-2:])
   page.wait_for_function("!document.querySelector('[data-review=good]').disabled")
   page.locator('[data-review=good]').click();wait(lambda:all(r['observation']=='good' for r in link.results[-2:]))
  finally:server.RECIPES=original_recipes;server.BY_ID.update(originals)
  page.wait_for_function("!document.querySelector('#save-settings').disabled")
  page.locator('#save-settings').click();wait(lambda:(Path(storage.name)/'calibration.h').exists())
  page.locator('[data-step="1"] summary').click();page.wait_for_function("document.querySelector('#step-title').textContent.includes('Floor spins')");assert '1 / 12' in page.locator('#trial-count').inner_text()
  page.locator('#confirm').check();enable(page);assert page.locator('#single-trial').is_disabled()
  page.locator('#surface').fill('wood floor');page.wait_for_function("!document.querySelector('#single-trial').disabled")
  page.locator('#single-trial').click();wait(lambda:link.snapshot()['trial'] is not None)
  wait(lambda:link.snapshot()['trial'] is None,22)
  assert link.results[-1]['complete'],link.results[-1]['reason']
  assert link.results[-1]['analysis']['valid'],link.results[-1]['analysis']
  assert abs(link.results[-1]['analysis']['k_icr']-1.5)<.001
  assert page.locator('#step-suggestions p').count()==4
  page.locator('[data-step="3"] summary').click();page.wait_for_function("document.querySelector('#step-title').textContent.includes('Odometry')")
  assert '1 / 14' in page.locator('#trial-count').inner_text()
  for _ in range(4):page.locator('#trial-next').click()
  assert '360° odometry verification' in page.locator('#prompt').inner_text()
  assert page.locator('#reference-label').is_hidden()
  page.screenshot(path='/tmp/base101-calibration-desktop.png',full_page=True)
  for width in [1200,1050,820,390]:
   page.set_viewport_size({'width':width,'height':844});assert page.evaluate('document.documentElement.scrollWidth<=innerWidth'),width
  page.screenshot(path='/tmp/base101-calibration-mobile.png',full_page=True)
  page.locator('[data-step="0"] summary').click();page.locator('#confirm').check();enable(page);page.locator('#single-trial').click();wait(lambda:link.snapshot()['trial'] is not None);page.evaluate("window.dispatchEvent(new Event('blur'))");wait(lambda:link.snapshot()['trial'] is None);page.mouse.up();wait(lambda:not link.snapshot()['status']['motion_enabled'])
  process.terminate();process.wait(timeout=2);page.wait_for_function("document.querySelector('#connection').textContent==='Disconnected'");assert page.locator('#enable').is_disabled();assert not errors,errors;browser.close()
 print('Live desktop/mobile UI: manual release, sliders, bounded trials, early abort, review/export, focus loss and disconnect pass.')
finally:
 http.shutdown();http.server_close();thread.join();link.close();os.close(slave)
 if process.poll() is None:process.terminate()
 process.wait(timeout=2);storage.cleanup()
