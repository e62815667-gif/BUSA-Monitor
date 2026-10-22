"""Optional Chromium UI test with mocked HTTP, not a Safari/hardware test.

Run from the checkout with Playwright installed:
  BUSA_CHROMIUM=/usr/bin/chromium python3 tests/browser_wifi_smoke.py
"""
import os
from pathlib import Path
import re


def main():
    from playwright.sync_api import sync_playwright

    root = Path(__file__).resolve().parents[1]
    source = (root / 'firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h').read_text()
    html = re.search(r'R"HTMLBUSA\((.*?)\)HTMLBUSA"', source, re.S).group(1)
    with sync_playwright() as playwright:
        options = {'headless': True, 'args': ['--no-sandbox', '--disable-dev-shm-usage']}
        if os.environ.get('BUSA_CHROMIUM'):
            options['executable_path'] = os.environ['BUSA_CHROMIUM']
        browser = playwright.chromium.launch(**options)
        try:
            for width, height in [(390, 844), (393, 852)]:
                page = browser.new_page(viewport={'width': width, 'height': height},
                                        is_mobile=True, has_touch=True)
                errors = []
                page.on('pageerror', lambda error: errors.append(str(error)))
                saved = []

                def respond(route):
                    path = route.request.url.split('http://busa.test', 1)[-1]
                    if path == '/':
                        route.fulfill(status=200, content_type='text/html', body=html)
                    elif path.startswith('/api/wifi/scan'):
                        route.fulfill(json={'scanning': False,
                                            'error': 'scan_start_failed', 'networks': []})
                    elif path == '/api/wifi':
                        saved.append(route.request.post_data)
                        route.fulfill(status=200, body='Wi-Fi сохранён')
                    elif path == '/api/status':
                        route.fulfill(status=503, body='Mock: no physical ESP32')
                    else:
                        route.fulfill(json={'ok': True})

                page.route('**/*', respond)
                page.goto('http://busa.test/', wait_until='domcontentloaded', timeout=15000)
                page.evaluate('page(5,false); settingsTab(0,false)')
                page.locator('#scan').click(timeout=5000)
                page.wait_for_function("document.getElementById('response').textContent.includes('scan_start_failed')")
                assert not page.locator('#scan').is_disabled()
                for ssid in ['iPhone 12', 'iPhone 16']:
                    page.locator('#manualSsid').fill(ssid)
                    page.locator('#pass').fill('test-only-password')
                    with page.expect_response('http://busa.test/api/wifi'):
                        page.locator('#save').click()
                    page.wait_for_function("!document.getElementById('save').disabled")
                    assert page.locator('#pass').input_value() == ''
                assert len(saved) == 2
                assert 'ssid=iPhone+12' in saved[0] and 'ssid=iPhone+16' in saved[1]
                assert not errors, errors
                page.close()
                print(f'PASS Chromium {width}x{height}: scan failure, released controls, manual SSID requests, cleared password (mocked HTTP)')
        finally:
            browser.close()


if __name__ == '__main__':
    main()
