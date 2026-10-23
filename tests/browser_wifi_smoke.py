"""Playwright tests against the actual HTML embedded in the ESP32 firmware.

HTTP and Wi-Fi scan results are simulated: these tests do not prove that the
real ESP32 radio, access point, or iPhone Safari works.
Run: python3 tests/browser_wifi_smoke.py
"""
from pathlib import Path
from urllib.parse import parse_qs
import re


def main():
    from playwright.sync_api import sync_playwright

    root = Path(__file__).resolve().parents[1]
    firmware = (root / "firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_Main.h").read_text()
    match = re.search(r'R"HTMLBUSA\((.*?)\)HTMLBUSA"', firmware, re.S)
    assert match, "Embedded firmware HTML not found"
    html = match.group(1)
    networks = [
        {"ssid": "iPhone 16", "rssi": -41},
        {"ssid": "iPhone 12", "rssi": -48},
    ] + [{"ssid": f"Test-AP-{i:02}", "rssi": -55 - i} for i in range(23)]
    assert len(networks) == 25

    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(headless=True, args=[
            "--no-sandbox", "--disable-dev-shm-usage"
        ])
        try:
            for width, height in ((390, 844), (393, 852)):
                page = browser.new_page(
                    viewport={"width": width, "height": height},
                    is_mobile=True, has_touch=True,
                )
                errors = []
                page.on("pageerror", lambda error: errors.append(str(error)))
                submitted = []
                mode = {"value": "success", "polls": 0}

                def respond(route):
                    path = route.request.url.split("http://busa.test", 1)[-1]
                    if path == "/":
                        route.fulfill(status=200, content_type="text/html", body=html)
                    elif path.startswith("/api/wifi/scan"):
                        if "start=1" in path:
                            mode["polls"] = 0
                            route.fulfill(json={"scanning": True, "networks": []})
                        else:
                            mode["polls"] += 1
                            if mode["value"] == "success":
                                route.fulfill(json={"scanning": False, "networks": networks})
                            elif mode["value"] == "empty":
                                route.fulfill(json={"scanning": False, "networks": []})
                            else:
                                route.fulfill(json={"scanning": False, "error": "scan_driver_start_failed", "networks": []})
                    elif path == "/api/wifi":
                        submitted.append(parse_qs(route.request.post_data or ""))
                        route.fulfill(status=200, body="Wi-Fi сохранён")
                    elif path == "/api/status":
                        route.fulfill(status=503, body="Mock: physical ESP32 unavailable")
                    else:
                        route.fulfill(json={"ok": True})

                page.route("**/*", respond)
                page.goto("http://busa.test/", wait_until="domcontentloaded", timeout=15000)
                page.evaluate("page(5,false);settingsTab(0,false)")
                page.locator("#scan").click(timeout=10000)
                page.wait_for_function(
                    "document.querySelector('#net').options.length === 26",
                    timeout=15000,
                )
                assert mode["polls"] > 0, "Expected scan status poll"
                assert page.locator("#net option").count() == 26
                assert page.locator("#response").inner_text().startswith("Найдено сетей: 25")
                assert not page.locator("#scan").is_disabled()

                page.locator("#net").select_option("iPhone 16")
                page.locator("#pass").fill("test-only-password")
                with page.expect_response("http://busa.test/api/wifi"):
                    page.locator("#save").click()
                page.wait_for_function("!document.querySelector('#save').disabled")
                assert len(submitted) == 1, "Wi-Fi credentials were not submitted"
                assert submitted[0].get("ssid") == ["iPhone 16"]
                assert submitted[0].get("pass") == ["test-only-password"]
                assert page.locator("#pass").input_value() == ""

                mode["value"] = "empty"
                page.locator("#scan").click()
                page.wait_for_function(
                    "document.querySelector('#response').textContent.includes('Сети не найдены')",
                    timeout=15000,
                )
                assert page.locator("#net option").count() == 1

                mode["value"] = "error"
                page.locator("#scan").click()
                page.wait_for_function(
                    "document.querySelector('#response').textContent.includes('scan_driver_start_failed')",
                    timeout=15000,
                )
                assert not page.locator("#scan").is_disabled()
                assert not errors, errors
                page.close()
                print(f"PASS {width}x{height}: 25 discovered APs shown, SSID selection and POST, empty/error states")
        finally:
            browser.close()


if __name__ == "__main__":
    main()
