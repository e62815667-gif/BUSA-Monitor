package ru.busa.monitor;

public class DeviceAddressTest {
    private static void equal(String a, String b) {
        if (!a.equals(b)) throw new AssertionError(a + " != " + b);
    }
    private static void reject(String value) {
        try { DeviceAddress.normalize(value); }
        catch (IllegalArgumentException expected) { return; }
        throw new AssertionError("Unsafe or malformed address accepted: " + value);
    }
    public static void main(String[] args) {
        equal(DeviceAddress.normalize(null), "http://192.168.4.1");
        equal(DeviceAddress.normalize(" 192.168.4.1/ "), "http://192.168.4.1");
        equal(DeviceAddress.normalize("10.0.2.2:8080"), "http://10.0.2.2:8080");
        equal(DeviceAddress.normalize("https://BUSA.local:443"), "https://busa.local:443");
        equal(DeviceAddress.normalize("172.16.0.1"), "http://172.16.0.1");
        equal(DeviceAddress.normalize("172.31.255.255"), "http://172.31.255.255");
        equal(DeviceAddress.normalize("127.0.0.1:8080"), "http://127.0.0.1:8080");
        for (String bad : new String[]{"https://example.com", "8.8.8.8", "192.168.256.1", "012.0.0.1",
                "172.15.1.1", "172.32.1.1", "file:///etc/passwd", "javascript:alert(1)",
                "http://192.168.4.1@evil.com", "http://user:pass@192.168.4.1", "192.168.4.1:0",
                "192.168.4.1:65536", "http://192.168.4.1/api/status", "192.168.4.1/?x=1", "192.168.4.1/#foo"}) reject(bad);
        if (!DeviceAddress.sameOrigin("http://192.168.4.1", "http://192.168.4.1:80/api/status")) throw new AssertionError();
        for (String bad : new String[]{"http://192.168.4.10", "http://192.168.4.1:81", "https://192.168.4.1", "http://x@192.168.4.1", "data:text/html,x"}) {
            if (DeviceAddress.sameOrigin("http://192.168.4.1", bad)) throw new AssertionError(bad);
        }
        System.out.println("PASS device address parsing, local-network boundary and exact origins");
    }
}
