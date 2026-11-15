package ru.busa.monitor;

import java.net.URI;
import java.util.Locale;

/** Plain Java policy shared by navigation, connection settings and host tests. */
public final class DeviceAddress {
    public static final String DEFAULT = "http://192.168.4.1";
    private DeviceAddress() {}

    public static String normalize(String input) {
        if (input == null || input.trim().isEmpty()) return DEFAULT;
        String text = input.trim();
        if (!text.contains("://")) text = "http://" + text;
        try {
            URI uri = new URI(text);
            String host = uri.getHost();
            String scheme = uri.getScheme().toLowerCase(Locale.ROOT);
            String path = uri.getRawPath();
            if (!(scheme.equals("http") || scheme.equals("https")) || host == null
                    || uri.getRawUserInfo() != null || uri.getRawQuery() != null
                    || uri.getRawFragment() != null || !(path == null || path.isEmpty() || path.equals("/"))
                    || uri.getPort() == 0 || uri.getPort() > 65535 || !isLocalHost(host)) {
                throw new IllegalArgumentException();
            }
            return scheme + "://" + host.toLowerCase(Locale.ROOT)
                    + (uri.getPort() == -1 ? "" : ":" + uri.getPort());
        } catch (Exception error) {
            throw new IllegalArgumentException("Укажи локальный IP БУСА, например 192.168.4.1");
        }
    }

    static boolean isLocalHost(String host) {
        host = host.toLowerCase(Locale.ROOT);
        if (host.equals("localhost") || host.endsWith(".local") && host.length() > 6) return true;
        String[] fields = host.split("\\.", -1);
        if (fields.length != 4) return false;
        int[] n = new int[4];
        try {
            for (int i = 0; i < 4; i++) {
                if (!fields[i].matches("0|[1-9][0-9]{0,2}")) return false;
                n[i] = Integer.parseInt(fields[i]);
                if (n[i] > 255) return false;
            }
        } catch (NumberFormatException error) { return false; }
        return n[0] == 10 || n[0] == 127 || n[0] == 192 && n[1] == 168
                || n[0] == 172 && n[1] >= 16 && n[1] <= 31 || n[0] == 169 && n[1] == 254;
    }

    public static boolean sameOrigin(String address, String destination) {
        try {
            URI a = new URI(address), b = new URI(destination);
            return a.getScheme().equalsIgnoreCase(b.getScheme()) && a.getHost().equalsIgnoreCase(b.getHost())
                    && effectivePort(a) == effectivePort(b) && b.getRawUserInfo() == null;
        } catch (Exception error) { return false; }
    }

    private static int effectivePort(URI uri) {
        return uri.getPort() != -1 ? uri.getPort() : "https".equalsIgnoreCase(uri.getScheme()) ? 443 : 80;
    }
}
