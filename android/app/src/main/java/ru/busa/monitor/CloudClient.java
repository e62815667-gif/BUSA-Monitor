package ru.busa.monitor;

import org.json.JSONArray;
import org.json.JSONObject;
import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URI;
import java.net.URL;
import java.net.URLEncoder;
import java.nio.charset.StandardCharsets;

/** Google reader: separate view key, verified HTTPS, no device write secret. */
final class CloudClient {
    static final long MAX_AGE_MS = 120000;
    final String endpoint, token;
    private final boolean debugFixture;

    CloudClient(String endpoint, String token, boolean debug) {
        this.endpoint = endpoint.trim(); this.token = token.trim();
        URI uri;
        try { uri = new URI(this.endpoint); } catch (Exception e) { throw new IllegalArgumentException("Нужна ссылка Google Apps Script /exec"); }
        debugFixture = debug && "http".equals(uri.getScheme()) && uri.getHost() != null
                && DeviceAddress.isLocalHost(uri.getHost()) && "/exec".equals(uri.getPath())
                && uri.getRawQuery() == null && uri.getRawUserInfo() == null;
        if (!debugFixture && !this.endpoint.matches("https://script\\.google\\.com/macros/s/[A-Za-z0-9_-]{20,}/exec"))
            throw new IllegalArgumentException("Нужна ссылка Google Apps Script, заканчивающаяся на /exec");
        if (!this.token.matches("[A-Fa-f0-9]{64}"))
            throw new IllegalArgumentException("Нужен ключ просмотра BUSA_VIEW_TOKEN из 64 символов");
    }

    JSONObject latest() throws Exception {
        String target = endpoint + "?mode=latest&callback=busaViewReceive&readToken=" + URLEncoder.encode(token, "UTF-8");
        for (int i = 0; i < 4; i++) {
            HttpURLConnection request = (HttpURLConnection) new URL(target).openConnection();
            request.setInstanceFollowRedirects(false);
            request.setConnectTimeout(10000); request.setReadTimeout(15000);
            request.setUseCaches(false);
            try {
                int code = request.getResponseCode();
                if (code == 301 || code == 302 || code == 303 || code == 307 || code == 308) {
                    String location = request.getHeaderField("Location");
                    if (location == null) throw new Exception("Google не прислал адрес ответа");
                    URI next = new URI(target).resolve(location);
                    if (!allowedRedirect(next))
                        throw new Exception("Google требует вход. Проверь доступ к развёртыванию Apps Script");
                    target = next.toString(); continue;
                }
                if (code != 200) throw new Exception("Google: HTTP " + code);
                String body;
                try (InputStream input = request.getInputStream(); ByteArrayOutputStream bytes = new ByteArrayOutputStream()) {
                    byte[] buffer = new byte[8192]; int n;
                    while ((n = input.read(buffer)) != -1) {
                        if (bytes.size() + n > 256000) throw new Exception("Ответ Google слишком большой");
                        bytes.write(buffer, 0, n);
                    }
                    body = new String(bytes.toByteArray(), StandardCharsets.UTF_8).trim();
                }
                if (body.startsWith("busaViewReceive(") && body.endsWith(");"))
                    body = body.substring("busaViewReceive(".length(), body.length() - 2);
                JSONObject packet;
                try { packet = new JSONObject(body); } catch (Exception e) { throw new Exception("Google вернул страницу вместо телеметрии. Проверь развёртывание Apps Script"); }
                if (!packet.optBoolean("ok")) throw new Exception("Google отклонил доступ. Проверь ключ просмотра");
                return packet;
            } finally { request.disconnect(); }
        }
        throw new Exception("Слишком много перенаправлений Google");
    }

    static boolean allowedRedirect(URI next) {
        return "https".equals(next.getScheme()) && next.getRawUserInfo() == null
                && (next.getPort() == -1 || next.getPort() == 443)
                && ("script.google.com".equals(next.getHost()) || "script.googleusercontent.com".equals(next.getHost()));
    }

    static JSONObject telemetry(JSONObject packet, long now) throws Exception {
        JSONObject latest = packet == null ? null : packet.optJSONObject("latest");
        if (latest == null) throw new Exception("В Google ещё нет показаний БУСА. ESP32 должна начать отправку");
        long at = latest.optLong("at", 0);
        if (at <= 0 || at > now + 60000 || now - at > MAX_AGE_MS)
            throw new Exception("Данные устарели. ESP32 не обновляет Google");
        JSONObject raw = latest.optJSONObject("telemetry");
        if (raw == null || !"BUSA-1M-001".equals(raw.optString("device"))
                || raw.optJSONArray("temperatures") == null || raw.optJSONArray("temperatures").length() != 4
                || raw.optJSONObject("water") == null)
            throw new Exception("Неполная телеметрия БУСА в Google");
        JSONArray temperatures=raw.getJSONArray("temperatures");
        for (int i=0;i<4;i++) {
            JSONObject sensor=temperatures.optJSONObject(i);
            if (sensor==null || !sensor.has("c") || !sensor.isNull("c") && !finiteNumber(sensor.opt("c")))
                throw new Exception("Некорректная температура в ответе Google");
        }
        JSONObject station=raw.optJSONObject("station"), google=raw.optJSONObject("google"), ota=raw.optJSONObject("ota");
        if (station==null || !(station.opt("connected") instanceof Boolean)
                || google==null || !(google.opt("configured") instanceof Boolean)
                || ota==null || !(ota.opt("message") instanceof String)
                || raw.optJSONObject("modules")==null || !(raw.opt("firmware") instanceof String)
                || raw.optString("firmware").isEmpty() || !finiteNumber(raw.opt("uptime_ms"))
                || raw.optDouble("uptime_ms")<0)
            throw new Exception("Неполная телеметрия БУСА в Google");
        JSONObject result = new JSONObject(raw.toString());
        JSONObject indicators = result.optJSONObject("network_indicators");
        if (indicators == null) { indicators = new JSONObject(); result.put("network_indicators", indicators); }
        indicators.put("cloud_fresh", true);
        return result;
    }

    private static boolean finiteNumber(Object value) {
        if (!(value instanceof Number)) return false;
        double n=((Number)value).doubleValue();
        return !Double.isNaN(n) && !Double.isInfinite(n);
    }

    static JSONObject chart(JSONObject packet, int minutes, long now) throws Exception {
        JSONObject current = telemetry(packet, now); // Never draw an old archive as current data.
        JSONArray rows = packet.optJSONArray("history");
        int bins = Math.min(minutes * 30, 160);
        JSONArray[] temperatures = {new JSONArray(), new JSONArray(), new JSONArray(), new JSONArray()};
        JSONArray water = new JSONArray(), collection = new JSONArray();
        long[] latestAt = new long[bins];
        for (int b = 0; b < bins; b++) {
            for (JSONArray ts : temperatures) ts.put(JSONObject.NULL);
            water.put(JSONObject.NULL); collection.put(JSONObject.NULL);
        }
        if (rows != null) for (int i = 0; i < rows.length(); i++) {
            JSONObject row = rows.optJSONObject(i);
            if (row == null || row.optLong("at") < now - minutes * 60000L || row.optLong("at") > now + 60000) continue;
            int bin = (int)Math.min(bins - 1, (row.optLong("at") - (now - minutes * 60000L)) * bins / (minutes * 60000L));
            if (row.optLong("at") < latestAt[bin]) continue;
            latestAt[bin] = row.optLong("at");
            JSONArray ts = row.optJSONArray("temperatures");
            for (int t = 0; t < 4; t++) temperatures[t].put(bin, ts == null ? JSONObject.NULL : ts.opt(t));
            water.put(bin, row.has("cooling_lph") ? row.opt("cooling_lph") : JSONObject.NULL);
            collection.put(bin, row.has("collection_ml_min") ? row.opt("collection_ml_min") : JSONObject.NULL);
        }
        return new JSONObject().put("ok", true).put("minutes", minutes)
                .put("temperature", new JSONArray().put(temperatures[0]).put(temperatures[1]).put(temperatures[2]).put(temperatures[3]))
                .put("cooling_lph", water).put("collection_ml_min", collection)
                .put("cooling_calibrated", current.getJSONObject("water").optBoolean("calibrated"))
                .put("collection_sensor", current.optJSONObject("collection") != null
                        && current.getJSONObject("collection").optBoolean("sensor_present"));
    }
}
