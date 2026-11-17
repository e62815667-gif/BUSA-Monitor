package ru.busa.monitor;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import org.json.JSONArray;
import org.json.JSONObject;

public final class CloudClientDataTest {
    static final long NOW = 1800000000000L;
    static JSONObject fixture;
    static JSONObject packet() throws Exception {
        return new JSONObject().put("ok",true).put("latest",new JSONObject()
                .put("at",NOW).put("telemetry",new JSONObject(fixture.toString())));
    }
    static void require(boolean value,String detail) {
        if (!value) throw new AssertionError(detail);
    }
    static void reject(JSONObject packet,String detail) throws Exception {
        boolean rejected=false;
        try { CloudClient.telemetry(packet,NOW); } catch (Exception expected) { rejected=true; }
        require(rejected,detail);
    }
    public static void main(String[] args) throws Exception {
        fixture=new JSONObject(new String(Files.readAllBytes(Paths.get(args[0])),StandardCharsets.UTF_8));
        fixture.put("device","BUSA-1M-001");
        require(CloudClient.telemetry(packet(),NOW).getJSONObject("network_indicators").getBoolean("cloud_fresh"),"Valid telemetry rejected");
        JSONObject missing=packet();
        missing.getJSONObject("latest").getJSONObject("telemetry").put("temperatures",new JSONArray("[null,null,null,null]"));
        reject(missing,"Null temperature elements must not produce connected/green UI");
        for (String key:new String[]{"station","google","firmware","ota","modules","uptime_ms"}) {
            JSONObject p=packet();p.getJSONObject("latest").getJSONObject("telemetry").remove(key);
            reject(p,"Missing UI16 structure accepted: "+key);
        }
        JSONObject bad=packet();bad.getJSONObject("latest").getJSONObject("telemetry").put("firmware",123);
        reject(bad,"Non-string firmware accepted");
        bad=packet();bad.getJSONObject("latest").getJSONObject("telemetry").getJSONArray("temperatures").getJSONObject(0).put("c","26.4");
        reject(bad,"Non-numeric sensor value accepted");
        JSONObject absent=packet();
        absent.getJSONObject("latest").getJSONObject("telemetry").getJSONArray("temperatures").getJSONObject(0).put("c",JSONObject.NULL);
        require(CloudClient.telemetry(absent,NOW).getJSONArray("temperatures").getJSONObject(0).isNull("c"),"Valid absent sensor was fabricated or rejected");
        JSONObject stale=packet();stale.getJSONObject("latest").put("at",NOW-120001);reject(stale,"Stale telemetry accepted");
        JSONObject future=packet();future.getJSONObject("latest").put("at",NOW+60001);reject(future,"Future telemetry accepted");
        reject(new JSONObject().put("ok",true).put("latest",JSONObject.NULL),"Empty sheet accepted");
        System.out.println("PASS actual cloud data contract: malformed/incomplete/stale/future rejected; real absent sensor retained");
    }
}
