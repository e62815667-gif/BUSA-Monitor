package ru.busa.monitor;
import java.net.URI;
public final class CloudClientPolicyTest {
    static final String TOKEN = new String(new char[64]).replace('\0','a');
    static void reject(String url, String key, boolean debug) {
        try { new CloudClient(url,key,debug); throw new AssertionError("Accepted unsafe cloud configuration"); }
        catch (IllegalArgumentException expected) { }
    }
    public static void main(String[] args) throws Exception {
        new CloudClient("https://script.google.com/macros/s/abcdefghijklmnopqrst/exec",TOKEN,false);
        new CloudClient("http://10.0.2.2:18080/exec",TOKEN,true);
        for(String url:new String[]{"http://10.0.2.2:18080/exec","http://script.google.com/macros/s/abcdefghijklmnopqrst/exec",
                "https://script.google.com.evil.test/macros/s/abcdefghijklmnopqrst/exec",
                "https://script.google.com/macros/s/abcdefghijklmnopqrst/exec?token=wrong",
                "https://user@script.google.com/macros/s/abcdefghijklmnopqrst/exec"}) reject(url,TOKEN,false);
        reject("https://script.google.com/macros/s/abcdefghijklmnopqrst/exec","short",false);
        assert CloudClient.allowedRedirect(new URI("https://script.googleusercontent.com/macros/echo?key=example"));
        for(String url:new String[]{"http://script.googleusercontent.com/","https://accounts.google.com/",
                "https://user@script.google.com/","https://script.google.com:444/","https://script.google.com.evil.test/","https://127.0.0.1/"})
            assert !CloudClient.allowedRedirect(new URI(url));
        System.out.println("PASS cloud endpoint, release-only HTTPS and redirect/credential boundaries");
    }
}
