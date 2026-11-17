package ru.busa.monitor;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.ActivityNotFoundException;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.ApplicationInfo;
import android.graphics.Color;
import android.net.ConnectivityManager;
import android.net.Network;
import android.net.NetworkCapabilities;
import android.net.NetworkRequest;
import android.net.Uri;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.provider.Settings;
import android.view.Gravity;
import android.view.View;
import android.view.WindowInsets;
import android.webkit.CookieManager;
import android.webkit.PermissionRequest;
import android.webkit.ValueCallback;
import android.webkit.WebChromeClient;
import android.webkit.WebResourceError;
import android.webkit.WebResourceRequest;
import android.webkit.WebResourceResponse;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.WebViewClient;
import android.widget.EditText;
import android.widget.FrameLayout;
import android.widget.LinearLayout;
import android.widget.ProgressBar;
import android.widget.ScrollView;
import android.widget.TextView;
import android.widget.Toast;
import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import org.json.JSONObject;
import java.util.Collections;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/** Native container; the ESP32 remains the source of UI and device behaviour. */
public final class MainActivity extends Activity {
    private static final int FILE_REQUEST = 14;
    private static final int BACKGROUND = Color.rgb(6, 14, 21);
    private static final String CLOUD_ORIGIN = "https://busa.invalid";
    private final Handler main = new Handler(Looper.getMainLooper());
    private final ExecutorService cloudWorker = Executors.newSingleThreadExecutor();
    private WebView web;
    private ProgressBar progress;
    private ConnectivityManager connectivity;
    private ConnectivityManager.NetworkCallback wifiCallback;
    private volatile String address;
    private volatile boolean offlineShell;
    private volatile boolean destroyed;
    private boolean resumed;
    private boolean connecting;
    private AlertDialog connectionError;
    private int connectionAttempt;
    private ValueCallback<Uri[]> fileResult;
    private SharedPreferences preferences;
    private TextView cloudStatus;
    private volatile boolean remoteMode;
    private volatile JSONObject cloudPacket;
    private volatile CloudClient cloud;
    private volatile boolean cloudBusy;
    private volatile int cloudGeneration;
    private final Runnable cloudTick = new Runnable() {
        @Override public void run() { if (!destroyed && remoteMode) { refreshCloud(); main.postDelayed(this, 30000); } }
    };

    @Override public void onCreate(Bundle state) {
        super.onCreate(state);
        preferences = getSharedPreferences("connection", MODE_PRIVATE);
        remoteMode = !"local".equals(preferences.getString("mode", "cloud"));
        try { address = DeviceAddress.normalize(preferences.getString("address", DeviceAddress.DEFAULT)); }
        catch (IllegalArgumentException error) { address = DeviceAddress.DEFAULT; }
        // Select local Wi-Fi before WebView creates its network stack.
        bindWifi();
        FrameLayout root = new FrameLayout(this);
        root.setBackgroundColor(BACKGROUND);
        web = new WebView(this);
        web.setBackgroundColor(BACKGROUND);
        LinearLayout content = new LinearLayout(this); content.setOrientation(LinearLayout.VERTICAL);
        cloudStatus = new TextView(this); cloudStatus.setTextColor(Color.rgb(110, 229, 222));
        cloudStatus.setTextSize(12); cloudStatus.setPadding(dp(10), dp(6), dp(10), dp(6));
        cloudStatus.setOnClickListener(view -> showConnectionMenu());
        content.addView(cloudStatus, new LinearLayout.LayoutParams(-1, -2));
        content.addView(web, new LinearLayout.LayoutParams(-1, 0, 1));
        root.addView(content, new FrameLayout.LayoutParams(-1, -1));
        progress = new ProgressBar(this);
        FrameLayout.LayoutParams spinner = new FrameLayout.LayoutParams(dp(40), dp(40), Gravity.CENTER);
        root.addView(progress, spinner);
        setContentView(root);
        if (Build.VERSION.SDK_INT >= 30) {
            getWindow().setDecorFitsSystemWindows(false);
            root.setOnApplyWindowInsetsListener((view, insets) -> {
                android.graphics.Insets pad = insets.getInsets(WindowInsets.Type.systemBars()
                        | WindowInsets.Type.displayCutout() | WindowInsets.Type.ime());
                view.setPadding(pad.left, pad.top, pad.right, pad.bottom);
                return insets;
            });
            root.requestApplyInsets();
        }
        WebSettings settings = web.getSettings();
        settings.setJavaScriptEnabled(true);
        settings.setDomStorageEnabled(true);
        settings.setAllowFileAccess(false);
        settings.setAllowContentAccess(true); // Only user-selected document URIs, for ESP32 OTA.
        settings.setJavaScriptCanOpenWindowsAutomatically(false);
        settings.setSupportMultipleWindows(false);
        settings.setMixedContentMode(WebSettings.MIXED_CONTENT_NEVER_ALLOW);
        settings.setCacheMode(WebSettings.LOAD_NO_CACHE);
        settings.setTextZoom(100);
        settings.setUseWideViewPort(true);
        settings.setLoadWithOverviewMode(true);
        settings.setSupportZoom(false);
        if (Build.VERSION.SDK_INT >= 29) settings.setForceDark(WebSettings.FORCE_DARK_OFF);
        CookieManager.getInstance().setAcceptThirdPartyCookies(web, false);
        WebView.setWebContentsDebuggingEnabled((getApplicationInfo().flags & ApplicationInfo.FLAG_DEBUGGABLE) != 0);
        web.setWebViewClient(new Client());
        web.setWebChromeClient(new Chrome());
        web.setDownloadListener((url, userAgent, disposition, type, length) -> {
            if (DeviceAddress.sameOrigin(address, url)) openExternal(url);
        });
        connect();
    }

    private int dp(int value) { return Math.round(value * getResources().getDisplayMetrics().density); }
    private String uiOrigin() { return remoteMode ? CLOUD_ORIGIN : address; }
    private boolean isDebug() { return (getApplicationInfo().flags & ApplicationInfo.FLAG_DEBUGGABLE) != 0; }

    private void bindWifi() {
        connectivity = (ConnectivityManager) getSystemService(CONNECTIVITY_SERVICE);
        wifiCallback = new ConnectivityManager.NetworkCallback() {
            @Override public void onAvailable(Network network) {
                main.post(() -> {
                    if (destroyed || remoteMode) return;
                    bindExistingWifi();
                    if (web != null && resumed && offlineShell && !connecting) connect();
                });
            }
            @Override public void onLost(Network network) {
                main.post(() -> {
                    if (!destroyed && network.equals(connectivity.getBoundNetworkForProcess())) {
                        try { connectivity.bindProcessToNetwork(null); }
                        catch (SecurityException error) { android.util.Log.w("BUSA","Cannot release local Wi-Fi route",error); }
                    }
                });
            }
        };
        try {
            // A local AP without internet must still win over mobile data.
            if (!remoteMode) bindExistingWifi();
            connectivity.registerNetworkCallback(new NetworkRequest.Builder()
                    .addTransportType(NetworkCapabilities.TRANSPORT_WIFI)
                    .removeCapability(NetworkCapabilities.NET_CAPABILITY_INTERNET).build(), wifiCallback);
        } catch (SecurityException ignored) { }
    }

    private boolean bindExistingWifi() {
        try {
            for (Network network : connectivity.getAllNetworks()) {
                NetworkCapabilities caps=connectivity.getNetworkCapabilities(network);
                if (caps!=null && caps.hasTransport(NetworkCapabilities.TRANSPORT_WIFI)) {
                    return network.equals(connectivity.getBoundNetworkForProcess()) || connectivity.bindProcessToNetwork(network);
                }
            }
        } catch (SecurityException error) { android.util.Log.w("BUSA","Cannot select local Wi-Fi route",error); }
        return false;
    }

    private void connect() {
        if (destroyed) return;
        cloudStatus.setVisibility(remoteMode ? View.VISIBLE : View.GONE);
        if (remoteMode) { connectCloud(); return; }
        bindExistingWifi();
        main.removeCallbacks(cloudTick); ++cloudGeneration;
        final int attempt = ++connectionAttempt;
        if (connectionError != null) { connectionError.dismiss(); connectionError = null; }
        web.stopLoading();
        connecting = true;
        offlineShell = false;
        progress.setVisibility(View.VISIBLE);
        // Load the same page as the browser. A short API preflight used to
        // silently replace a reachable, slow ESP32 with the bundled interface.
        web.loadUrl(address + "/");
        main.postDelayed(() -> {
            if (!destroyed && attempt == connectionAttempt && connecting) {
                web.stopLoading(); showOffline("Прибор не ответил за 30 секунд.");
            }
        }, 30000);
    }

    private void connectCloud() {
        ++connectionAttempt; ++cloudGeneration; cloudPacket = null; cloudBusy = false;
        if (connectionError != null) { connectionError.dismiss(); connectionError = null; }
        connecting = false; offlineShell = false; progress.setVisibility(View.GONE);
        // Mobile data / the system's internet route must remain available.
        try { connectivity.bindProcessToNetwork(null); } catch (SecurityException ignored) { }
        main.removeCallbacks(cloudTick);
        try { cloud = new CloudClient(preferences.getString("cloud_url", ""), preferences.getString("view_token", ""), isDebug()); }
        catch (IllegalArgumentException error) { cloud = null; }
        try {
            web.stopLoading();
            web.loadDataWithBaseURL(CLOUD_ORIGIN + "/", read(getAssets().open("local/index.html"), 2000000), "text/html", "UTF-8", null);
        } catch (Exception error) { cloudStatus.setText("Не удалось открыть интерфейс БУСА"); return; }
        if (cloud == null) {
            cloudStatus.setText("Интернет · подключить БУСА — нажмите здесь");
            main.postDelayed(() -> { if (!destroyed && remoteMode && cloud == null) editCloud(); }, 700);
        } else { cloudStatus.setText("Интернет · запрашиваю данные Google…"); main.post(cloudTick); }
    }

    private void refreshCloud() {
        final CloudClient client = cloud; final int generation = cloudGeneration;
        if (client == null || cloudBusy || destroyed || !remoteMode) return;
        cloudBusy = true;
        cloudWorker.execute(() -> {
            JSONObject packet = null; String message;
            try {
                packet = client.latest(); CloudClient.telemetry(packet, System.currentTimeMillis());
                long at = packet.getJSONObject("latest").getLong("at");
                message = "Google · данные " + new java.text.SimpleDateFormat("HH:mm:ss", java.util.Locale.getDefault()).format(new java.util.Date(at));
            } catch (Exception error) {
                message = error instanceof java.net.SocketTimeoutException ? "Google не отвечает. Повторю подключение"
                        : error instanceof java.io.IOException ? "Нет соединения с Google. Проверь интернет телефона"
                        : error.getMessage();
                packet = null;
            }
            final JSONObject result = packet; final String text = message;
            main.post(() -> {
                if (destroyed || generation != cloudGeneration || !remoteMode) return;
                cloudBusy = false; cloudPacket = result; cloudStatus.setText(text);
            });
        });
    }

    private static WebResourceResponse response(int code, String type, String body) {
        return new WebResourceResponse(type, "UTF-8", code, code == 200 ? "OK" : "Unavailable",
                Collections.singletonMap("Cache-Control", "no-store"),
                new java.io.ByteArrayInputStream(body.getBytes(StandardCharsets.UTF_8)));
    }

    private WebResourceResponse cloudApi(WebResourceRequest request) {
        String path = request.getUrl().getPath();
        if (!"GET".equals(request.getMethod()))
            return response(409, "text/plain", "Для изменения настроек прибора подключитесь к Wi-Fi БУСА");
        try {
            JSONObject packet = cloudPacket;
            if ("/api/status".equals(path)) return response(200, "application/json", CloudClient.telemetry(packet, System.currentTimeMillis()).toString());
            if ("/api/chart-history".equals(path)) {
                int minutes = Integer.parseInt(request.getUrl().getQueryParameter("minutes"));
                if (minutes != 5 && minutes != 10 && minutes != 15 && minutes != 30 && minutes != 60) minutes = 5;
                return response(200, "application/json", CloudClient.chart(packet, minutes, System.currentTimeMillis()).toString());
            }
        } catch (Exception ignored) { return response(503, "application/json", "{\"error\":\"no_fresh_cloud_telemetry\"}"); }
        return response(409, "application/json", "{\"error\":\"direct_connection_required\"}");
    }

    private void showOffline(String reason) {
        if (destroyed || offlineShell) return;
        connecting = false;
        offlineShell = true;
        progress.setVisibility(View.GONE);
        try {
            String html = read(getAssets().open("local/index.html"), 2000000);
            web.loadDataWithBaseURL(address + "/", html, "text/html", "UTF-8", null);
            connectionError = new AlertDialog.Builder(this).setTitle("Нет связи с БУСА")
                    .setMessage(address + "\n" + reason + "\nТелефон и прибор должны быть в одной сети Wi-Fi. Если БУСА подключена к роутеру, укажи её адрес в этой сети.")
                    .setPositiveButton("Повторить", (dialog, which) -> connect())
                    .setNeutralButton("Адрес БУСА", (dialog, which) -> editAddress())
                    .setNegativeButton("Открыть без связи", null).create();
            connectionError.show();
        } catch (Exception error) {
            progress.setVisibility(View.GONE);
            new AlertDialog.Builder(this).setTitle("БУСА").setMessage("Не удалось открыть интерфейс")
                    .setPositiveButton("Подключиться", (dialog, which) -> connect()).show();
        }
    }

    private static String read(InputStream stream, int limit) throws Exception {
        try (InputStream input = stream; ByteArrayOutputStream output = new ByteArrayOutputStream()) {
            byte[] buffer = new byte[8192]; int length;
            while ((length = input.read(buffer)) != -1) {
                if (output.size() + length > limit) throw new IllegalArgumentException("Response too large");
                output.write(buffer, 0, length);
            }
            return new String(output.toByteArray(), StandardCharsets.UTF_8);
        }
    }

    private final class Client extends WebViewClient {
        @Override public boolean shouldOverrideUrlLoading(WebView view, WebResourceRequest request) {
            return navigate(request.getUrl().toString(), request.hasGesture());
        }
        @Override public boolean shouldOverrideUrlLoading(WebView view, String url) { return navigate(url, true); }
        @Override public WebResourceResponse shouldInterceptRequest(WebView view, WebResourceRequest request) {
            String url = request.getUrl().toString();
            if (url.startsWith("data:image/")) return null; // Embedded apparatus and SVG icons.
            if (!DeviceAddress.sameOrigin(uiOrigin(), url)) {
                return new WebResourceResponse("text/plain", "UTF-8", new java.io.ByteArrayInputStream(new byte[0]));
            }
            if (remoteMode && request.getUrl().getPath().startsWith("/api/")) return cloudApi(request);
            if ((offlineShell || remoteMode) && request.getMethod().equals("GET")) {
                String path = request.getUrl().getPath();
                if ("/calculator.js".equals(path) || "/calculator.css".equals(path)) {
                    try {
                        return new WebResourceResponse(path.endsWith("js") ? "application/javascript" : "text/css",
                                "UTF-8", getAssets().open("local" + path));
                    } catch (Exception ignored) { }
                }
            }
            return null;
        }
        @Override public void onPageCommitVisible(WebView view, String url) {
            connecting = false;
            progress.setVisibility(View.GONE);
            // Android 6+ can have a WebView predating AbortSignal.timeout.
            // Keep requests bounded there too; do not replace device responses.
            view.evaluateJavascript("(function(){if(typeof AbortSignal!=='undefined'&&"
                    + "typeof AbortSignal.timeout!=='function'&&typeof AbortController==='function'){"
                    + "AbortSignal.timeout=function(ms){var c=new AbortController();"
                    + "setTimeout(function(){c.abort();},ms);return c.signal;};}})()", null);
        }
        @Override public void onPageFinished(WebView view, String url) {
            progress.setVisibility(View.GONE);
            onPageCommitVisible(view, url);
            if (remoteMode) {
                try { view.evaluateJavascript(read(getAssets().open("cloud.js"), 30000), null); }
                catch (Exception ignored) { cloudStatus.setText("Не удалось открыть облачный интерфейс"); }
            }
        }
        @Override public void onReceivedError(WebView view, WebResourceRequest request, WebResourceError error) {
            if (!remoteMode && request.isForMainFrame() && !offlineShell && DeviceAddress.sameOrigin(address,request.getUrl().toString()))
                showOffline("Ошибка соединения (" + error.getErrorCode() + ").");
        }
        @Override public void onReceivedHttpError(WebView view, WebResourceRequest request, WebResourceResponse response) {
            if (!remoteMode && request.isForMainFrame() && !offlineShell && DeviceAddress.sameOrigin(address,request.getUrl().toString()))
                showOffline("Ответ прибора: HTTP " + response.getStatusCode() + ".");
        }
    }

    private boolean navigate(String url, boolean userGesture) {
        if (remoteMode && userGesture && "busa://connection".equals(url)) { showConnectionMenu(); return true; }
        if (DeviceAddress.sameOrigin(uiOrigin(), url)) return false;
        if (userGesture && url.startsWith("https://")) openExternal(url);
        return true;
    }

    private void openExternal(String url) {
        try { startActivity(new Intent(Intent.ACTION_VIEW, Uri.parse(url))); }
        catch (ActivityNotFoundException error) { Toast.makeText(this, "Не найден браузер", Toast.LENGTH_SHORT).show(); }
    }

    private final class Chrome extends WebChromeClient {
        @Override public void onPermissionRequest(PermissionRequest request) { request.deny(); }
        @Override public boolean onShowFileChooser(WebView view, ValueCallback<Uri[]> callback, FileChooserParams params) {
            if (fileResult != null) fileResult.onReceiveValue(null);
            fileResult = callback;
            Intent intent = new Intent(Intent.ACTION_OPEN_DOCUMENT).addCategory(Intent.CATEGORY_OPENABLE)
                    .setType("*/*").putExtra(Intent.EXTRA_ALLOW_MULTIPLE, false);
            try { startActivityForResult(intent, FILE_REQUEST); }
            catch (ActivityNotFoundException error) {
                fileResult.onReceiveValue(null); fileResult = null;
                Toast.makeText(MainActivity.this, "Не найдено приложение выбора файлов", Toast.LENGTH_SHORT).show();
            }
            return true;
        }
    }

    @Override protected void onActivityResult(int request, int result, Intent data) {
        super.onActivityResult(request, result, data);
        if (request != FILE_REQUEST || fileResult == null) return;
        Uri[] selected = WebChromeClient.FileChooserParams.parseResult(result, data);
        Uri uri = selected != null && selected.length == 1 ? selected[0] : null;
        fileResult.onReceiveValue(uri != null && "content".equals(uri.getScheme()) ? new Uri[]{uri} : null);
        fileResult = null;
    }

    @Override public void onBackPressed() {
        if (fileResult != null) { fileResult.onReceiveValue(null); fileResult = null; return; }
        web.evaluateJavascript("(function(){if(typeof active==='undefined')return false;"
                + "if(active===5&&typeof settingsActive!=='undefined'&&settingsActive!==-1){settingsTab(-1,false);return true;}"
                + "if(active!==0){page(0,false);return true;}return false;})()", handled -> {
            if (!"true".equals(handled) && !destroyed) showConnectionMenu();
        });
    }

    private void showConnectionMenu() {
        new AlertDialog.Builder(this).setTitle("БУСА · подключение")
                .setItems(new String[]{"Повторить подключение", "Доступ через интернет", "Подключение по Wi-Fi", "Адрес БУСА", "Закрыть приложение"}, (dialog, which) -> {
                    if (which == 0) connect();
                    if (which == 1) editCloud();
                    if (which == 2) {
                        remoteMode = false; preferences.edit().putString("mode", "local").apply();
                        connect();
                        try { startActivity(new Intent(Settings.ACTION_WIFI_SETTINGS)); }
                        catch (ActivityNotFoundException error) { Toast.makeText(this, "Открой Wi-Fi в настройках телефона", Toast.LENGTH_SHORT).show(); }
                    }
                    if (which == 3) editAddress();
                    if (which == 4) finish();
                }).setNegativeButton("Отмена", null).show();
    }

    private void editCloud() {
        LinearLayout form = new LinearLayout(this); form.setOrientation(LinearLayout.VERTICAL);
        form.setPadding(dp(20), dp(6), dp(20), 0);
        TextView hint = new TextView(this);
        hint.setText("ESP32 отправляет показания в Google, телефон получает их через интернет. Один раз укажите ссылку Apps Script и отдельный ключ просмотра BUSA_VIEW_TOKEN. Ключ ESP32 сюда не вводите.");
        EditText url = new EditText(this); url.setSingleLine(true); url.setHint("https://script.google.com/macros/s/…/exec");
        url.setText(preferences.getString("cloud_url", ""));
        url.setInputType(android.text.InputType.TYPE_CLASS_TEXT | android.text.InputType.TYPE_TEXT_VARIATION_URI);
        EditText token = new EditText(this); token.setSingleLine(true); token.setHint("Ключ просмотра BUSA_VIEW_TOKEN");
        token.setInputType(android.text.InputType.TYPE_CLASS_TEXT | android.text.InputType.TYPE_TEXT_VARIATION_PASSWORD);
        token.setText(preferences.getString("view_token", ""));
        form.addView(hint); form.addView(url); form.addView(token);
        ScrollView scroll=new ScrollView(this);scroll.addView(form);
        AlertDialog dialog = new AlertDialog.Builder(this).setTitle("БУСА через интернет").setView(scroll)
                .setPositiveButton("Подключить", null).setNegativeButton("Отмена", null).create();
        dialog.setOnShowListener(unused -> dialog.getButton(AlertDialog.BUTTON_POSITIVE).setOnClickListener(view -> {
            try {
                CloudClient client = new CloudClient(url.getText().toString(), token.getText().toString(), isDebug());
                preferences.edit().putString("cloud_url", client.endpoint).putString("view_token", client.token).putString("mode", "cloud").apply();
                remoteMode = true; dialog.dismiss(); connect();
            } catch (IllegalArgumentException error) { token.setError(error.getMessage()); }
        })); dialog.show();
    }

    private void editAddress() {
        LinearLayout form = new LinearLayout(this); form.setOrientation(LinearLayout.VERTICAL);
        form.setPadding(dp(20), dp(6), dp(20), 0);
        TextView hint = new TextView(this); hint.setText("Обычно 192.168.4.1. Менять нужно только для другого адреса прибора.");
        EditText input = new EditText(this); input.setSingleLine(true); input.setText(address);
        input.setInputType(android.text.InputType.TYPE_CLASS_TEXT | android.text.InputType.TYPE_TEXT_VARIATION_URI);
        form.addView(hint); form.addView(input);
        AlertDialog dialog = new AlertDialog.Builder(this).setTitle("Адрес БУСА").setView(form)
                .setPositiveButton("Подключиться", null).setNegativeButton("Отмена", null).create();
        dialog.setOnShowListener(unused -> dialog.getButton(AlertDialog.BUTTON_POSITIVE).setOnClickListener(view -> {
            try {
                String next = DeviceAddress.normalize(input.getText().toString());
                web.stopLoading(); address = next; remoteMode = false;
                preferences.edit().putString("address", next).putString("mode", "local").apply();
                dialog.dismiss(); connect();
            } catch (IllegalArgumentException error) { input.setError(error.getMessage()); }
        }));
        dialog.show();
    }

    @Override protected void onResume() {
        super.onResume(); resumed = true;
        if (web != null) {
            web.onResume(); web.resumeTimers();
            if (remoteMode) refreshCloud();
            else if (offlineShell && !connecting) connect();
        }
    }
    @Override protected void onPause() { resumed = false; if (web != null) { web.onPause(); web.pauseTimers(); } super.onPause(); }
    @Override protected void onDestroy() {
        destroyed = true; ++connectionAttempt;
        main.removeCallbacksAndMessages(null);
        ++cloudGeneration; cloudWorker.shutdownNow();
        if (connectionError != null) { connectionError.dismiss(); connectionError = null; }
        if (fileResult != null) { fileResult.onReceiveValue(null); fileResult = null; }
        if (connectivity != null && wifiCallback != null) {
            try { connectivity.unregisterNetworkCallback(wifiCallback); connectivity.bindProcessToNetwork(null); }
            catch (RuntimeException ignored) { }
        }
        if (web != null) { web.stopLoading(); web.destroy(); web = null; }
        super.onDestroy();
    }
}
