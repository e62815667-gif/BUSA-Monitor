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
import android.widget.TextView;
import android.widget.Toast;
import org.json.JSONObject;
import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/** Native container; the ESP32 remains the source of UI and device behaviour. */
public final class MainActivity extends Activity {
    private static final int FILE_REQUEST = 14;
    private static final int BACKGROUND = Color.rgb(6, 14, 21);
    private final Handler main = new Handler(Looper.getMainLooper());
    private final ExecutorService worker = Executors.newSingleThreadExecutor();
    private WebView web;
    private ProgressBar progress;
    private ConnectivityManager connectivity;
    private ConnectivityManager.NetworkCallback wifiCallback;
    private volatile String address;
    private volatile boolean offlineShell;
    private volatile boolean destroyed;
    private int connectionAttempt;
    private ValueCallback<Uri[]> fileResult;
    private SharedPreferences preferences;

    @Override public void onCreate(Bundle state) {
        super.onCreate(state);
        preferences = getSharedPreferences("connection", MODE_PRIVATE);
        try { address = DeviceAddress.normalize(preferences.getString("address", DeviceAddress.DEFAULT)); }
        catch (IllegalArgumentException error) { address = DeviceAddress.DEFAULT; }
        FrameLayout root = new FrameLayout(this);
        root.setBackgroundColor(BACKGROUND);
        web = new WebView(this);
        web.setBackgroundColor(BACKGROUND);
        root.addView(web, new FrameLayout.LayoutParams(-1, -1));
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
        bindWifi();
        connect();
    }

    private int dp(int value) { return Math.round(value * getResources().getDisplayMetrics().density); }

    private void bindWifi() {
        connectivity = (ConnectivityManager) getSystemService(CONNECTIVITY_SERVICE);
        wifiCallback = new ConnectivityManager.NetworkCallback() {
            @Override public void onAvailable(Network network) {
                if (!destroyed) {
                    try { connectivity.bindProcessToNetwork(network); }
                    catch (SecurityException ignored) { }
                }
            }
            @Override public void onLost(Network network) {
                if (network.equals(connectivity.getBoundNetworkForProcess())) {
                    try { connectivity.bindProcessToNetwork(null); }
                    catch (SecurityException ignored) { }
                }
            }
        };
        try {
            // A local AP without internet must still win over mobile data.
            for (Network network : connectivity.getAllNetworks()) {
                NetworkCapabilities caps = connectivity.getNetworkCapabilities(network);
                if (caps != null && caps.hasTransport(NetworkCapabilities.TRANSPORT_WIFI)) {
                    connectivity.bindProcessToNetwork(network); break;
                }
            }
            connectivity.registerNetworkCallback(new NetworkRequest.Builder()
                    .addTransportType(NetworkCapabilities.TRANSPORT_WIFI)
                    .removeCapability(NetworkCapabilities.NET_CAPABILITY_INTERNET).build(), wifiCallback);
        } catch (SecurityException ignored) { }
    }

    private void connect() {
        final String target = address;
        final int attempt = ++connectionAttempt;
        progress.setVisibility(View.VISIBLE);
        worker.execute(() -> {
            boolean reachable = false;
            HttpURLConnection connection = null;
            try {
                connection = (HttpURLConnection) new URL(target + "/api/status").openConnection();
                connection.setInstanceFollowRedirects(false);
                connection.setConnectTimeout(1500);
                connection.setReadTimeout(1500);
                connection.setUseCaches(false);
                if (connection.getResponseCode() == 200) {
                    String body = read(connection.getInputStream(), 65536);
                    JSONObject status = new JSONObject(body);
                    reachable = status.has("temperatures") && status.has("water");
                }
            } catch (Exception ignored) { }
            finally { if (connection != null) connection.disconnect(); }
            final boolean online = reachable;
            main.post(() -> {
                if (destroyed || attempt != connectionAttempt) return;
                if (online) {
                    offlineShell = false;
                    web.loadUrl(target + "/");
                    main.postDelayed(() -> {
                        if (!destroyed && attempt == connectionAttempt && progress.getVisibility() == View.VISIBLE) {
                            web.stopLoading(); showOffline();
                        }
                    }, 4500);
                } else showOffline();
            });
        });
    }

    private void showOffline() {
        if (destroyed) return;
        offlineShell = true;
        try {
            String html = read(getAssets().open("local/index.html"), 2000000);
            web.loadDataWithBaseURL(address + "/", html, "text/html", "UTF-8", null);
            Toast.makeText(this, "Для показаний подключи телефон к Wi-Fi БУСА. Меню подключения — кнопкой «Назад» на главной.", Toast.LENGTH_LONG).show();
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
            if (!DeviceAddress.sameOrigin(address, url)) {
                return new WebResourceResponse("text/plain", "UTF-8", new java.io.ByteArrayInputStream(new byte[0]));
            }
            if (offlineShell && request.getMethod().equals("GET")) {
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
        @Override public void onPageFinished(WebView view, String url) { progress.setVisibility(View.GONE); }
        @Override public void onReceivedError(WebView view, WebResourceRequest request, WebResourceError error) {
            if (request.isForMainFrame() && !offlineShell) showOffline();
        }
        @Override public void onReceivedHttpError(WebView view, WebResourceRequest request, WebResourceResponse response) {
            if (request.isForMainFrame() && !offlineShell) showOffline();
        }
    }

    private boolean navigate(String url, boolean userGesture) {
        if (DeviceAddress.sameOrigin(address, url)) return false;
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
                .setItems(new String[]{"Повторить подключение", "Сеть Wi-Fi", "Адрес БУСА", "Закрыть приложение"}, (dialog, which) -> {
                    if (which == 0) connect();
                    if (which == 1) {
                        try { startActivity(new Intent(Settings.ACTION_WIFI_SETTINGS)); }
                        catch (ActivityNotFoundException error) { Toast.makeText(this, "Открой Wi-Fi в настройках телефона", Toast.LENGTH_SHORT).show(); }
                    }
                    if (which == 2) editAddress();
                    if (which == 3) finish();
                }).setNegativeButton("Отмена", null).show();
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
                web.stopLoading(); address = next;
                preferences.edit().putString("address", next).apply();
                dialog.dismiss(); connect();
            } catch (IllegalArgumentException error) { input.setError(error.getMessage()); }
        }));
        dialog.show();
    }

    @Override protected void onResume() { super.onResume(); if (web != null) { web.onResume(); web.resumeTimers(); } }
    @Override protected void onPause() { if (web != null) { web.onPause(); web.pauseTimers(); } super.onPause(); }
    @Override protected void onDestroy() {
        destroyed = true; ++connectionAttempt;
        main.removeCallbacksAndMessages(null); worker.shutdownNow();
        if (fileResult != null) { fileResult.onReceiveValue(null); fileResult = null; }
        if (connectivity != null && wifiCallback != null) {
            try { connectivity.unregisterNetworkCallback(wifiCallback); connectivity.bindProcessToNetwork(null); }
            catch (RuntimeException ignored) { }
        }
        if (web != null) { web.stopLoading(); web.destroy(); web = null; }
        super.onDestroy();
    }
}
