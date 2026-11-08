// Scanner: the badge opens an OPEN SoftAP. Any device that joins is listed and
// port-scanned by the badge. The verdict (SECURE / AT RISK) is shown on the badge
// e-ink (kept there) and mirrored to the phone via a captive portal.
// In scope: only scans devices that chose to connect to the badge own AP.
#include <string.h>
#include <stdio.h>
#include "badge.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_netif_types.h"
#include "esp_http_server.h"
#include "lwip/sockets.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#define ADDR "192.168.4.1"
#define MAXC 8
#define SCAN_PORTS 24
static const int scan_ports[SCAN_PORTS] = {
    21, 22, 23, 25, 53, 80, 110, 111, 135, 139, 143, 161, 389, 443, 445,
    587, 631, 993, 995, 1433, 3306, 5432, 8080, 8443
};
static const int risky_ports[] = {21, 23, 135, 139, 445, 5900, 5985, 8080, 8443, 3389};

typedef struct {
    uint8_t ip[4];
    uint8_t mac[6];
    int known;
    int open[SCAN_PORTS];
    int n_open;
    int at_risk;
} client_t;

static client_t cl[ MAXC ];
static char pagebuf[4096];
static int64_t now_ms(void) { return esp_timer_get_time() / 1000; }

static void mac_str(const uint8_t m[6], char *out) {
    snprintf(out, 18, "%02x:%02x:%02x:%02x:%02x:%02x", m[0],m[1],m[2],m[3],m[4],m[5]);
}
static void ip_str(const uint8_t ip[4], char *out) {
    snprintf(out, 16, "%u.%u.%u.%u", ip[0],ip[1],ip[2],ip[3]);
}

// refresh the client table from the AP station list (MAC + IP)
static void refresh_clients(void) {
    wifi_sta_list_t sl = {0};
    if (esp_wifi_ap_get_sta_list(&sl) != ESP_OK) return;
    wifi_sta_mac_ip_list_t ipm = {0};
    esp_wifi_ap_get_sta_list_with_ip(&sl, &ipm);
    for (int i = 0; i < MAXC; i++) cl[i].known = 0;
    int n = ipm.num < MAXC ? ipm.num : MAXC;
    for (int i = 0; i < n; i++) {
        memcpy(cl[i].mac, ipm.sta[i].mac, 6);
        memcpy(cl[i].ip, &ipm.sta[i].ip, 4);
        cl[i].known = 1;
    }
}

// TCP connect probe with a short timeout
static int probe(const uint8_t ip[4], int port) {
    int s = socket(AF_INET, SOCK_STREAM, 0);
    if (s < 0) return 0;
    struct timeval tv = { .tv_sec = 0, .tv_usec = 250000 };
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
    struct sockaddr_in a = { .sin_family = AF_INET, .sin_port = htons(port) };
    memcpy(&a.sin_addr.s_addr, ip, 4);
    int ok = (connect(s, (struct sockaddr*)&a, sizeof a) == 0);
    close(s);
    return ok;
}

static void scan_client(client_t *c) {
    c->n_open = 0;
    for (int i = 0; i < SCAN_PORTS; i++)
        if (probe(c->ip, scan_ports[i])) c->open[c->n_open++] = scan_ports[i];
    c->at_risk = 0;
    for (int i = 0; i < c->n_open; i++)
        for (size_t j = 0; j < sizeof risky_ports/sizeof risky_ports[0]; j++)
            if (c->open[i] == risky_ports[j]) c->at_risk = 1;
    if (c->n_open >= 5) c->at_risk = 1;
}

// captive-portal DNS: every A query answers with the badge IP
static void dns_task(void *arg) {
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in a = { .sin_family=AF_INET, .sin_port=htons(53), .sin_addr.s_addr=htonl(INADDR_ANY) };
    bind(s, (struct sockaddr*)&a, sizeof a);
    static const uint8_t ans[] = {0xC0,0x0C,0,1,0,1,0,0,0,60,0,4,192,168,4,1};
    for (;;) {
        uint8_t b[512];
        struct sockaddr_in from; socklen_t fl = sizeof from;
        int n = recvfrom(s, b, sizeof b - sizeof ans, 0, (struct sockaddr*)&from, &fl);
        int p = 12;
        while (p < n && b[p]) p += b[p]+1;
        p += 5;
        if (n < 12 || p > n) continue;
        int aq = b[p-4]==0 && b[p-3]==1;
        b[2]=0x81, b[3]=0x80, b[4]=0, b[5]=1, b[6]=0, b[7]=aq;
        memset(b+8, 0, 4);
        if (aq) memcpy(b+p, ans, sizeof ans), p += sizeof ans;
        sendto(s, b, p, 0, (struct sockaddr*)&from, fl);
    }
}

static void build_page(void) {
    int any_risk = 0, live = 0;
    for (int i=0;i<MAXC;i++) if (cl[i].known) { live++; if (cl[i].at_risk) any_risk=1; }
    const char *cls = any_risk ? "bad" : "ok";
    const char *verdict = any_risk ? "AT RISK" : "SECURE";
    char *p = pagebuf;
    p += snprintf(p, sizeof pagebuf-(size_t)(p-pagebuf),
        "<!doctype html><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
        "<title>Badge security scan</title><style>body{font:16px system-ui;background:#111;color:#eee;margin:24px}"
        ".v{font-size:44px;font-weight:700}.ok{color:#5fd06b}.bad{color:#ff5a5a}"
        ".c{border:1px solid #444;border-radius:10px;padding:12px;margin:12px 0}.h{font-size:13px;color:#999}"
        ".o{color:#ff9f43}.s{color:#5fd06b}</style>"
        "<h2>Badge security scan</h2>"
        "<div class=v %s>%s</div>"
        "<p>%d device(s) connected to this badge.</p>",
        cls, verdict, live);
    for (int i=0;i<MAXC;i++) if (cl[i].known) {
        char ip[16], mac[18];
        const char *oc = cl[i].at_risk ? "o" : "s";
        ip_str(cl[i].ip, ip); mac_str(cl[i].mac, mac);
        p += snprintf(p, sizeof pagebuf-(size_t)(p-pagebuf),
            "<div class=c><div class=h>%s  %s</div><div class=%s>%d open port(s)</div>",
            ip, mac, oc, cl[i].n_open);
        for (int k=0;k<cl[i].n_open;k++)
            p += snprintf(p, sizeof pagebuf-(size_t)(p-pagebuf), "<span class=o>%d</span> ", cl[i].open[k]);
        p += snprintf(p, sizeof pagebuf-(size_t)(p-pagebuf), "</div>");
    }
    p += snprintf(p, sizeof pagebuf-(size_t)(p-pagebuf),
        "<p class=h>Rescanning every 10s. Open ports are services reachable on this badge AP.</p>");
}

static esp_err_t get_page(httpd_req_t *r) { build_page(); return httpd_resp_send(r, pagebuf, strlen(pagebuf)); }
static esp_err_t to_portal(httpd_req_t *r, httpd_err_code_t err) {
    httpd_resp_set_status(r, "303 See Other");
    httpd_resp_set_hdr(r, "Location", "http://" ADDR "/");
    return httpd_resp_sendstr(r, "Badge security scan: http://" ADDR "/");
}

static void draw_report(void) {
    memset(fb, 0, sizeof fb);
    int live = 0, risk = 0;
    for (int i=0;i<MAXC;i++) if (cl[i].known) { live++; if (cl[i].at_risk) risk++; }
    const char *v = risk ? "AT RISK" : "SECURE";
    text((W - (int)strlen(v)*40)/2, TOP+40, 5, v, risk);
    text(48, TOP+150, 3, "Devices on this badge:", true);
    int y = TOP+190, shown = 0;
    for (int i=0;i<MAXC && shown<6;i++) {
        if (!cl[i].known) continue;
        char ip[16], line[80];
        ip_str(cl[i].ip, ip);
        snprintf(line, sizeof line, "%s  %d open%s", ip, cl[i].n_open, cl[i].at_risk ? " !":"");
        text(48, y, 3, line, cl[i].at_risk);
        if (cl[i].n_open) {
            char ports[64] = "ports: ";
            for (int k=0;k<cl[i].n_open && k<6;k++)
                snprintf(ports+strlen(ports), 64-strlen(ports), "%d ", cl[i].open[k]);
            text(80, y+34, 2, ports, true);
        }
        y += 70; shown++;
    }
    if (!live) {
        text(48, y, 3, "No devices connected yet.", true);
        text(48, y+40, 2, "Join the open AP from your phone.", true);
        text(48, y+70, 2, "The scan page opens by itself.", true);
    }
    ui_frame(1);
}

void app_scanner(void)
{
    char id[5]; badge_id(id);
    wifi_on();
    esp_wifi_stop();
    esp_netif_create_default_wifi_ap();
    wifi_config_t c = {0};
    c.ap.ssid_len = snprintf((char*)c.ap.ssid, sizeof c.ap.ssid, "Badge-SCAN");
    c.ap.channel = 1;
    c.ap.max_connection = MAXC;
    c.ap.authmode = WIFI_AUTH_OPEN;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &c));
    ESP_ERROR_CHECK(esp_wifi_start());

    xTaskCreate(dns_task, "dns", 3072, NULL, 4, NULL);
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = 6144;
    cfg.lru_purge_enable = true;
    httpd_handle_t srv;
    ESP_ERROR_CHECK(httpd_start(&srv, &cfg));
    const httpd_uri_t uris[] = { { .uri="/", .method=HTTP_GET, .handler=get_page } };
    httpd_register_uri_handler(srv, &uris[0]);
    httpd_register_err_handler(srv, HTTPD_404_NOT_FOUND, to_portal);

    ui_keys(NULL, NULL, "Rescan", "Home");
    int64_t last_scan = 0, last_draw = 0;
    for (;;) {
        int64_t t = now_ms();
        refresh_clients();
        if (t - last_scan > 10000) {
            for (int i=0;i<MAXC;i++) if (cl[i].known) scan_client(&cl[i]);
            last_scan = t;
        }
        if (t - last_draw > 1500) { draw_report(); last_draw = t; }
        if (btn_held() & (1<<BTN_A)) last_scan = 0;
        if (btn_held() & (1<<BTN_B)) break;
        ms(500);
    }
    httpd_stop(srv);
}
