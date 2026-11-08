// Scanner: the badge opens an OPEN SoftAP. Every device that joins is port-scanned
// by the badge, ranked by a RISK SCORE (a leaderboard, most-risky on top), and the
// result is shown on the badge e-ink and mirrored to the phone via a captive portal.
// When no devices are connected, the badge advertises ZTLP (zero-trust / minimize
// your attack surface). In scope: only scans devices that chose to join the AP.
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
// ports we probe; weight = how risky it is that it is open
typedef struct { int port; int w; } sport_t;
static const sport_t scan_ports[SCAN_PORTS] = {
    {21,12},{22,4},{23,14},{25,8},{53,5},{80,6},{110,8},{111,7},
    {135,12},{139,12},{143,8},{161,6},{389,8},{443,2},{445,14},
    {587,6},{631,5},{993,8},{995,8},{1433,10},{3306,10},{5432,10},{8080,10},{8443,10},
};

typedef struct {
    uint8_t ip[4];
    uint8_t mac[6];
    int known;
    int open[SCAN_PORTS];
    int n_open;
    int score;        // risk score (higher = riskier)
} client_t;

static client_t cl[ MAXC ];
static char pagebuf[4608];
static int64_t now_ms(void) { return esp_timer_get_time() / 1000; }

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
    c->n_open = 0; c->score = 0;
    for (int i = 0; i < SCAN_PORTS; i++) {
        if (probe(c->ip, scan_ports[i].port)) {
            c->open[c->n_open++] = scan_ports[i].port;
            c->score += scan_ports[i].w;          // add the port's weight to the score
        }
    }
    if (c->n_open >= 5) c->score += 15;           // bonus: a lot of services open
}

// leaderboard order: highest score first (stable by IP on ties).
// rank = 0 is the most-risky device.
static int leader_idx(int rank) {
    int order[MAXC], n = 0;
    for (int i = 0; i < MAXC; i++) if (cl[i].known) order[n++] = i;
    // selection sort by score desc, tie-break by IP asc
    for (int i = 0; i < n; i++) {
        int best = i;
        for (int j = i + 1; j < n; j++) {
            if (cl[order[j]].score > cl[order[best]].score ||
                (cl[order[j]].score == cl[order[best]].score &&
                 memcmp(cl[order[j]].ip, cl[order[best]].ip, 4) < 0))
                best = j;
        }
        int t = order[i]; order[i] = order[best]; order[best] = t;
    }
    return (rank < n) ? order[rank] : -1;
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

// the phone page: a leaderboard table (score, IP, ports) + the ZTLP ad
static void build_page(void) {
    int live = 0, top = -1;
    for (int i=0;i<MAXC;i++) if (cl[i].known) { live++; if (cl[i].score > (top<0? -1: cl[top].score)) top=i; }
    const char *verdict = (top<0 || cl[top].score<10) ? "LOW RISK" : (cl[top].score<40 ? "MODERATE" : "HIGH RISK");
    char *p = pagebuf;
    p += snprintf(p, sizeof pagebuf-(size_t)(p-pagebuf),
        "<!doctype html><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
        "<title>Badge risk scoreboard</title><style>body{font:15px system-ui;background:#111;color:#eee;margin:20px}"
        ".v{font-size:42px;font-weight:700}.low{color:#5fd06b}.mod{color:#ff9f43}.high{color:#ff5a5a}"
        "table{width:100%;border-collapse:collapse}th,td{border:1px solid #333;padding:8px;text-align:left}"
        ".r{color:#ff5a5a;font-weight:700}.m{color:#ff9f43}.s{color:#5fd06b}"
        ".z{margin-top:26px;border:1px solid #2a6;padding:14px;border-radius:10px;background:#0c2418}"
        ".z b{font-size:20px}.o{color:#ff9f43}</style>"
        "<h2>Badge risk scoreboard</h2><div class=v %s>%s</div><p>%d device(s) on this badge, ranked by risk.</p>"
        "<table><tr><th>#</th><th>Risk</th><th>Device</th><th>Open ports</th></tr>",
        (top<0||cl[top].score<10)?"low":(cl[top].score<40?"mod":"high"), verdict, live);
    int rank=0;
    for (int r=0;r<MAXC;r++) {
        int i = leader_idx(r);
        if (i<0) break;
        rank++;
        const char *cls = cl[i].score<10 ? "s" : (cl[i].score<40 ? "m" : "r");
        char ip[16]; ip_str(cl[i].ip, ip);
        p += snprintf(p, sizeof pagebuf-(size_t)(p-pagebuf),
            "<tr><td>%d</td><td class=%s>%d</td><td>%s</td><td>", rank, cls, cl[i].score, ip);
        for (int k=0;k<cl[i].n_open;k++)
            p += snprintf(p, sizeof pagebuf-(size_t)(p-pagebuf), "<span class=o>%d</span> ", cl[i].open[k]);
        p += snprintf(p, sizeof pagebuf-(size_t)(p-pagebuf), "</td></tr>");
    }
    p += snprintf(p, sizeof pagebuf-(size_t)(p-pagebuf),
        "</table>"
        "<div class=z><b>ZTLP</b> &mdash; Secure Identity-First Network Overlay.<br>"
        "Authenticating every device before it gets network access shrinks your attack surface: "
        "unauthorized traffic is rejected before it costs anything. <b>Minimize your attack surface.</b>"
        "</div>");
}

static esp_err_t get_page(httpd_req_t *r) { build_page(); return httpd_resp_send(r, pagebuf, strlen(pagebuf)); }
static esp_err_t to_portal(httpd_req_t *r, httpd_err_code_t err) {
    httpd_resp_set_status(r, "303 See Other");
    httpd_resp_set_hdr(r, "Location", "http://" ADDR "/");
    return httpd_resp_sendstr(r, "Badge risk scoreboard: http://" ADDR "/");
}

// ---- e-ink: the leaderboard (risk score + ports), or the ZTLP ad when idle ----
static void draw_leaderboard(void) {
    memset(fb, 0, sizeof fb);
    text(48, TOP+34, 4, "RISK SCOREBOARD", true);
    int y = TOP+96, rank = 0;
    for (int r = 0; r < 6 && rank < 6; r++) {
        int i = leader_idx(rank);
        if (i < 0) break;
        rank++;
        char ip[16], line[90], ports[64];
        ip_str(cl[i].ip, ip);
        snprintf(line, sizeof line, "#%d  %d  %s", rank, cl[i].score, ip);
        // risk color: red high / (no red on 3-colour panel -> use black, but mark risk)
        int red = cl[i].score >= 40;
        text(48, y, 3, line, red);
        snprintf(ports, sizeof ports, "open: ");
        int k = 0;
        for (int j = 0; j < cl[i].n_open && k < 6; j++) {
            snprintf(ports + strlen(ports), 64 - strlen(ports), "%d ", cl[i].open[j]);
            k++;
        }
        if (!k) strcpy(ports, "open: none");
        text(80, y + 32, 2, ports, false);
        y += 74;
    }
    if (!rank) {
        text(48, y + 20, 3, "No devices on this badge yet.", true);
    }
    ui_frame(1);
}

static void draw_ztlp_ad(void) {
    memset(fb, 0, sizeof fb);
    // banner
    box(0, TOP, W, 8, true);
    box(0, TOP+120, W, 4, true);
    text((W - (int)strlen("ZTLP"))*1/2, TOP+40, 8, "ZTLP", true);
    text((W - (int)strlen("Secure Identity-First Network Overlay"))/2, TOP+150, 2,
         "Secure Identity-First Network Overlay", true);
    // the pitch
    int y = TOP+210;
    text(48, y,   3, "Minimize your attack surface.", true);
    y += 56;
    text(48, y,   2, "ZTLP authenticates every device before it gets network access.", true);
    y += 34;
    text(48, y,   2, "Unauthorized traffic is rejected before it costs anything.", true);
    y += 34;
    text(48, y,   2, "Legitimate connectivity stays private by default.", true);
    y += 60;
    text(48, y,   2, "Connect to this badge to see the live risk", true);
    y += 30;
    text(48, y,   2, "scoreboard of the devices on the network.", true);
    ui_frame(1);
}

static void draw_screen(void) {
    int live = 0;
    for (int i=0;i<MAXC;i++) if (cl[i].known) live++;
    if (live) draw_leaderboard();
    else      draw_ztlp_ad();
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
        if (t - last_draw > 1500) { draw_screen(); last_draw = t; }
        if (btn_held() & (1<<BTN_A)) last_scan = 0;
        if (btn_held() & (1<<BTN_B)) break;
        ms(500);
    }
    httpd_stop(srv);
}
