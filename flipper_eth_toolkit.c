/*
 * Flipper Ethernet Toolkit v0.2 (experimental)
 * W5500 5V plug-in module on external SPI; Momentum mntm-012.
 * DHCP/UDP socket 0, gateway ICMP echo/IPRAW socket 1.
 * SPDX-License-Identifier: MIT
 */
#include <furi.h>
#include <furi_hal.h>
#include <furi_hal_spi.h>
#include <furi_hal_spi_config.h>
#include <furi_hal_resources.h>
#include <furi_hal_version.h>
#include <furi_hal_power.h>
#include <gui/gui.h>
#include <input/input.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define SPI_TIMEOUT 100U
#define SPI_CHUNK 128U
#define PAGES 4U
#define DHCP_TIMEOUT_MS 4500U
#define PING_TIMEOUT_MS 1800U
#define S_DHCP 0U
#define S_PING 1U

/* W5500 common registers */
#define REG_GAR 0x0001U
#define REG_SUBR 0x0005U
#define REG_SHAR 0x0009U
#define REG_SIPR 0x000FU
#define REG_PHY 0x002EU
#define REG_VERSION 0x0039U
/* W5500 socket register offsets */
#define SN_MR 0x0000U
#define SN_CR 0x0001U
#define SN_IR 0x0002U
#define SN_SR 0x0003U
#define SN_PORT 0x0004U
#define SN_PROTO 0x0014U
#define SN_DIPR 0x000CU
#define SN_DPORT 0x0010U
#define SN_TX_FSR 0x0020U
#define SN_TX_WR 0x0024U
#define SN_RX_RSR 0x0026U
#define SN_RX_RD 0x0028U
#define CR_OPEN 0x01U
#define CR_CLOSE 0x10U
#define CR_SEND 0x20U
#define CR_RECV 0x40U
#define MODE_UDP 0x02U
#define MODE_IPRAW 0x03U
#define SOCK_UDP 0x22U
#define SOCK_IPRAW 0x32U

static bool expired(uint32_t deadline) {
    return (int32_t)(furi_get_tick() - deadline) >= 0;
}
static uint32_t after_ms(uint32_t ms) {
    return furi_get_tick() + furi_ms_to_ticks(ms);
}
static bool zero_ip(const uint8_t ip[4]) {
    return !(ip[0] | ip[1] | ip[2] | ip[3]);
}

/* W5500 SPI: control = (block select << 3) | (write ? 4 : 0); VDM. */
static bool spi_transfer(uint8_t block, uint16_t address, uint8_t* data, size_t n, bool write) {
    const FuriHalSpiBusHandle* h = &furi_hal_spi_bus_handle_external;
    uint8_t tx[SPI_CHUNK + 3];
    uint8_t rx[SPI_CHUNK + 3];
    size_t done = 0;
    while(done < n) {
        const size_t count = n - done > SPI_CHUNK ? SPI_CHUNK : n - done;
        const uint16_t at = (uint16_t)(address + done);
        tx[0] = (uint8_t)(at >> 8);
        tx[1] = (uint8_t)at;
        tx[2] = (uint8_t)((block << 3) | (write ? 0x04 : 0));
        if(write) memcpy(tx + 3, data + done, count);
        else memset(tx + 3, 0, count);
        furi_hal_spi_acquire(h);
        bool ok = furi_hal_spi_bus_trx(h, tx, rx, count + 3, SPI_TIMEOUT);
        furi_hal_spi_release(h);
        if(!ok) return false;
        if(!write) memcpy(data + done, rx + 3, count);
        done += count;
    }
    return true;
}
static bool reg_read(uint8_t block, uint16_t addr, uint8_t* dst, size_t n) {
    return spi_transfer(block, addr, dst, n, false);
}
static bool reg_write(uint8_t block, uint16_t addr, const uint8_t* src, size_t n) {
    /* SPI function never mutates input in write mode */
    return spi_transfer(block, addr, (uint8_t*)src, n, true);
}
static bool rd8(uint8_t block, uint16_t addr, uint8_t* out) {
    return reg_read(block, addr, out, 1);
}
static bool wr8(uint8_t block, uint16_t addr, uint8_t v) {
    return reg_write(block, addr, &v, 1);
}
static bool rd16(uint8_t block, uint16_t addr, uint16_t* out) {
    uint8_t v[2];
    if(!reg_read(block, addr, v, 2)) return false;
    *out = (uint16_t)((v[0] << 8) | v[1]);
    return true;
}
static bool wr16(uint8_t block, uint16_t addr, uint16_t v) {
    const uint8_t data[2] = {(uint8_t)(v >> 8), (uint8_t)v};
    return reg_write(block, addr, data, 2);
}
static uint8_t sblock(uint8_t s) { return (uint8_t)(4 * s + 1); }
static uint8_t txblock(uint8_t s) { return (uint8_t)(4 * s + 2); }
static uint8_t rxblock(uint8_t s) { return (uint8_t)(4 * s + 3); }
static bool sn8(uint8_t s, uint16_t off, uint8_t* v) { return rd8(sblock(s), off, v); }
static bool sw8(uint8_t s, uint16_t off, uint8_t v) { return wr8(sblock(s), off, v); }
static bool sn16(uint8_t s, uint16_t off, uint16_t* v) { return rd16(sblock(s), off, v); }
static bool sw16(uint8_t s, uint16_t off, uint16_t v) { return wr16(sblock(s), off, v); }
static bool command(uint8_t s, uint8_t cmd) {
    if(!sw8(s, SN_CR, cmd)) return false;
    for(unsigned i=0; i<20; ++i) {
        uint8_t val = 255;
        if(!sn8(s, SN_CR, &val)) return false;
        if(val == 0) return true;
        furi_delay_ms(1);
    }
    return false;
}
static void sock_close(uint8_t s) {
    (void)command(s, CR_CLOSE);
    (void)sw8(s, SN_IR, 0x1FU);
}
static bool sock_open(uint8_t s, uint8_t mode, uint16_t port, uint8_t proto) {
    sock_close(s);
    if(!sw8(s, SN_MR, mode)) return false;
    if(!sw16(s, SN_PORT, port)) return false;
    if(mode == MODE_IPRAW && !sw8(s, SN_PROTO, proto)) return false;
    if(!command(s, CR_OPEN)) return false;
    uint8_t status=0;
    return sn8(s, SN_SR, &status) && status == (mode == MODE_UDP ? SOCK_UDP : SOCK_IPRAW);
}
static bool sock_send(uint8_t s, const uint8_t* data, uint16_t count, const uint8_t dst[4], uint16_t port) {
    uint16_t free_size=0, ptr=0;
    if(!sn16(s, SN_TX_FSR, &free_size) || free_size < count) return false;
    if(!reg_write(sblock(s), SN_DIPR, dst, 4)) return false;
    if(port && !sw16(s, SN_DPORT, port)) return false;
    if(!sn16(s, SN_TX_WR, &ptr)) return false;
    if(!reg_write(txblock(s), ptr, data, count)) return false;
    if(!sw16(s, SN_TX_WR, (uint16_t)(ptr + count))) return false;
    if(!sw8(s, SN_IR, 0x18)) return false;
    return command(s, CR_SEND);
}
/* Receive one W5500 UDP/IPRAW message, strip 8/6-byte chip metadata.
 * Returns 1 packet, 0 no full packet, -1 SPI error. Oversized packet is discarded.
 */
static int sock_receive(uint8_t s, bool udp, uint8_t* dst, size_t cap,
                        uint16_t* plen, uint8_t from[4]) {
    uint16_t total=0, ptr=0;
    const uint16_t header_len = udp ? 8U : 6U;
    if(!sn16(s, SN_RX_RSR, &total)) return -1;
    if(total < header_len) return 0;
    if(!sn16(s, SN_RX_RD, &ptr)) return -1;
    uint8_t header[8];
    if(!reg_read(rxblock(s), ptr, header, header_len)) return -1;
    uint16_t n = (uint16_t)((header[header_len-2] << 8) | header[header_len-1]);
    if(total < header_len + n) return 0;
    memcpy(from, header, 4);
    if(n <= cap && n > 0) {
        if(!reg_read(rxblock(s), (uint16_t)(ptr+header_len), dst, n)) return -1;
    }
    if(!sw16(s, SN_RX_RD, (uint16_t)(ptr + header_len + n))) return -1;
    if(!command(s, CR_RECV)) return -1;
    if(n > cap || n == 0) return 0;
    *plen = n;
    return 1;
}

typedef enum {D_IDLE, D_OFFER, D_ACK, D_LEASE, D_FAIL} DHCPStage;
typedef enum {P_IDLE, P_WAIT, P_OK, P_TIMEOUT, P_FAIL} PingStage;
typedef struct {
    uint8_t version, phy, mac[6], ip[4], mask[4], gateway[4], dns[4];
    bool found, otg_ok;
    DHCPStage dhcp;
    uint8_t dhcp_attempt;
    uint8_t offer_ip[4], server[4];
    uint32_t dhcp_xid, dhcp_deadline;
    PingStage ping;
    uint16_t ping_id;
    uint32_t ping_started, ping_deadline, ping_rtt_ms;
    uint8_t page;
} AppState;

typedef struct {
    FuriMutex* mutex;
    AppState s;
} EthernetApp;

static void update_physical(AppState* s) {
    uint8_t version=0, phy=0;
    bool ok = rd8(0, REG_VERSION, &version);
    ok = ok && rd8(0, REG_PHY, &phy);
    s->version = version;
    s->phy = phy;
    s->found = ok && version == 0x04;
}
static void generate_mac(uint8_t mac[6]) {
    const uint8_t* uid = furi_hal_version_uid();
    size_t len = furi_hal_version_uid_size();
    mac[0]=0x02; mac[1]=0x46; mac[2]=0x5A;
    for(unsigned i=0;i<3;i++) mac[3+i] = (uid && len) ? (uint8_t)(uid[i%len] ^ uid[(i+3)%len]) : (uint8_t)(0x40+i);
}
static bool config_network(AppState* s) {
    static const uint8_t zeros[4] = {0};
    return reg_write(0, REG_SHAR, s->mac, 6) &&
           reg_write(0, REG_GAR, zeros, 4) &&
           reg_write(0, REG_SUBR, zeros, 4) &&
           reg_write(0, REG_SIPR, zeros, 4);
}

/* Minimal DHCPv4. We send DISCOVER and REQUEST (RFC 2131); no persistent lease yet. */
static bool dhcp_packet(AppState* s, uint8_t kind) {
    uint8_t pkt[320] = {0};
    pkt[0]=1; pkt[1]=1; pkt[2]=6;
    pkt[4]=(uint8_t)(s->dhcp_xid>>24);
    pkt[5]=(uint8_t)(s->dhcp_xid>>16);
    pkt[6]=(uint8_t)(s->dhcp_xid>>8);
    pkt[7]=(uint8_t)s->dhcp_xid;
    pkt[10]=0x80; /* broadcast reply flag */
    memcpy(pkt+28,s->mac,6);
    pkt[236]=0x63; pkt[237]=0x82; pkt[238]=0x53; pkt[239]=0x63;
    size_t p=240;
    pkt[p++]=53; pkt[p++]=1; pkt[p++]=kind;
    pkt[p++]=61; pkt[p++]=7; pkt[p++]=1;
    memcpy(pkt+p,s->mac,6); p+=6;
    pkt[p++]=55; pkt[p++]=4; pkt[p++]=1; pkt[p++]=3; pkt[p++]=6; pkt[p++]=51;
    if(kind==3) {
        pkt[p++]=50; pkt[p++]=4; memcpy(pkt+p,s->offer_ip,4); p+=4;
        pkt[p++]=54; pkt[p++]=4; memcpy(pkt+p,s->server,4); p+=4;
    }
    pkt[p++]=255;
    const uint8_t broadcast[4]={255,255,255,255};
    return sock_send(S_DHCP,pkt,(uint16_t)p,broadcast,67);
}
static void dhcp_fail(AppState* s) {
    sock_close(S_DHCP);
    s->dhcp=D_FAIL;
}
static void dhcp_start(AppState* s) {
    sock_close(S_PING);
    s->ping=P_IDLE;
    s->dhcp=D_IDLE;
    memset(s->ip,0,4); memset(s->mask,0,4);
    memset(s->gateway,0,4); memset(s->dns,0,4);
    if(!s->found || !(s->phy & 1)) { s->dhcp=D_FAIL; return; }
    if(!config_network(s) || !sock_open(S_DHCP,MODE_UDP,68,0)) {
        dhcp_fail(s); return;
    }
    s->dhcp_xid = furi_get_tick() ^ ((uint32_t)s->mac[3]<<24) ^
                  ((uint32_t)s->mac[4]<<16) ^ ((uint32_t)s->mac[5]<<8) ^ 0x71F52A19U;
    s->dhcp_attempt=1;
    if(!dhcp_packet(s,1)) { dhcp_fail(s); return; }
    s->dhcp=D_OFFER;
    s->dhcp_deadline=after_ms(DHCP_TIMEOUT_MS);
}

typedef struct {
    uint8_t msg_type;
    uint8_t ip[4], server[4], mask[4], router[4], dns[4];
} DhcpReply;
static bool parse_dhcp(const uint8_t* b, uint16_t n, const AppState* s, DhcpReply* reply) {
    if(n<240 || b[0]!=2 || b[1]!=1 || b[2]!=6) return false;
    if(b[4]!=(uint8_t)(s->dhcp_xid>>24) || b[5]!=(uint8_t)(s->dhcp_xid>>16) ||
       b[6]!=(uint8_t)(s->dhcp_xid>>8) || b[7]!=(uint8_t)s->dhcp_xid) return false;
    if(memcmp(b+28,s->mac,6)!=0 || memcmp(b+236,"\x63\x82\x53\x63",4)!=0) return false;
    memset(reply,0,sizeof(*reply));
    memcpy(reply->ip,b+16,4);
    size_t i=240;
    while(i<n) {
        uint8_t tag=b[i++];
        if(tag==255) break;
        if(tag==0) continue;
        if(i>=n) return false;
        uint8_t length=b[i++];
        if(i+length>n) return false;
        if(tag==53 && length==1) reply->msg_type=b[i];
        else if(tag==54 && length>=4) memcpy(reply->server,b+i,4);
        else if(tag==1 && length>=4) memcpy(reply->mask,b+i,4);
        else if(tag==3 && length>=4) memcpy(reply->router,b+i,4);
        else if(tag==6 && length>=4) memcpy(reply->dns,b+i,4);
        i+=length;
    }
    return reply->msg_type != 0;
}
static void dhcp_tick(AppState* s) {
    if(s->dhcp != D_OFFER && s->dhcp != D_ACK) return;
    /* DHCP packets <= 640 bytes; W5500 socket RX buffer is 2KB. */
    uint8_t payload[640];
    for(unsigned i=0;i<3;i++) {
        uint16_t size=0; uint8_t peer[4]={0};
        int got=sock_receive(S_DHCP,true,payload,sizeof(payload),&size,peer);
        if(got<0) { dhcp_fail(s); return; }
        if(got==0) break;
        DhcpReply r;
        if(!parse_dhcp(payload,size,s,&r)) continue;
        if(s->dhcp==D_OFFER && r.msg_type==2 && !zero_ip(r.ip) && !zero_ip(r.server)) {
            memcpy(s->offer_ip,r.ip,4); memcpy(s->server,r.server,4);
            if(!dhcp_packet(s,3)) { dhcp_fail(s); return; }
            s->dhcp=D_ACK; s->dhcp_deadline=after_ms(DHCP_TIMEOUT_MS);
            break;
        }
        if(s->dhcp==D_ACK && r.msg_type==5 && !zero_ip(r.ip)) {
            if(!zero_ip(r.server) && memcmp(r.server,s->server,4)!=0) continue;
            memcpy(s->ip,r.ip,4);
            memcpy(s->mask,r.mask,4);
            memcpy(s->gateway,r.router,4);
            memcpy(s->dns,r.dns,4);
            if(reg_write(0,REG_SIPR,s->ip,4) && reg_write(0,REG_SUBR,s->mask,4)
               && reg_write(0,REG_GAR,s->gateway,4)) {
                s->dhcp=D_LEASE; sock_close(S_DHCP);
            } else dhcp_fail(s);
            return;
        }
        if(s->dhcp==D_ACK && r.msg_type==6) { dhcp_fail(s); return; }
    }
    if(!expired(s->dhcp_deadline)) return;
    if(s->dhcp==D_OFFER && s->dhcp_attempt<3) {
        s->dhcp_attempt++;
        if(!dhcp_packet(s,1)) { dhcp_fail(s); return; }
        s->dhcp_deadline=after_ms(DHCP_TIMEOUT_MS);
    } else dhcp_fail(s);
}

static uint16_t checksum(const uint8_t* b, size_t len) {
    uint32_t sum=0;
    for(size_t i=0;i<len;i+=2) {
        sum += (uint16_t)(b[i]<<8 | (i+1<len ? b[i+1] : 0));
    }
    while(sum>>16) sum=(sum&0xffffU)+(sum>>16);
    return (uint16_t)~sum;
}
static void ping_start(AppState* s) {
    sock_close(S_PING);
    s->ping=P_IDLE;
    if(s->dhcp!=D_LEASE || zero_ip(s->gateway) || !(s->phy&1)) {s->ping=P_FAIL;return;}
    if(!sock_open(S_PING,MODE_IPRAW,0,1)) {s->ping=P_FAIL;return;}
    uint8_t icmp[16]={8,0,0,0,0,0,0,1,'F','L','I','P','P','E','R','!'};
    s->ping_id=(uint16_t)(furi_get_tick() ^ s->dhcp_xid);
    icmp[4]=(uint8_t)(s->ping_id>>8); icmp[5]=(uint8_t)s->ping_id;
    uint16_t check=checksum(icmp,sizeof(icmp));
    icmp[2]=(uint8_t)(check>>8); icmp[3]=(uint8_t)check;
    s->ping_started=furi_get_tick();
    if(!sock_send(S_PING,icmp,sizeof(icmp),s->gateway,0)) {
        sock_close(S_PING); s->ping=P_FAIL; return;
    }
    s->ping=P_WAIT; s->ping_deadline=after_ms(PING_TIMEOUT_MS);
}
static void ping_tick(AppState* s) {
    if(s->ping!=P_WAIT) return;
    uint8_t payload[128];
    for(unsigned i=0;i<3;i++) {
        uint8_t from[4]={0};uint16_t n=0;
        int r=sock_receive(S_PING,false,payload,sizeof(payload),&n,from);
        if(r<0) { s->ping=P_FAIL; sock_close(S_PING); return; }
        if(r==0) break;
        if(n>=8 && !memcmp(from,s->gateway,4) && payload[0]==0 && payload[1]==0 &&
           payload[4]==(uint8_t)(s->ping_id>>8) && payload[5]==(uint8_t)s->ping_id &&
           payload[6]==0 && payload[7]==1) {
            uint32_t ticks=furi_get_tick()-s->ping_started;
            s->ping_rtt_ms=(uint32_t)(((uint64_t)ticks*1000U)/furi_kernel_get_tick_frequency());
            s->ping=P_OK; sock_close(S_PING); return;
        }
    }
    if(expired(s->ping_deadline)) { s->ping=P_TIMEOUT; sock_close(S_PING); }
}

static void ip_string(char* b,size_t cap,const uint8_t ip[4]) {
    snprintf(b,cap,"%u.%u.%u.%u",ip[0],ip[1],ip[2],ip[3]);
}
static void app_draw(Canvas* canvas, void* ctx) {
    EthernetApp* app=ctx;
    AppState s;
    furi_mutex_acquire(app->mutex,FuriWaitForever);
    s=app->s;
    furi_mutex_release(app->mutex);
    char line[64], ip[20];
    canvas_clear(canvas);
    canvas_set_font(canvas,FontPrimary);
    canvas_draw_str(canvas,2,11,"Eth Toolkit v0.2");
    canvas_draw_line(canvas,0,14,127,14);
    canvas_set_font(canvas,FontSecondary);
    if(s.page==0) {
        if(!s.found) {
            snprintf(line,sizeof(line),"W5500: absent (0x%02X)",s.version);
            canvas_draw_str(canvas,2,27,line);
            canvas_draw_str(canvas,2,39,"Check module / power");
        } else {
            canvas_draw_str(canvas,2,25,"W5500: found (v04)");
            canvas_draw_str(canvas,2,36,(s.phy&1)?"Cable: CONNECTED":"Cable: DISCONNECTED");
            if(s.phy&1) {
                snprintf(line,sizeof(line),"PHY: %u Mbps, %s",(s.phy&2)?100:10,(s.phy&4)?"Full":"Half");
                canvas_draw_str(canvas,2,47,line);
            }
        }
        canvas_draw_str(canvas,2,57,s.otg_ok?"5V GPIO: ON":"5V GPIO: OFF");
    } else if(s.page==1) {
        snprintf(line,sizeof(line),"MAC %02X:%02X:%02X:%02X:%02X:%02X",
                 s.mac[0],s.mac[1],s.mac[2],s.mac[3],s.mac[4],s.mac[5]);
        canvas_draw_str(canvas,2,23,line);
        ip_string(ip,sizeof(ip),s.ip);snprintf(line,sizeof(line),"IP  %s",ip);canvas_draw_str(canvas,2,33,line);
        ip_string(ip,sizeof(ip),s.mask);snprintf(line,sizeof(line),"SN  %s",ip);canvas_draw_str(canvas,2,43,line);
        ip_string(ip,sizeof(ip),s.gateway);snprintf(line,sizeof(line),"GW  %s",ip);canvas_draw_str(canvas,2,53,line);
    } else if(s.page==2) {
        const char* status="Idle";
        if(s.dhcp==D_OFFER) status="DISCOVER...";
        if(s.dhcp==D_ACK) status="REQUEST...";
        if(s.dhcp==D_LEASE) status="LEASED";
        if(s.dhcp==D_FAIL) status="FAILED";
        snprintf(line,sizeof(line),"DHCP: %s",status);canvas_draw_str(canvas,2,26,line);
        ip_string(ip,sizeof(ip),s.dns);snprintf(line,sizeof(line),"DNS: %s",ip);canvas_draw_str(canvas,2,38,line);
        canvas_draw_str(canvas,2,51,"OK = renew DHCP");
    } else {
        canvas_draw_str(canvas,2,23,"Target: gateway IP");
        ip_string(ip,sizeof(ip),s.gateway);canvas_draw_str(canvas,2,34,ip);
        const char* status="OK = ping gateway";
        if(s.ping==P_WAIT) status="PING: waiting...";
        if(s.ping==P_OK) {snprintf(line,sizeof(line),"Reply: %lu ms",(unsigned long)s.ping_rtt_ms);status=line;}
        if(s.ping==P_TIMEOUT) status="PING: timeout";
        if(s.ping==P_FAIL) status="No DHCP / send error";
        canvas_draw_str(canvas,2,49,status);
    }
    snprintf(line,sizeof(line),"</> page %u/%u  OK action",(unsigned)(s.page+1),PAGES);
    canvas_draw_str(canvas,2,63,line);
}
static void on_input(InputEvent* input,void* ctx) {
    FuriMessageQueue* q=ctx;
    (void)furi_message_queue_put(q,input,0);
}
int32_t flipper_eth_toolkit_app(void* p) {
    UNUSED(p);
    EthernetApp app={0};
    app.mutex=furi_mutex_alloc(FuriMutexTypeNormal);
    furi_check(app.mutex);
    FuriMessageQueue* queue=furi_message_queue_alloc(8,sizeof(InputEvent));
    furi_check(queue);
    bool previous_5v=furi_hal_power_is_otg_enabled();
    bool enabled=previous_5v;
    for(uint8_t attempt=0;attempt<3 && !enabled;attempt++) {
        enabled=furi_hal_power_enable_otg();
        furi_delay_ms(100);
    }
    app.s.otg_ok=enabled;
    furi_hal_spi_bus_handle_init(&furi_hal_spi_bus_handle_external);
    furi_delay_ms(150);
    generate_mac(app.s.mac);
    update_physical(&app.s);
    if(app.s.found && (app.s.phy&1)) dhcp_start(&app.s);

    ViewPort* vp=view_port_alloc();
    view_port_draw_callback_set(vp,app_draw,&app);
    view_port_input_callback_set(vp,on_input,queue);
    Gui* gui=furi_record_open(RECORD_GUI);
    gui_add_view_port(gui,vp,GuiLayerFullscreen);

    bool running=true;
    uint32_t refresh_at=after_ms(1100);
    while(running) {
        InputEvent event;
        FuriStatus got=furi_message_queue_get(queue,&event,furi_ms_to_ticks(70));
        furi_mutex_acquire(app.mutex,FuriWaitForever);
        if(got==FuriStatusOk && event.type==InputTypeShort) {
            if(event.key==InputKeyBack) running=false;
            else if(event.key==InputKeyRight || event.key==InputKeyDown) app.s.page=(app.s.page+1)%PAGES;
            else if(event.key==InputKeyLeft || event.key==InputKeyUp) app.s.page=(app.s.page+PAGES-1)%PAGES;
            else if(event.key==InputKeyOk) {
                if(app.s.page==2) dhcp_start(&app.s);
                else if(app.s.page==3) ping_start(&app.s);
                else update_physical(&app.s);
            }
        }
        if(running) {
            if(expired(refresh_at)) {update_physical(&app.s);refresh_at=after_ms(1100);}
            if(!app.s.found || !(app.s.phy&1)) {
                if(app.s.dhcp==D_OFFER || app.s.dhcp==D_ACK) dhcp_fail(&app.s);
                if(app.s.ping==P_WAIT) { sock_close(S_PING); app.s.ping=P_FAIL; }
            } else {
                if(app.s.dhcp==D_IDLE) dhcp_start(&app.s);
                dhcp_tick(&app.s);
                ping_tick(&app.s);
            }
        }
        furi_mutex_release(app.mutex);
        if(running) view_port_update(vp);
    }
    gui_remove_view_port(gui,vp);
    view_port_free(vp);
    furi_record_close(RECORD_GUI);
    sock_close(S_DHCP); sock_close(S_PING);
    furi_hal_spi_bus_handle_deinit(&furi_hal_spi_bus_handle_external);
    if(!previous_5v && enabled) furi_hal_power_disable_otg();
    furi_message_queue_free(queue);
    furi_mutex_free(app.mutex);
    return 0;
}
