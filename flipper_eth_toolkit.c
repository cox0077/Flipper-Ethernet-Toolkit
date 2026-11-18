/*
 * Flipper Ethernet Toolkit v0.5 (experimental)
 * W5500 5V plug-in module on external SPI; Momentum mntm-012.
 * DHCP/UDP socket 0, gateway ICMP echo/IPRAW socket 1.
 * ARP/MACRAW socket 0 (while DHCP is idle), TCP scan socket 2.
 * DNS UDP socket 3, Wake-on-LAN UDP socket 4, discovery UDP socket 5.
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
#include <storage/storage.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define SPI_TIMEOUT 100U
#define SPI_CHUNK 128U
#define PAGES 13U
#define DISC_LIMIT 12U
#define DISC_TIMEOUT_MS 3800U
#define S_DISC 5U
#define ARP_LIMIT 48U
#define ARP_SPACING_MS 130U
#define TCP_TIMEOUT_MS 1200U
#define TCP_PORT_COUNT 10U
#define REPORT_DIR "/ext/apps_data/flipper_eth_toolkit"
#define REPORT_FILE REPORT_DIR "/report.txt"
#define CSV_FILE REPORT_DIR "/report.csv"
#define DNS_TIMEOUT_MS 2200U
#define DNS_NAME_COUNT 3U
#define DHCP_TIMEOUT_MS 4500U
#define PING_TIMEOUT_MS 1800U
#define S_DHCP 0U
#define S_PING 1U
#define S_TCP 2U
#define S_ARP 0U
#define S_DNS 3U
#define S_WOL 4U

/* W5500 common registers */
#define REG_GAR 0x0001U
#define REG_SUBR 0x0005U
#define REG_SHAR 0x0009U
#define REG_SIPR 0x000FU
#define REG_PHY 0x002EU
#define REG_VERSION 0x0039U
#define REG_RTR 0x0019U
#define REG_RCR 0x001BU
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
#define SN_RXBUF_SIZE 0x001EU
#define CR_OPEN 0x01U
#define CR_CONNECT 0x04U
#define CR_CLOSE 0x10U
#define CR_SEND 0x20U
#define CR_RECV 0x40U
#define MODE_TCP 0x01U
#define MODE_UDP 0x02U
#define MODE_MACRAW 0x04U
#define MODE_IPRAW 0x03U
#define SOCK_INIT 0x13U
#define SOCK_ESTABLISHED 0x17U
#define SOCK_MACRAW 0x42U
#define SOCK_UDP 0x22U
#define SOCK_IPRAW 0x32U
#define DISC_SSDP 0U
#define DISC_MDNS 1U

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

/* The scan is local and deliberately bounded to the device's current /24. */
typedef struct {
    uint8_t ip[4];
    uint8_t mac[6];
} ArpHost;

typedef enum {SCAN_IDLE, SCAN_ACTIVE, SCAN_DRAIN, SCAN_DONE, SCAN_ERROR} ScanStage;
typedef enum {PORT_NOT_TESTED, PORT_OPEN, PORT_CLOSED, PORT_TIMEOUT, PORT_ERROR} PortResult;
static const uint16_t tcp_ports[TCP_PORT_COUNT] = {21,22,23,53,80,139,443,445,3389,8080};

typedef enum {D_IDLE, D_OFFER, D_ACK, D_LEASE, D_FAIL} DHCPStage;
typedef enum {P_IDLE, P_WAIT, P_OK, P_TIMEOUT, P_FAIL} PingStage;
typedef enum {DNS_IDLE, DNS_WAIT, DNS_OK, DNS_TIMEOUT, DNS_FAIL, DNS_NXDOMAIN} DnsStage;
typedef enum {DISC_IDLE, DISC_WAIT, DISC_DONE, DISC_FAIL} DiscStage;
typedef struct {
    uint8_t ip[4];
    char description[40];
    uint8_t protocol;
} DiscoveredService;
static const char* dns_names[DNS_NAME_COUNT]={"example.com","github.com","openai.com"};
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
    /* ARP scan: socket 0 is temporarily MACRAW, one host per tick. */
    ScanStage arp_stage;
    uint16_t arp_probe; /* next host in 1..254 */
    uint16_t arp_sent;
    uint32_t arp_next_at, arp_drain_until;
    uint8_t arp_count, arp_view;
    ArpHost hosts[ARP_LIMIT];
    /* TCP connect probe, one of ten fixed common ports per operation. */
    ScanStage tcp_stage;
    uint8_t tcp_index, tcp_view, tcp_target_index;
    uint8_t tcp_target_ip[4];
    uint8_t tcp_standard_ip[4], tcp_custom_ip[4];
    uint32_t tcp_deadline;
    uint16_t prior_rtr;
    uint8_t prior_rcr;
    bool timeout_cfg_saved;
    PortResult tcp_results[TCP_PORT_COUNT];
    bool sd_ok, sd_attempted;
    bool csv_ok, csv_attempted;
    /* DNS: one IPv4 A lookup at a time; domain chosen from presets. */
    uint8_t dns_choice;
    DnsStage dns_stage;
    uint16_t dns_txid;
    uint32_t dns_deadline;
    uint8_t dns_answer[4];
    /* WOL: last-seen ARP MAC; can wake an offline device after earlier scan. */
    uint8_t wol_target_index;
    bool wol_attempted, wol_ok;
    /* Custom TCP: one selected port, otherwise the standard list. */
    uint16_t custom_port;
    uint8_t custom_digit;
    PortResult custom_result;
    bool tcp_custom;
    /* Service discovery: bounded active multicast queries, unicast replies. */
    uint8_t disc_choice, disc_count, disc_view;
    DiscStage disc_stage;
    uint32_t disc_deadline;
    DiscoveredService services[DISC_LIMIT];

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
    sock_close(S_DNS);
    s->dns_stage=DNS_IDLE;
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

/* DNS query: one question (A/IN), response validated by ID, QR and source. */
static bool network_busy(const AppState* s) {
    return s->dhcp==D_OFFER || s->dhcp==D_ACK ||
           s->arp_stage==SCAN_ACTIVE || s->arp_stage==SCAN_DRAIN ||
           s->tcp_stage==SCAN_ACTIVE || s->ping==P_WAIT || s->dns_stage==DNS_WAIT ||
           s->disc_stage==DISC_WAIT;
}
static void dns_start(AppState* s) {
    if(network_busy(s)) return;
    s->dns_stage=DNS_IDLE;
    memset(s->dns_answer,0,4);
    if(s->dhcp!=D_LEASE || zero_ip(s->dns) || !(s->phy&1)) {
        s->dns_stage=DNS_FAIL;return;
    }
    if(!sock_open(S_DNS,MODE_UDP,53053,0)) {
        s->dns_stage=DNS_FAIL;return;
    }
    uint8_t packet[128]={0};
    s->dns_txid=(uint16_t)(furi_get_tick() ^ s->dhcp_xid ^ 0xD531U);
    packet[0]=(uint8_t)(s->dns_txid>>8);
    packet[1]=(uint8_t)s->dns_txid;
    packet[2]=1; /* RD (recursion desired) */
    packet[5]=1; /* QDCOUNT = 1 */
    const char* name=dns_names[s->dns_choice];
    size_t pos=12;
    while(*name) {
        const char* dot=strchr(name,'.');
        size_t len=dot?(size_t)(dot-name):strlen(name);
        if(!len || len>63 || pos+len+6>sizeof(packet)) {
            s->dns_stage=DNS_FAIL;sock_close(S_DNS);return;
        }
        packet[pos++]=(uint8_t)len;
        memcpy(packet+pos,name,len);pos+=len;
        name+=len;
        if(*name=='.') name++;
    }
    packet[pos++]=0;
    packet[pos++]=0;packet[pos++]=1; /* TYPE A */
    packet[pos++]=0;packet[pos++]=1; /* CLASS IN */
    if(!sock_send(S_DNS,packet,(uint16_t)pos,s->dns,53)) {
        sock_close(S_DNS);s->dns_stage=DNS_FAIL;return;
    }
    s->dns_stage=DNS_WAIT;
    s->dns_deadline=after_ms(DNS_TIMEOUT_MS);
}
/* Skip a compressed or plain DNS name without following pointers. */
static bool dns_skip_name(const uint8_t* b, size_t n, size_t* off) {
    size_t p=*off;
    for(size_t steps=0;steps<128 && p<n;steps++) {
        uint8_t len=b[p++];
        if(len==0) {*off=p;return true;}
        if((len&0xC0U)==0xC0U) {
            if(p>=n) return false;
            *off=p+1;return true;
        }
        if((len&0xC0U)!=0 || p+len>n) return false;
        p+=len;
    }
    return false;
}
static void dns_tick(AppState* s) {
    if(s->dns_stage!=DNS_WAIT) return;
    uint8_t payload[512];
    for(unsigned i=0;i<3;i++) {
        uint16_t n=0; uint8_t src[4]={0};
        int got=sock_receive(S_DNS,true,payload,sizeof(payload),&n,src);
        if(got<0) {s->dns_stage=DNS_FAIL; sock_close(S_DNS);return;}
        if(got==0) break;
        if(n<12 || memcmp(src,s->dns,4)!=0) continue;
        if(payload[0]!=(uint8_t)(s->dns_txid>>8) || payload[1]!=(uint8_t)s->dns_txid) continue;
        if(!(payload[2]&0x80U) || (payload[2]&0x78U)) continue;
        if(payload[3]&0x0FU) {
            s->dns_stage=(payload[3]&0x0FU)==3?DNS_NXDOMAIN:DNS_FAIL;
            sock_close(S_DNS);return;
        }
        uint16_t questions=(uint16_t)(payload[4]<<8|payload[5]);
        uint16_t answers=(uint16_t)(payload[6]<<8|payload[7]);
        if(questions!=1) {s->dns_stage=DNS_FAIL;sock_close(S_DNS);return;}
        size_t pos=12;
        if(!dns_skip_name(payload,n,&pos) || pos+4>n) {
            s->dns_stage=DNS_FAIL;sock_close(S_DNS);return;
        }
        pos+=4;
        bool found=false;
        for(uint16_t a=0;a<answers && a<32;a++) {
            if(!dns_skip_name(payload,n,&pos) || pos+10>n) break;
            uint16_t type=(uint16_t)(payload[pos]<<8|payload[pos+1]);
            uint16_t klass=(uint16_t)(payload[pos+2]<<8|payload[pos+3]);
            uint16_t data_len=(uint16_t)(payload[pos+8]<<8|payload[pos+9]);
            pos+=10;
            if(pos+data_len>n) break;
            if(type==1 && klass==1 && data_len==4) {
                memcpy(s->dns_answer,payload+pos,4);
                found=true;break;
            }
            pos+=data_len;
        }
        s->dns_stage=found?DNS_OK:DNS_FAIL;
        sock_close(S_DNS);return;
    }
    if(expired(s->dns_deadline)) {
        s->dns_stage=DNS_TIMEOUT;
        sock_close(S_DNS);
    }
}
/* UDP magic packet for Wake-on-LAN: broadcast, 6 x FF + 16 copies of MAC. */
static void wol_send(AppState* s) {
    if(network_busy(s)) return;
    s->wol_attempted=true;
    s->wol_ok=false;
    if(s->dhcp!=D_LEASE || !(s->phy&1) || s->arp_count==0 ||
       s->wol_target_index>=s->arp_count) return;
    uint8_t packet[102];
    memset(packet,0xFF,6);
    for(uint8_t i=0;i<16;i++)
        memcpy(packet+6+6*i,s->hosts[s->wol_target_index].mac,6);
    if(!sock_open(S_WOL,MODE_UDP,49152,0)) return;
    const uint8_t broadcast[4]={255,255,255,255};
    s->wol_ok=sock_send(S_WOL,packet,sizeof(packet),broadcast,9);
    sock_close(S_WOL);
}

/* Conservative offline vendor hints: only a handful of well-known OUIs.
 * Locally administered addresses cannot be mapped to a reliable manufacturer. */
static const char* mac_vendor(const uint8_t mac[6]) {
    if(mac[0] & 1U) return "Multicast";
    if(mac[0] & 2U) return "Local/Random";
    static const struct {uint8_t b[3];const char* vendor;} known[] = {
        {{0xB8,0x27,0xEB},"Raspberry Pi"},
        {{0xDC,0xA6,0x32},"Raspberry Pi"},
        {{0x24,0x0A,0xC4},"Espressif"},
        {{0x18,0xFE,0x34},"Espressif"},
        {{0x00,0x1B,0x63},"Apple"},
        {{0x00,0x1B,0x21},"Intel"},
        {{0x4C,0x5E,0x0C},"MikroTik"},
    };
    for(size_t i=0; i<sizeof(known)/sizeof(known[0]); i++) {
        if(memcmp(mac,known[i].b,3)==0) return known[i].vendor;
    }
    return "Unknown";
}

/* Output printable, single-line ASCII fields. No untrusted text is executed. */
static void sanitize_label(char* out, size_t cap, const uint8_t* in, size_t n) {
    if(!cap) return;
    size_t used=0;
    while(used+1<cap && n--) {
        uint8_t c=*in++;
        if(c==',' || c=='\r' || c=='\n' || c=='"') c=' ';
        out[used++]=(c>=32 && c<=126)?(char)c:'?';
    }
    out[used]=0;
}

/* Decode a DNS name with strict pointer and packet bounds. */
static bool dns_name_text(const uint8_t* packet, size_t n, size_t offset,
                          char* out, size_t cap) {
    if(!cap || offset>=n) return false;
    size_t outlen=0;
    size_t p=offset;
    for(unsigned steps=0; steps<32; steps++) {
        if(p>=n) return false;
        uint8_t len=packet[p++];
        if(len==0) {out[outlen]=0; return true;}
        if((len&0xC0U)==0xC0U) {
            if(p>=n) return false;
            size_t target=(size_t)(((len&0x3FU)<<8)|packet[p++]);
            if(target>=n) return false;
            p=target;
            continue;
        }
        if(len>63 || (len&0xC0U) || p+len>n) return false;
        if(outlen && outlen+1<cap) out[outlen++]='.';
        for(uint8_t i=0;i<len;i++) {
            uint8_t c=packet[p+i];
            if(outlen+1<cap) out[outlen++]=(c>=33 && c<=126 && c!=',' && c!='"')?(char)c:'_';
        }
        p+=len;
    }
    return false;
}

static void discovery_add(AppState* s, const uint8_t ip[4], const char* label) {
    if(zero_ip(ip) || !label[0]) return;
    for(uint8_t i=0;i<s->disc_count;i++) {
        if(s->services[i].protocol==s->disc_choice &&
           !memcmp(s->services[i].ip,ip,4) &&
           !strncmp(s->services[i].description,label,sizeof(s->services[i].description))) return;
    }
    if(s->disc_count>=DISC_LIMIT) return;
    DiscoveredService* h=&s->services[s->disc_count++];
    memcpy(h->ip,ip,4);
    h->protocol=s->disc_choice;
    snprintf(h->description,sizeof(h->description),"%s",label);
}

static void discovery_ssdp(AppState* s, const uint8_t* b, uint16_t n, const uint8_t ip[4]) {
    if(n<12 || memcmp(b,"HTTP/1.1 200",12)!=0) return;
    /* Prefer the ST: header; bounded to 39 bytes and a single line. */
    char name[40]="SSDP response";
    size_t pos=0;
    while(pos<n) {
        size_t start=pos;
        while(pos<n && b[pos]!='\n' && b[pos]!='\r') pos++;
        size_t end=pos;
        while(pos<n && (b[pos]=='\n' || b[pos]=='\r')) pos++;
        if(end>=start+3 && (b[start]=='S'||b[start]=='s') &&
           (b[start+1]=='T'||b[start+1]=='t') && b[start+2]==':') {
            size_t v=start+3;
            while(v<end && b[v]==' ') v++;
            sanitize_label(name,sizeof(name),b+v,end-v);
            break;
        }
    }
    discovery_add(s,ip,name);
}

static void discovery_mdns(AppState* s, const uint8_t* b, uint16_t n, const uint8_t ip[4]) {
    if(n<12 || !(b[2]&0x80U) || (b[2]&0x78U)) return;
    uint16_t q=(uint16_t)((b[4]<<8)|b[5]);
    uint16_t a=(uint16_t)((b[6]<<8)|b[7]);
    size_t pos=12;
    if(q>8 || a>48) return;
    for(uint16_t i=0;i<q;i++) {
        if(!dns_skip_name(b,n,&pos) || pos+4>n) return;
        pos+=4;
    }
    for(uint16_t i=0;i<a;i++) {
        if(!dns_skip_name(b,n,&pos) || pos+10>n) return;
        uint16_t type=(uint16_t)((b[pos]<<8)|b[pos+1]);
        uint16_t rdlen=(uint16_t)((b[pos+8]<<8)|b[pos+9]);
        pos+=10;
        if(pos+rdlen>n) return;
        if(type==12 && rdlen>0) {
            char name[40];
            if(dns_name_text(b,n,pos,name,sizeof(name))) discovery_add(s,ip,name);
        }
        pos+=rdlen;
    }
}

static void discovery_start(AppState* s) {
    if(network_busy(s)) return;
    s->disc_count=0;
    s->disc_view=0;
    s->disc_stage=DISC_FAIL;
    if(s->dhcp!=D_LEASE || !(s->phy&1)) return;
    uint16_t src_port=s->disc_choice==DISC_SSDP?49055U:49056U;
    if(!sock_open(S_DISC,MODE_UDP,src_port,0)) return;
    bool ok=false;
    if(s->disc_choice==DISC_SSDP) {
        static const uint8_t dst[4]={239,255,255,250};
        static const char query[]="M-SEARCH * HTTP/1.1\r\n"
            "HOST: 239.255.255.250:1900\r\n"
            "MAN: \"ssdp:discover\"\r\n"
            "MX: 1\r\nST: ssdp:all\r\n\r\n";
        ok=sock_send(S_DISC,(const uint8_t*)query,(uint16_t)(sizeof(query)-1),dst,1900);
    } else {
        static const uint8_t dst[4]={224,0,0,251};
        static const uint8_t q[]={
            0x32,0x51, 0x00,0x00, 0x00,0x01, 0x00,0x00, 0x00,0x00, 0x00,0x00,
            9,'_','s','e','r','v','i','c','e','s',
            7,'_','d','n','s','-','s','d',4,'_','u','d','p',5,'l','o','c','a','l',0,
            0,12,0,1
        };
        /* Legacy unicast reply requested by using a non-5353 source port. */
        ok=sock_send(S_DISC,q,sizeof(q),dst,5353);
    }
    if(!ok) {sock_close(S_DISC);return;}
    s->disc_stage=DISC_WAIT;
    s->disc_deadline=after_ms(DISC_TIMEOUT_MS);
}

static void discovery_tick(AppState* s) {
    if(s->disc_stage!=DISC_WAIT) return;
    uint8_t b[850],ip[4];
    for(unsigned i=0;i<4;i++) {
        uint16_t n=0;
        int received=sock_receive(S_DISC,true,b,sizeof(b),&n,ip);
        if(received<0) {s->disc_stage=DISC_FAIL;sock_close(S_DISC);return;}
        if(received==0) break;
        if(s->disc_choice==DISC_SSDP) discovery_ssdp(s,b,n,ip);
        else discovery_mdns(s,b,n,ip);
    }
    if(expired(s->disc_deadline)) {
        s->disc_stage=DISC_DONE;
        sock_close(S_DISC);
    }
}

/* Ethernet RAW frames: W5500 prepends a two-byte length, including itself.
 * Only first 42 bytes needed for ARP replies; consume entire RX packet.
 * Returns 1 frame, 0 no frame/bad frame, -1 read error.
 */
static int macraw_receive(uint8_t* frame, uint16_t cap, uint16_t* out_len) {
    uint16_t available=0, ptr=0;
    if(!sn16(S_ARP,SN_RX_RSR,&available)) return -1;
    if(available<2) return 0;
    if(!sn16(S_ARP,SN_RX_RD,&ptr)) return -1;
    uint8_t prefix[2];
    if(!reg_read(rxblock(S_ARP),ptr,prefix,2)) return -1;
    uint16_t len=(uint16_t)((prefix[0]<<8)|prefix[1]);
    if(len<2 || len>2048) {
        /* Unexpected framing: close scanner rather than corrupt RX pointer. */
        return -1;
    }
    if(available<len) return 0;
    uint16_t frame_len=(uint16_t)(len-2);
    if(frame_len>0) {
        uint16_t copy=frame_len<cap?frame_len:cap;
        if(!reg_read(rxblock(S_ARP),(uint16_t)(ptr+2),frame,copy)) return -1;
    }
    if(!sw16(S_ARP,SN_RX_RD,(uint16_t)(ptr+len)) || !command(S_ARP,CR_RECV)) return -1;
    *out_len=frame_len<cap?frame_len:cap;
    return 1;
}
static bool macraw_send(const uint8_t* frame, uint16_t len) {
    uint16_t free_bytes=0, wr_ptr=0;
    if(!sn16(S_ARP,SN_TX_FSR,&free_bytes) || free_bytes<len) return false;
    if(!sn16(S_ARP,SN_TX_WR,&wr_ptr)) return false;
    if(!reg_write(txblock(S_ARP),wr_ptr,frame,len)) return false;
    if(!sw16(S_ARP,SN_TX_WR,(uint16_t)(wr_ptr+len))) return false;
    return command(S_ARP,CR_SEND);
}
static bool arp_request(const AppState* s, uint8_t last_octet) {
    uint8_t eth[60]={0};
    memset(eth,0xFF,6);          /* destination MAC: broadcast */
    memcpy(eth+6,s->mac,6);      /* Ethernet source MAC */
    eth[12]=0x08; eth[13]=0x06;  /* ARP */
    eth[14]=0; eth[15]=1;       /* hardware: Ethernet */
    eth[16]=0x08; eth[17]=0;    /* protocol: IPv4 */
    eth[18]=6; eth[19]=4;       /* MAC and IP lengths */
    eth[20]=0; eth[21]=1;       /* operation: request */
    memcpy(eth+22,s->mac,6);
    memcpy(eth+28,s->ip,4);
    /* eth[32..37] target MAC = 0 */
    memcpy(eth+38,s->ip,3);
    eth[41]=last_octet;
    return macraw_send(eth,sizeof(eth));
}
static void arp_keep_reply(AppState* s, const uint8_t* f, uint16_t n) {
    if(n<42 || f[12]!=0x08 || f[13]!=0x06 ||
       f[14]!=0 || f[15]!=1 || f[16]!=8 || f[17]!=0 ||
       f[18]!=6 || f[19]!=4 || f[20]!=0 || f[21]!=2) return;
    /* Reply for our exact IPv4, on the current /24, from actual sender. */
    if(memcmp(f+38,s->ip,4)!=0 || memcmp(f+28,s->ip,3)!=0 ||
       (f[31]==0 || f[31]==255) || !memcmp(f+28,s->ip,4) ||
       memcmp(f+32,s->mac,6)!=0 || memcmp(f+6,f+22,6)!=0) return;
    if(memcmp(f,s->mac,6)!=0 && memcmp(f,"\xFF\xFF\xFF\xFF\xFF\xFF",6)!=0) return;
    for(uint8_t i=0;i<s->arp_count;i++) {
        if(memcmp(s->hosts[i].ip,f+28,4)==0) {
            memcpy(s->hosts[i].mac,f+22,6);
            return;
        }
    }
    if(s->arp_count>=ARP_LIMIT) return;
    ArpHost* host=&s->hosts[s->arp_count++];
    memcpy(host->ip,f+28,4);
    memcpy(host->mac,f+22,6);
}
static void arp_stop(AppState* s, bool err) {
    sock_close(S_ARP);
    s->arp_stage=err?SCAN_ERROR:SCAN_DONE;
}
static void arp_start(AppState* s) {
    if(s->arp_stage==SCAN_ACTIVE || s->arp_stage==SCAN_DRAIN) return;
    s->arp_count=0; s->arp_view=0; s->arp_sent=0; s->arp_probe=1;
    s->wol_target_index=0; s->wol_attempted=false;
    s->tcp_target_index=0;
    s->tcp_stage=SCAN_IDLE;
    s->tcp_index=0;
    memset(s->tcp_standard_ip,0,4);
    memset(s->tcp_custom_ip,0,4);
    s->custom_result=PORT_NOT_TESTED;
    memset(s->tcp_results,0,sizeof(s->tcp_results));
    if(s->dhcp!=D_LEASE || !s->found || !(s->phy&1) ||
       zero_ip(s->ip) || zero_ip(s->mask)) {
        s->arp_stage=SCAN_ERROR; return;
    }
    sock_close(S_DHCP);
    sock_close(S_PING);
    s->ping=P_IDLE;
    sock_close(S_TCP);
    if(!sw8(S_ARP,SN_MR,MODE_MACRAW) || !command(S_ARP,CR_OPEN)) {
        arp_stop(s,true);return;
    }
    uint8_t status=0;
    if(!sn8(S_ARP,SN_SR,&status) || status!=SOCK_MACRAW) {arp_stop(s,true);return;}
    s->arp_stage=SCAN_ACTIVE;
    s->arp_next_at=furi_get_tick();
}
static void arp_tick(AppState* s) {
    if(s->arp_stage!=SCAN_ACTIVE && s->arp_stage!=SCAN_DRAIN) return;
    for(uint8_t i=0;i<6;i++) {
        uint8_t frame[64];uint16_t size=0;
        int got=macraw_receive(frame,sizeof(frame),&size);
        if(got<0) {arp_stop(s,true);return;}
        if(!got) break;
        arp_keep_reply(s,frame,size);
    }
    if(s->arp_stage==SCAN_DRAIN) {
        if(expired(s->arp_drain_until)) arp_stop(s,false);
        return;
    }
    if(!expired(s->arp_next_at)) return;
    /* One packet every 130 ms; single /24 and max 254 probes. */
    while(s->arp_probe<=254) {
        uint8_t host=(uint8_t)s->arp_probe;
        uint8_t host_bits=(uint8_t)~s->mask[3];
        bool same_subnet=(uint8_t)(host&s->mask[3])==(uint8_t)(s->ip[3]&s->mask[3]);
        bool net_or_bcast=((host&host_bits)==0 || (host&host_bits)==host_bits);
        if(host!=s->ip[3] && same_subnet && !net_or_bcast) break;
        s->arp_probe++;
    }
    if(s->arp_probe>254) {
        s->arp_stage=SCAN_DRAIN;
        s->arp_drain_until=after_ms(800);
        return;
    }
    if(!arp_request(s,(uint8_t)s->arp_probe)) {arp_stop(s,true);return;}
    s->arp_probe++; s->arp_sent++;
    s->arp_next_at=after_ms(ARP_SPACING_MS);
}
static const char* port_text(PortResult r) {
    switch(r) {
    case PORT_OPEN: return "OPEN";
    case PORT_CLOSED: return "CLOSED";
    case PORT_TIMEOUT: return "TIMEOUT";
    case PORT_ERROR: return "ERROR";
    default: return "-";
    }
}
static void tcp_restore_timeout(AppState* s) {
    if(s->timeout_cfg_saved) {
        (void)wr16(0,REG_RTR,s->prior_rtr);
        (void)wr8(0,REG_RCR,s->prior_rcr);
        s->timeout_cfg_saved=false;
    }
}
static void tcp_stop(AppState* s, bool err) {
    sock_close(S_TCP);
    tcp_restore_timeout(s);
    s->tcp_stage=err?SCAN_ERROR:SCAN_DONE;
}
static bool tcp_begin_port(AppState* s) {
    const uint8_t sock=S_TCP;
    sock_close(sock);
    if(!sw8(sock,SN_MR,MODE_TCP) ||
       !sw16(sock,SN_PORT,(uint16_t)(40000U+s->tcp_index)) ||
       !reg_write(sblock(sock),SN_DIPR,s->tcp_target_ip,4) ||
       !sw16(sock,SN_DPORT,s->tcp_custom?s->custom_port:tcp_ports[s->tcp_index]) ||
       !command(sock,CR_OPEN)) return false;
    uint8_t status=0;
    if(!sn8(sock,SN_SR,&status) || status!=SOCK_INIT) return false;
    if(!command(sock,CR_CONNECT)) return false;
    s->tcp_deadline=after_ms(TCP_TIMEOUT_MS);
    return true;
}
static void tcp_start(AppState* s,bool custom) {
    if(s->tcp_stage==SCAN_ACTIVE || s->arp_stage==SCAN_ACTIVE ||
       s->arp_stage==SCAN_DRAIN) return;
    s->tcp_custom=custom;
    if(custom) s->custom_result=PORT_NOT_TESTED;
    else memset(s->tcp_results,0,sizeof(s->tcp_results));
    s->tcp_index=0; s->tcp_view=0;
    if(s->dhcp!=D_LEASE || !s->found || !(s->phy&1)) {
        s->tcp_stage=SCAN_ERROR;return;
    }
    if(s->tcp_target_index==0) memcpy(s->tcp_target_ip,s->gateway,4);
    else if((unsigned)s->tcp_target_index<=s->arp_count)
        memcpy(s->tcp_target_ip,s->hosts[s->tcp_target_index-1].ip,4);
    else {s->tcp_stage=SCAN_ERROR;return;}
    if(zero_ip(s->tcp_target_ip)) {s->tcp_stage=SCAN_ERROR;return;}
    if(custom) memcpy(s->tcp_custom_ip,s->tcp_target_ip,4);
    else memcpy(s->tcp_standard_ip,s->tcp_target_ip,4);
    if(!rd16(0,REG_RTR,&s->prior_rtr) || !rd8(0,REG_RCR,&s->prior_rcr)) {
        s->tcp_stage=SCAN_ERROR;return;
    }
    s->timeout_cfg_saved=true;
    /* Short retries for a bounded, low-rate diagnostic port check. */
    if(!wr16(0,REG_RTR,1000) || !wr8(0,REG_RCR,2)) {
        tcp_stop(s,true);return;
    }
    s->tcp_stage=SCAN_ACTIVE;
    if(!tcp_begin_port(s)) {if(custom) s->custom_result=PORT_ERROR;
        else s->tcp_results[0]=PORT_ERROR;
        tcp_stop(s,true);}
}
static void tcp_tick(AppState* s) {
    if(s->tcp_stage!=SCAN_ACTIVE) return;
    uint8_t status=0,ir=0;
    if(!sn8(S_TCP,SN_SR,&status) || !sn8(S_TCP,SN_IR,&ir)) {
        if(s->tcp_custom) s->custom_result=PORT_ERROR;
        else s->tcp_results[s->tcp_index]=PORT_ERROR;
        tcp_stop(s,true);return;
    }
    PortResult outcome=PORT_NOT_TESTED;
    if(status==SOCK_ESTABLISHED || (ir&0x01U)) outcome=PORT_OPEN;
    else if(ir&0x08U) outcome=PORT_TIMEOUT;
    else if(status==0) outcome=PORT_CLOSED;
    else if(expired(s->tcp_deadline)) outcome=PORT_TIMEOUT;
    if(outcome==PORT_NOT_TESTED) return;
    if(s->tcp_custom) s->custom_result=outcome;
    else s->tcp_results[s->tcp_index]=outcome;
    sock_close(S_TCP);
    s->tcp_index++;
    if(s->tcp_index>=(s->tcp_custom?1U:TCP_PORT_COUNT)) {tcp_stop(s,false);return;}
    if(!tcp_begin_port(s)) {
        s->tcp_results[s->tcp_index]=PORT_ERROR;
        tcp_stop(s,true);
    }
}
static bool file_text(File* f, const char* b) {
    size_t len=strlen(b);
    return storage_file_write(f,b,len)==len;
}
static bool save_report(AppState* s) {
    Storage* storage=furi_record_open(RECORD_STORAGE);
    if(!storage) return false;
    bool ok=storage_simply_mkdir(storage,"/ext/apps_data") &&
            storage_simply_mkdir(storage,REPORT_DIR);
    File* f=storage_file_alloc(storage);
    if(!f) {furi_record_close(RECORD_STORAGE);return false;}
    if(ok) ok=storage_file_open(f,REPORT_FILE,FSAM_WRITE,FSOM_OPEN_APPEND);
    char row[144];
    if(ok) {
        ok=file_text(f,"\n=== Flipper Ethernet Toolkit v0.5 ===\n");
        snprintf(row,sizeof(row),"MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                 s->mac[0],s->mac[1],s->mac[2],s->mac[3],s->mac[4],s->mac[5]);
        ok=ok&&file_text(f,row);
        snprintf(row,sizeof(row),"IP: %u.%u.%u.%u / mask %u.%u.%u.%u\n",
                 s->ip[0],s->ip[1],s->ip[2],s->ip[3],
                 s->mask[0],s->mask[1],s->mask[2],s->mask[3]);
        ok=ok&&file_text(f,row);
        snprintf(row,sizeof(row),"Gateway: %u.%u.%u.%u, DNS: %u.%u.%u.%u\n",
                 s->gateway[0],s->gateway[1],s->gateway[2],s->gateway[3],
                 s->dns[0],s->dns[1],s->dns[2],s->dns[3]);
        ok=ok&&file_text(f,row);
        snprintf(row,sizeof(row),"ARP: %u probes; %u replies stored (max %u)\n",
                 s->arp_sent,s->arp_count,ARP_LIMIT);
        ok=ok&&file_text(f,row);
        for(uint8_t i=0;i<s->arp_count && ok;i++) {
            const ArpHost* h=&s->hosts[i];
            snprintf(row,sizeof(row),"ARP %u.%u.%u.%u %02X:%02X:%02X:%02X:%02X:%02X (%s)\n",
                     h->ip[0],h->ip[1],h->ip[2],h->ip[3],h->mac[0],h->mac[1],
                     h->mac[2],h->mac[3],h->mac[4],h->mac[5],mac_vendor(h->mac));
            ok=file_text(f,row);
        }
        snprintf(row,sizeof(row),"TCP target: %u.%u.%u.%u\n",
                 s->tcp_standard_ip[0],s->tcp_standard_ip[1],
                 s->tcp_standard_ip[2],s->tcp_standard_ip[3]);
        ok=ok&&file_text(f,row);
        for(uint8_t i=0;i<TCP_PORT_COUNT && ok;i++) {
            snprintf(row,sizeof(row),"TCP %u %s\n",tcp_ports[i],port_text(s->tcp_results[i]));
            ok=file_text(f,row);
        }
        snprintf(row,sizeof(row),"DNS %s: %u.%u.%u.%u (state %u)\n",
                 dns_names[s->dns_choice],s->dns_answer[0],s->dns_answer[1],
                 s->dns_answer[2],s->dns_answer[3],(unsigned)s->dns_stage);
        ok=ok&&file_text(f,row);
        snprintf(row,sizeof(row),"Custom TCP %u.%u.%u.%u:%u: %s\n",
                 s->tcp_custom_ip[0],s->tcp_custom_ip[1],s->tcp_custom_ip[2],
                 s->tcp_custom_ip[3],s->custom_port,port_text(s->custom_result));
        ok=ok&&file_text(f,row);
        for(uint8_t i=0;i<s->disc_count && ok;i++) {
            const DiscoveredService* h=&s->services[i];
            snprintf(row,sizeof(row),"SERVICE %s %u.%u.%u.%u %s\n",
                     h->protocol==DISC_SSDP?"SSDP":"mDNS",
                     h->ip[0],h->ip[1],h->ip[2],h->ip[3],h->description);
            ok=file_text(f,row);
        }
        ok=ok&&file_text(f,"=== END ===\n");
        if(ok) ok=storage_file_sync(f);
    }
    (void)storage_file_close(f);
    storage_file_free(f);
    furi_record_close(RECORD_STORAGE);
    s->sd_attempted=true;
    s->sd_ok=ok;
    return ok;
}

/* CSV is overwritten per export: a single, spreadsheet-friendly network snapshot. */
static bool save_csv(AppState* s) {
    Storage* storage=furi_record_open(RECORD_STORAGE);
    if(!storage) return false;
    bool ok=storage_simply_mkdir(storage,"/ext/apps_data") &&
            storage_simply_mkdir(storage,REPORT_DIR);
    File* f=storage_file_alloc(storage);
    if(!f) {furi_record_close(RECORD_STORAGE);return false;}
    if(ok) ok=storage_file_open(f,CSV_FILE,FSAM_WRITE,FSOM_CREATE_ALWAYS);
    char row[160];
    if(ok) {
        ok=file_text(f,"type,ip,mac,port,status,details\n");
        snprintf(row,sizeof(row),"DEVICE,%u.%u.%u.%u,%02X:%02X:%02X:%02X:%02X:%02X,,DHCP,%u.%u.%u.%u\n",
                 s->ip[0],s->ip[1],s->ip[2],s->ip[3],
                 s->mac[0],s->mac[1],s->mac[2],s->mac[3],s->mac[4],s->mac[5],
                 s->gateway[0],s->gateway[1],s->gateway[2],s->gateway[3]);
        ok=ok&&file_text(f,row);
        for(uint8_t i=0;i<s->arp_count && ok;i++) {
            const ArpHost* h=&s->hosts[i];
            snprintf(row,sizeof(row),"ARP,%u.%u.%u.%u,%02X:%02X:%02X:%02X:%02X:%02X,,FOUND,%s\n",
                     h->ip[0],h->ip[1],h->ip[2],h->ip[3],
                     h->mac[0],h->mac[1],h->mac[2],h->mac[3],h->mac[4],h->mac[5],mac_vendor(h->mac));
            ok=file_text(f,row);
        }
        for(uint8_t i=0;i<TCP_PORT_COUNT && ok;i++) {
            snprintf(row,sizeof(row),"TCP,%u.%u.%u.%u,,%u,%s,standard\n",
                     s->tcp_standard_ip[0],s->tcp_standard_ip[1],
                     s->tcp_standard_ip[2],s->tcp_standard_ip[3],
                     tcp_ports[i],port_text(s->tcp_results[i]));
            ok=file_text(f,row);
        }
        snprintf(row,sizeof(row),"TCP,%u.%u.%u.%u,,%u,%s,custom\n",
                 s->tcp_custom_ip[0],s->tcp_custom_ip[1],
                 s->tcp_custom_ip[2],s->tcp_custom_ip[3],
                 s->custom_port,port_text(s->custom_result));
        ok=ok&&file_text(f,row);
        snprintf(row,sizeof(row),"DNS,%u.%u.%u.%u,,,result,%s\n",
                 s->dns_answer[0],s->dns_answer[1],s->dns_answer[2],s->dns_answer[3],
                 dns_names[s->dns_choice]);
        ok=ok&&file_text(f,row);
        for(uint8_t i=0;i<s->disc_count && ok;i++) {
            const DiscoveredService* h=&s->services[i];
            snprintf(row,sizeof(row),"SERVICE,%u.%u.%u.%u,,,%.5s,%s\n",
                     h->ip[0],h->ip[1],h->ip[2],h->ip[3],
                     h->protocol==DISC_SSDP?"SSDP":"mDNS",h->description);
            ok=file_text(f,row);
        }
        if(ok) ok=storage_file_sync(f);
    }
    (void)storage_file_close(f);
    storage_file_free(f);
    furi_record_close(RECORD_STORAGE);
    s->csv_attempted=true;
    s->csv_ok=ok;
    return ok;
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
    canvas_draw_str(canvas,2,11,"Eth Toolkit v0.5");
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
    } else if(s.page==3) {
        canvas_draw_str(canvas,2,23,"Target: gateway IP");
        ip_string(ip,sizeof(ip),s.gateway);canvas_draw_str(canvas,2,34,ip);
        const char* status="OK = ping gateway";
        if(s.ping==P_WAIT) status="PING: waiting...";
        if(s.ping==P_OK) {snprintf(line,sizeof(line),"Reply: %lu ms",(unsigned long)s.ping_rtt_ms);status=line;}
        if(s.ping==P_TIMEOUT) status="PING: timeout";
        if(s.ping==P_FAIL) status="No DHCP / send error";
        canvas_draw_str(canvas,2,49,status);
    } else if(s.page==4) {
        const char* st="OK = ARP scan /24";
        if(s.arp_stage==SCAN_ACTIVE) st="Scanning local /24...";
        if(s.arp_stage==SCAN_DRAIN) st="Waiting for replies";
        if(s.arp_stage==SCAN_DONE) st="Scan complete";
        if(s.arp_stage==SCAN_ERROR) st="ARP failed / no DHCP";
        canvas_draw_str(canvas,2,23,st);
        snprintf(line,sizeof(line),"Probe %u/254 found %u",s.arp_sent,s.arp_count);
        canvas_draw_str(canvas,2,33,line);
        for(uint8_t i=0;i<2;i++) {
            uint8_t index=(uint8_t)(s.arp_view+i);
            if(index<s.arp_count) {
                const ArpHost* h=&s.hosts[index];
                snprintf(line,sizeof(line),"%u) %u.%u.%u.%u",(unsigned)(index+1),
                         h->ip[0],h->ip[1],h->ip[2],h->ip[3]);
                canvas_draw_str(canvas,2,(uint8_t)(43+i*10),line);
            }
        }
    } else if(s.page==5) {
        const char* st="OK = scan TCP";
        if(s.tcp_stage==SCAN_ACTIVE && !s.tcp_custom) st="TCP: scanning...";
        if(s.tcp_stage==SCAN_DONE && !s.tcp_custom) st="TCP: complete";
        if(s.tcp_stage==SCAN_ERROR && !s.tcp_custom) st="TCP: error / DHCP";
        canvas_draw_str(canvas,2,23,st);
        const uint8_t* target=s.tcp_target_index==0?s.gateway:
              (s.tcp_target_index<=s.arp_count?s.hosts[s.tcp_target_index-1].ip:s.gateway);
        if(!zero_ip(s.tcp_standard_ip)) target=s.tcp_standard_ip;
        ip_string(ip,sizeof(ip),target);
        snprintf(line,sizeof(line),"Target: %s",ip);
        canvas_draw_str(canvas,2,33,line);
        uint8_t open=0;
        for(uint8_t i=0;i<TCP_PORT_COUNT;i++) if(s.tcp_results[i]==PORT_OPEN) open++;
        uint8_t tested=0;
        for(uint8_t i=0;i<TCP_PORT_COUNT;i++)
            if(s.tcp_results[i]!=PORT_NOT_TESTED) tested++;
        snprintf(line,sizeof(line),"%u/%u  OPEN: %u",tested,TCP_PORT_COUNT,open);
        canvas_draw_str(canvas,2,43,line);
        snprintf(line,sizeof(line),"%u: %s",tcp_ports[s.tcp_view],port_text(s.tcp_results[s.tcp_view]));
        canvas_draw_str(canvas,2,53,line);
    } else if(s.page==6) {
        canvas_draw_str(canvas,2,24,"microSD report.txt");
        canvas_draw_str(canvas,2,36,"OK = append report");
        const char* st="Ready";
        if(s.sd_attempted) st=s.sd_ok?"Saved successfully":"Save failed (SD?)";
        canvas_draw_str(canvas,2,50,st);
    } else if(s.page==7) {
        canvas_draw_str(canvas,2,23,"DNS Lookup (UP/DOWN)");
        canvas_draw_str(canvas,2,34,dns_names[s.dns_choice]);
        const char* st="OK = query A record";
        if(s.dns_stage==DNS_WAIT) st="DNS: resolving...";
        if(s.dns_stage==DNS_FAIL) st="DNS: failed";
        if(s.dns_stage==DNS_TIMEOUT) st="DNS: timeout";
        if(s.dns_stage==DNS_NXDOMAIN) st="DNS: NXDOMAIN";
        if(s.dns_stage==DNS_OK) {
            ip_string(ip,sizeof(ip),s.dns_answer);
            snprintf(line,sizeof(line),"A: %s",ip);st=line;
        }
        canvas_draw_str(canvas,2,47,st);
    } else if(s.page==8) {
        canvas_draw_str(canvas,2,23,"Wake-on-LAN");
        if(s.arp_count==0) {
            canvas_draw_str(canvas,2,36,"Run ARP scan first");
        } else {
            const ArpHost* h=&s.hosts[s.wol_target_index];
            ip_string(ip,sizeof(ip),h->ip);
            canvas_draw_str(canvas,2,34,ip);
            snprintf(line,sizeof(line),"MAC %02X:%02X:%02X:%02X:%02X:%02X",
                     h->mac[0],h->mac[1],h->mac[2],h->mac[3],h->mac[4],h->mac[5]);
            canvas_draw_str(canvas,2,44,line);
        }
        canvas_draw_str(canvas,2,54,s.wol_attempted?
                        (s.wol_ok?"WOL packet sent":"WOL send failed"):
                        "OK send / hold choose");
    } else if(s.page==9) {
        canvas_draw_str(canvas,2,23,"Custom TCP Port");
        const uint8_t* target=s.tcp_target_index==0?s.gateway:
              (s.tcp_target_index<=s.arp_count?s.hosts[s.tcp_target_index-1].ip:s.gateway);
        if(s.custom_result!=PORT_NOT_TESTED && !zero_ip(s.tcp_custom_ip)) target=s.tcp_custom_ip;
        ip_string(ip,sizeof(ip),target);
        snprintf(line,sizeof(line),"Target %s",ip);
        canvas_draw_str(canvas,2,33,line);
        snprintf(line,sizeof(line),"Port %u  %s",s.custom_port,port_text(s.custom_result));
        canvas_draw_str(canvas,2,44,line);
        static const uint16_t step[5]={1,10,100,1000,10000};
        snprintf(line,sizeof(line),"Step:%u  hold </> set",step[s.custom_digit]);
        canvas_draw_str(canvas,2,54,line);
    } else if(s.page==10) {
        canvas_draw_str(canvas,2,23,"Export CSV to microSD");
        canvas_draw_str(canvas,2,36,"OK = write report.csv");
        canvas_draw_str(canvas,2,50,s.csv_attempted?
                        (s.csv_ok?"Saved successfully":"Save failed (SD?)"):
                        "Overwrite last export");
    } else if(s.page==11) {
        snprintf(line,sizeof(line),"Discovery %s",s.disc_choice==DISC_SSDP?"SSDP":"mDNS");
        canvas_draw_str(canvas,2,23,line);
        const char* status=s.disc_stage==DISC_WAIT?"Receiving replies...":
                           s.disc_stage==DISC_DONE?"Scan finished":
                           s.disc_stage==DISC_FAIL?"Network/send error":"OK start / UP change";
        canvas_draw_str(canvas,2,34,status);
        snprintf(line,sizeof(line),"Found: %u (max %u)",s.disc_count,DISC_LIMIT);
        canvas_draw_str(canvas,2,44,line);
        if(s.disc_count) {
            const DiscoveredService* host=&s.services[s.disc_view];
            ip_string(ip,sizeof(ip),host->ip);
            snprintf(line,sizeof(line),"%s %s",ip,host->description);
            canvas_draw_str(canvas,2,54,line);
        }
    } else {
        canvas_draw_str(canvas,2,23,"Network inventory");
        snprintf(line,sizeof(line),"ARP hosts %u",s.arp_count);
        canvas_draw_str(canvas,2,33,line);
        if(s.arp_count) {
            const ArpHost* host=&s.hosts[s.arp_view];
            ip_string(ip,sizeof(ip),host->ip);
            canvas_draw_str(canvas,2,43,ip);
            snprintf(line,sizeof(line),"Vendor: %s",mac_vendor(host->mac));
            canvas_draw_str(canvas,2,53,line);
        } else canvas_draw_str(canvas,2,43,"Run ARP scan (page 5)");
    }
    snprintf(line,sizeof(line),"< > page %u/%u  OK",(unsigned)(s.page+1),PAGES);
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
    app.s.custom_port=8080;
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
        if(got==FuriStatusOk && event.type==InputTypeLong &&
           event.key==InputKeyOk && app.s.page==5 &&
           app.s.tcp_stage!=SCAN_ACTIVE) {
            /* Next target: gateway then each discovered ARP host. */
            app.s.tcp_target_index=(uint8_t)((app.s.tcp_target_index+1)%(app.s.arp_count+1));
            app.s.tcp_stage=SCAN_IDLE;
            app.s.tcp_index=0;
            memset(app.s.tcp_standard_ip,0,4);
            memset(app.s.tcp_results,0,sizeof(app.s.tcp_results));
        }
        if(got==FuriStatusOk && event.type==InputTypeLong &&
           event.key==InputKeyOk && app.s.page==8 && app.s.arp_count>0 &&
           !network_busy(&app.s)) {
            app.s.wol_target_index=(uint8_t)((app.s.wol_target_index+1)%app.s.arp_count);
            app.s.wol_attempted=false;
        }
        if(got==FuriStatusOk && event.type==InputTypeLong &&
           app.s.page==9 &&
           (event.key==InputKeyLeft || event.key==InputKeyRight)) {
            app.s.custom_digit=(uint8_t)((app.s.custom_digit +
                (event.key==InputKeyRight?1:4))%5);
        }
        if(got==FuriStatusOk && event.type==InputTypeLong &&
           event.key==InputKeyOk && app.s.page==9 &&
           app.s.tcp_stage!=SCAN_ACTIVE) {
            app.s.tcp_target_index=(uint8_t)((app.s.tcp_target_index+1)%(app.s.arp_count+1));
            app.s.custom_result=PORT_NOT_TESTED;
            memset(app.s.tcp_custom_ip,0,4);
        }
        if(got==FuriStatusOk && event.type==InputTypeLong && app.s.page==9 &&
           (event.key==InputKeyUp || event.key==InputKeyDown) &&
           app.s.tcp_stage!=SCAN_ACTIVE) {
            static const uint16_t step[5]={1,10,100,1000,10000};
            int next=(int)app.s.custom_port+(event.key==InputKeyUp?10:-10)*(int)step[app.s.custom_digit];
            if(next>65535) next=65535;
            if(next<1) next=1;
            app.s.custom_port=(uint16_t)next;
            app.s.custom_result=PORT_NOT_TESTED;
        }
        if(got==FuriStatusOk && event.type==InputTypeShort) {
            if(event.key==InputKeyBack) running=false;
            else if(event.key==InputKeyRight) app.s.page=(app.s.page+1)%PAGES;
            else if(event.key==InputKeyLeft) app.s.page=(app.s.page+PAGES-1)%PAGES;
            else if(event.key==InputKeyDown || event.key==InputKeyUp) {
                int delta=(event.key==InputKeyDown)?1:-1;
                if(app.s.page==4) {
                    int v=(int)app.s.arp_view+delta;
                    if(v>=0 && v<(int)app.s.arp_count) app.s.arp_view=(uint8_t)v;
                } else if(app.s.page==5) {
                    int v=(int)app.s.tcp_view+delta;
                    if(v>=0 && v<(int)TCP_PORT_COUNT) app.s.tcp_view=(uint8_t)v;
                } else if(app.s.page==7 && app.s.dns_stage!=DNS_WAIT) {
                    app.s.dns_choice=(uint8_t)((app.s.dns_choice+DNS_NAME_COUNT+delta)%DNS_NAME_COUNT);
                    app.s.dns_stage=DNS_IDLE;
                    memset(app.s.dns_answer,0,4);
                } else if(app.s.page==11 && app.s.disc_stage!=DISC_WAIT) {
                    if(event.key==InputKeyUp) {
                        app.s.disc_choice=app.s.disc_choice==DISC_SSDP?DISC_MDNS:DISC_SSDP;
                        app.s.disc_stage=DISC_IDLE;
                        app.s.disc_count=0;
                        app.s.disc_view=0;
                    } else if(app.s.disc_count) {
                        app.s.disc_view=(uint8_t)((app.s.disc_view+1)%app.s.disc_count);
                    }
                } else if(app.s.page==12) {
                    int v=(int)app.s.arp_view+delta;
                    if(v>=0 && v<(int)app.s.arp_count) app.s.arp_view=(uint8_t)v;
                } else if(app.s.page==9 && app.s.tcp_stage!=SCAN_ACTIVE) {
                    static const uint16_t step[5]={1,10,100,1000,10000};
                    int next=(int)app.s.custom_port+delta*(int)step[app.s.custom_digit];
                    if(next>65535) next=65535;
                    if(next<1) next=1;
                    app.s.custom_port=(uint16_t)next;
                    app.s.custom_result=PORT_NOT_TESTED;
                } else app.s.page=(app.s.page+PAGES+(delta>0?1:PAGES-1))%PAGES;
            }
            else if(event.key==InputKeyOk) {
                if(app.s.page==2 && app.s.arp_stage!=SCAN_ACTIVE &&
                   app.s.arp_stage!=SCAN_DRAIN && app.s.tcp_stage!=SCAN_ACTIVE)
                    dhcp_start(&app.s);
                else if(app.s.page==3 && !network_busy(&app.s))
                    ping_start(&app.s);
                else if(app.s.page==4 && !network_busy(&app.s))
                    arp_start(&app.s);
                else if(app.s.page==5 && !network_busy(&app.s)) tcp_start(&app.s,false);
                else if(app.s.page==6 && !network_busy(&app.s))
                    (void)save_report(&app.s);
                else if(app.s.page==7) dns_start(&app.s);
                else if(app.s.page==8) wol_send(&app.s);
                else if(app.s.page==9 && !network_busy(&app.s)) tcp_start(&app.s,true);
                else if(app.s.page==10 && !network_busy(&app.s))
                    (void)save_csv(&app.s);
                else if(app.s.page==11 && !network_busy(&app.s))
                    discovery_start(&app.s);
                else if(app.s.page<2) update_physical(&app.s);
            }
        }
        if(running) {
            if(expired(refresh_at)) {update_physical(&app.s);refresh_at=after_ms(1100);}
            if(!app.s.found || !(app.s.phy&1)) {
                if(app.s.dhcp==D_OFFER || app.s.dhcp==D_ACK) dhcp_fail(&app.s);
                if(app.s.ping==P_WAIT) { sock_close(S_PING); app.s.ping=P_FAIL; }
                if(app.s.arp_stage==SCAN_ACTIVE || app.s.arp_stage==SCAN_DRAIN)
                    arp_stop(&app.s,true);
                if(app.s.tcp_stage==SCAN_ACTIVE) tcp_stop(&app.s,true);
                if(app.s.dns_stage==DNS_WAIT) {sock_close(S_DNS);app.s.dns_stage=DNS_FAIL;}
                if(app.s.disc_stage==DISC_WAIT) {sock_close(S_DISC);app.s.disc_stage=DISC_FAIL;}
            } else {
                if(app.s.dhcp==D_IDLE) dhcp_start(&app.s);
                dhcp_tick(&app.s);
                ping_tick(&app.s);
                arp_tick(&app.s);
                tcp_tick(&app.s);
                dns_tick(&app.s);
                discovery_tick(&app.s);
            }
        }
        furi_mutex_release(app.mutex);
        if(running) view_port_update(vp);
    }
    gui_remove_view_port(gui,vp);
    view_port_free(vp);
    furi_record_close(RECORD_GUI);
    sock_close(S_DHCP); sock_close(S_PING); sock_close(S_TCP);
    sock_close(S_DNS); sock_close(S_WOL); sock_close(S_DISC);
    tcp_restore_timeout(&app.s);
    furi_hal_spi_bus_handle_deinit(&furi_hal_spi_bus_handle_external);
    if(!previous_5v && enabled) furi_hal_power_disable_otg();
    furi_message_queue_free(queue);
    furi_mutex_free(app.mutex);
    return 0;
}
