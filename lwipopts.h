#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

// Common settings used in most pico_w examples
#define NO_SYS                      1
#define LWIP_SOCKET                 0
#define LWIP_NETCONN                0
#define LWIP_NETIF_TX_SINGLE_PBUF   1
#define LWIP_IPV4                   1
#define LWIP_IPV6                   0
#define LWIP_DHCP                   1
#define LWIP_ICMP                   1
#define LWIP_UDP                    1
#define LWIP_TCP                    0
#define TCP_MSS                     1460
#define TCP_WND                     (8 * TCP_MSS)
#define TCP_SND_BUF                 (8 * TCP_MSS)
#define TCP_LISTEN_BACKLOG          1
#define MEMP_NUM_TCP_SEG            32
#define MEMP_NUM_ARP_QUEUE          10
#define PBUF_POOL_SIZE              24

#endif /* _LWIPOPTS_H */
