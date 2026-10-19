#ifndef _LWIPOPTS_H
#define _LWIPOPTS_H

// Based on the pico_w examples (lwipopts_examples_common.h), trimmed to
// what the car needs: an access point that only receives UDP.
#define NO_SYS                      1
#define LWIP_SOCKET                 0
#define LWIP_NETCONN                0
#define MEM_LIBC_MALLOC             0
#define MEM_ALIGNMENT               4
#define MEM_SIZE                    4000
#define MEMP_NUM_ARP_QUEUE          10
#define PBUF_POOL_SIZE              24
#define LWIP_ARP                    1
#define LWIP_ETHERNET               1
#define LWIP_NETIF_TX_SINGLE_PBUF   1
#define LWIP_NETIF_STATUS_CALLBACK  1
#define LWIP_NETIF_LINK_CALLBACK    1
#define LWIP_NETIF_HOSTNAME         1
#define LWIP_CHKSUM_ALGORITHM       3
#define LWIP_IPV4                   1
#define LWIP_IPV6                   0
#define LWIP_ICMP                   1
#define LWIP_UDP                    1
#define LWIP_TCP                    0

// The AP has a fixed IP (192.168.4.1) and the remote uses a static IP,
// so no DHCP client is needed.
#define LWIP_DHCP                   0

#endif /* _LWIPOPTS_H */
