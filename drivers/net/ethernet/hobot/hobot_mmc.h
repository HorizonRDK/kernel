#ifndef __HOBOT_MMC_H_
#define __HOBOT_MMC_H_

#define MMC_CNTRL_COUNTER_RESET 0x1
#define MMC_CNTRL_COUNTER_STOP_ROLLOVER 0x2
#define MMC_CNTRL_RESET_ON_READ 0x4
#define MMC_CNTRL_COUNTER_FREEZER 0x8

#define MMC_CNTRL_PRESET 0x10
#define MMC_CNTRL_FULL_HALF_PRESET 0x20
#define MMC_GMAC4_OFFSET 0x700

struct hobot_counters {
    unsigned int mmc_tx_octetcount_gb;
    unsigned int mmc_tx_framecount_gb;
    unsigned int mmc_tx_broadcastframe_g;
    unsigned int mmc_tx_multicastframe_g;
    unsigned int mmc_tx_64_octets_gb;
    unsigned int mmc_tx_65_to_127_octets_gb;
    unsigned int mmc_tx_128_to_255_octets_gb;
    unsigned int mmc_tx_256_to_511_octets_gb;
    unsigned int mmc_tx_512_to_1023_octets_gb;
    unsigned int mmc_tx_1024_to_max_octets_gb;
    unsigned int mmc_tx_unicast_gb;
    unsigned int mmc_tx_multicast_gb;
    unsigned int mmc_tx_broadcast_gb;
    unsigned int mmc_tx_underflow_error;
    unsigned int mmc_tx_singlecol_g;
    unsigned int mmc_tx_multicol_g;
    unsigned int mmc_tx_deferred;
    unsigned int mmc_tx_latecol;
    unsigned int mmc_tx_exesscol;
    unsigned int mmc_tx_carrier_error;
    unsigned int mmc_tx_octetcount_g;
    unsigned int mmc_tx_framecount_g;
    unsigned int mmc_tx_excessdef;
    unsigned int mmc_tx_pause_frame;
    unsigned int mmc_tx_vlan_frame_g;

    /* MMC RX counter registers */
    unsigned int mmc_rx_framecount_gb;
    unsigned int mmc_rx_octetcount_gb;
    unsigned int mmc_rx_octetcount_g;
    unsigned int mmc_rx_broadcastframe_g;
    unsigned int mmc_rx_multicastframe_g;
    unsigned int mmc_rx_crc_error;
    unsigned int mmc_rx_align_error;
    unsigned int mmc_rx_run_error;
    unsigned int mmc_rx_jabber_error;
    unsigned int mmc_rx_undersize_g;
    unsigned int mmc_rx_oversize_g;
    unsigned int mmc_rx_64_octets_gb;
    unsigned int mmc_rx_65_to_127_octets_gb;
    unsigned int mmc_rx_128_to_255_octets_gb;
    unsigned int mmc_rx_256_to_511_octets_gb;
    unsigned int mmc_rx_512_to_1023_octets_gb;
    unsigned int mmc_rx_1024_to_max_octets_gb;
    unsigned int mmc_rx_unicast_g;
    unsigned int mmc_rx_length_error;
    unsigned int mmc_rx_autofrangetype;
    unsigned int mmc_rx_pause_frames;
    unsigned int mmc_rx_fifo_overflow;
    unsigned int mmc_rx_vlan_frames_gb;
    unsigned int mmc_rx_watchdog_error;

/* IPC */
    unsigned int mmc_rx_ipc_intr_mask;
    unsigned int mmc_rx_ipc_intr;
    /* IPv4 */
    unsigned int mmc_rx_ipv4_gd;
    unsigned int mmc_rx_ipv4_hderr;
    unsigned int mmc_rx_ipv4_nopay;
    unsigned int mmc_rx_ipv4_frag;
    unsigned int mmc_rx_ipv4_udsbl;

    unsigned int mmc_rx_ipv4_gd_octets;
    unsigned int mmc_rx_ipv4_hderr_octets;
    unsigned int mmc_rx_ipv4_nopay_octets;
    unsigned int mmc_rx_ipv4_frag_octets;
    unsigned int mmc_rx_ipv4_udsbl_octets;

    /* IPV6 */
    unsigned int mmc_rx_ipv6_gd_octets;
    unsigned int mmc_rx_ipv6_hderr_octets;
    unsigned int mmc_rx_ipv6_nopay_octets;
    unsigned int mmc_rx_ipv6_gd;
    unsigned int mmc_rx_ipv6_hderr;
    unsigned int mmc_rx_ipv6_nopay;

    /* Protocols */
    unsigned int mmc_rx_udp_gd;
    unsigned int mmc_rx_udp_err;
    unsigned int mmc_rx_tcp_gd;
    unsigned int mmc_rx_tcp_err;
    unsigned int mmc_rx_icmp_gd;
    unsigned int mmc_rx_icmp_err;

    unsigned int mmc_rx_udp_gd_octets;
    unsigned int mmc_rx_udp_err_octets;
    unsigned int mmc_rx_tcp_gd_octets;
    unsigned int mmc_rx_tcp_err_octets;
    unsigned int mmc_rx_icmp_gd_octets;
    unsigned int mmc_rx_icmp_err_octets;
};

void hobot_mmc_read(void *__iomem addr, struct hobot_counters *mmc);
void hobot_mmc_intr_all_mask(void *__iomem addr);
void hobot_mmc_ctrl(void *__iomem addr, unsigned int mode);
#endif
