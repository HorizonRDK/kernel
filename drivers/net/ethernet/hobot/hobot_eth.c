/*
 *
 * This code from  Synopsys DWC Ethernet Quality-of-Service v4.10a linux driver
 *
 *  This is a driver for the Synopsys DWC Ethernet QoS IP version 4.10a (GMAC).
 *  This version introduced a lot of changes which breaks backwards
 *  compatibility the non-QoS IP from Synopsys (used in the ST Micro drivers).
 *  Some fields differ between version 4.00a and 4.10a, mainly the interrupt
 *  bit fields. The driver could be made compatible with 4.00, if all relevant
 *  HW erratas are handled.
 *
 *  The GMAC is highly configurable at synthesis time. This driver has been
 *  developed for a subset of the total available feature set. Currently
 *  it supports:
 *  - TSO
 *  - Checksum offload for RX and TX.
 *  - Energy efficient ethernet.
 *  - GMII phy interface.
 *  - The statistics module.
 *  - Single RX and TX queue.
 *
 *  Copyright (C) 2015 Axis Communications AB.
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms and conditions of the GNU General Public License,
 *  version 2, as published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/ethtool.h>
#include <linux/stat.h>
#include <linux/types.h>

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/platform_device.h>

#include <linux/phy.h>
#include <linux/mii.h>
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>

#include <linux/device.h>
#include <linux/bitrev.h>
#include <linux/crc32.h>

#include <linux/of.h>
#include <linux/interrupt.h>
#include <linux/clocksource.h>
#include <linux/net_tstamp.h>
#include <linux/pm_runtime.h>
#include <linux/of_net.h>
#include <linux/of_address.h>
#include <linux/of_mdio.h>
#include <linux/timer.h>
#include <linux/tcp.h>
#include <linux/if_vlan.h>
#include <linux/ip.h>
#include <uapi/linux/if_arp.h>

#include "hobot_eth.h"
#include "hobot_reg.h"
#include "hobot_tsn.h"



#define TSO_MAX_BUFF_SIZE (SZ_16K - 1)

//#define DRIVER_NAME "hobot-x2-ethernet driver"
#define DRIVER_NAME "st_gmac"

#define x2_reg_read(priv, reg) readl_relaxed(((void __iomem*)((priv)->ioaddr)) + (reg))

#define x2_reg_write(priv,reg,val) writel_relaxed((val), ((void __iomem*)((priv)->ioaddr)) + (reg))

static int x2_mdio_read(struct mii_bus *bus, int mii_id, int phyreg)
{
	struct net_device *ndev = bus->priv;
	struct x2_priv *priv = netdev_priv(ndev);
	u32 regval;
	int i;
	int data;

//	printk("%s, reg:0x%x\n",__func__,phyreg);
	regval = DWCEQOS_MDIO_PHYADDR(mii_id) |
		DWCEQOS_MDIO_PHYREG(phyreg) |
		DWCEQOS_MAC_MDIO_ADDR_CR(priv->csr_val) |
		DWCEQOS_MAC_MDIO_ADDR_GB |
		DWCEQOS_MAC_MDIO_ADDR_GOC_READ;
	x2_reg_write(priv, REG_DWCEQOS_MAC_MDIO_ADDR, regval);

	for (i = 0; i < 5; ++i) {
		usleep_range(64, 128);
		if (!(x2_reg_read(priv, REG_DWCEQOS_MAC_MDIO_ADDR) &
		      DWCEQOS_MAC_MDIO_ADDR_GB))
			break;
	}

	data = x2_reg_read(priv, REG_DWCEQOS_MAC_MDIO_DATA);
	if (i == 5) {
		netdev_warn(ndev, "MDIO read timed out\n");
		data = 0xffff;
	}

//	printk("%s, reg:0x%x, data:0x%x\n",__func__,phyreg,data & 0xffff);
	return data & 0xffff;
}


static int x2_mdio_write(struct mii_bus *bus, int mii_id, int phyreg, u16 value)
{
	struct net_device *ndev = bus->priv;
	struct x2_priv *priv = netdev_priv(ndev);

	u32 regval;
	int i;

//	printk("%s, reg:0x%x, value:0x%x\n",__func__,phyreg,value);
	x2_reg_write(priv, REG_DWCEQOS_MAC_MDIO_DATA, value);

	regval = DWCEQOS_MDIO_PHYADDR(mii_id) |
		DWCEQOS_MDIO_PHYREG(phyreg) |
		DWCEQOS_MAC_MDIO_ADDR_CR(priv->csr_val) |
		DWCEQOS_MAC_MDIO_ADDR_GB |
		DWCEQOS_MAC_MDIO_ADDR_GOC_WRITE;
	x2_reg_write(priv, REG_DWCEQOS_MAC_MDIO_ADDR, regval);

	for (i = 0; i < 5; ++i) {
		usleep_range(64, 128);
		if (!(x2_reg_read(priv, REG_DWCEQOS_MAC_MDIO_ADDR) &
		      DWCEQOS_MAC_MDIO_ADDR_GB))
			break;
	}
	if (i == 5)
		netdev_warn(priv->dev, "MDIO write timed out\n");
	return 0;
}
static int x2_dt_phy(struct plat_config_data *plat, struct device_node *np, struct device *dev)
{

	bool mdio = true;

	static const struct of_device_id need_mdio_ids[] = {
		{.compatible = "snps,dwc-qos-ethernet-4.10a" },
		{},
	};


	plat->phy_node = of_parse_phandle(np, "phy-handle", 0);
	if (plat->phy_node)
		printk("found phy-handle subnode\n");

//	printk("%s, plat->phy_node: %p\n",__func__, plat->phy_node);

	if (!plat->phy_node && of_phy_is_fixed_link(np)) {
		if ((of_phy_register_fixed_link(np) < 0))
			return -ENODEV;

		dev_dbg(dev, "Found fixed-link subnode\n");
		plat->phy_node = of_node_get(np);
		mdio = false;
	}

	if (of_match_node(need_mdio_ids, np)) {
		plat->mdio_node = of_get_child_by_name(np, "mdio");
	} else {

		for_each_child_of_node(np, plat->mdio_node) {
			if (of_device_is_compatible(plat->mdio_node, "snps,dwmac-mdio"))
				break;
		}
	}


	if (plat->mdio_node) {
		mdio = true;
	}

	if (mdio)
		plat->mdio_bus_data = devm_kzalloc(dev, sizeof(struct x2_mdio_bus_data), GFP_KERNEL);



#if 0
	plat->mdio_node = of_get_child_by_name(np, "mdio");
	if (!plat->mdio_node)
		printk("%s,mdio_node is NULL\n",__func__);
#endif


	return 0;
}

static void x2_mtl_setup(struct platform_device *pdev, struct plat_config_data *plat)
{
	struct device_node *q_node;
	struct device_node *rx_node;
	struct device_node *tx_node;
	u8 queue = 0;
/*common data*/


	plat->rx_queues_to_use = 1;
	plat->tx_queues_to_use = 1;

	plat->rx_queues_cfg[0].mode_to_use = MTL_QUEUE_DCB;
	plat->tx_queues_cfg[0].mode_to_use = MTL_QUEUE_DCB;

	rx_node = of_get_child_by_name(pdev->dev.of_node,"snps,mtl-rx-config");
//	rx_node = of_parse_phandle(pdev->dev.of_node,"snps,mtl-rx-config",0);
	if (!rx_node) {
		printk("%s, snps,mtl-rx-config rx-node is NULL\n",__func__);
		return;
	}

	tx_node = of_get_child_by_name(pdev->dev.of_node,"snps,mtl-tx-config");
//	tx_node = of_parse_phandle(pdev->dev.of_node,"snps,mtl-tx-config",0);
	if (!tx_node)
		return;

	if (of_property_read_u32(rx_node, "snps,rx-queues-to-use",&plat->rx_queues_to_use))
		plat->rx_queues_to_use = 1;

	if (of_property_read_u32(tx_node, "snps,tx-queues-to-use", &plat->tx_queues_to_use))
		plat->tx_queues_to_use = 1;

	printk("%s, plat->rx-queues-to-use:%d, and tx:%d\n",__func__,plat->rx_queues_to_use, plat->tx_queues_to_use);

	if (of_property_read_bool(rx_node,"snps,rx-sched-sp"))
		plat->rx_sched_algorithm = MTL_RX_ALGORITHM_SP;
	else if (of_property_read_bool(rx_node,"snps,rx-sched-wsp"))
		plat->rx_sched_algorithm = MTL_RX_ALGORITHM_WSP;
	else
		plat->rx_sched_algorithm = MTL_RX_ALGORITHM_SP;

	for_each_child_of_node(rx_node, q_node) {
		if (queue >= plat->rx_queues_to_use)
			break;

		if (of_property_read_bool(q_node, "snps,dcb-algorithm"))
			plat->rx_queues_cfg[queue].mode_to_use = MTL_QUEUE_DCB;
		else if (of_property_read_bool(q_node,"snps,avb-algorithm"))
			plat->rx_queues_cfg[queue].mode_to_use = MTL_QUEUE_AVB;
		else
			plat->rx_queues_cfg[queue].mode_to_use = MTL_QUEUE_DCB;

		if (of_property_read_u32(q_node, "snps,map-to-dma-channel",&plat->rx_queues_cfg[queue].chan))
			plat->rx_queues_cfg[queue].chan = queue;

		if (of_property_read_u32(q_node,"snps,priority",&plat->rx_queues_cfg[queue].prio)) {
			plat->rx_queues_cfg[queue].prio = 0;
			plat->rx_queues_cfg[queue].use_prio = false;
		} else {

			plat->rx_queues_cfg[queue].use_prio = true;
		}


		if (of_property_read_bool(q_node,"snps,route-avcp"))
			plat->rx_queues_cfg[queue].pkt_route = PACKET_AVCPQ;
		else if (of_property_read_bool(q_node,"snps,route-ptp"))
			plat->rx_queues_cfg[queue].pkt_route = PACKET_PTPQ;
		else if (of_property_read_bool(q_node,"snps,route-dcbcp"))
			plat->rx_queues_cfg[queue].pkt_route = PACKET_DCBCPQ;
		else
			plat->rx_queues_cfg[queue].pkt_route = 0x0;

		queue++;
	}


	if (of_property_read_u32(tx_node,"snps,tx-queues-to-use",&plat->tx_queues_to_use))
		plat->tx_queues_to_use = 1;

	if (of_property_read_bool(tx_node,"snps,tx-sched-wrr"))
		plat->tx_sched_algorithm = MTL_TX_ALGORITHM_WRR;
	else if (of_property_read_bool(tx_node,"snps,tx-sched-sp"))
		plat->tx_sched_algorithm = MTL_TX_ALGORITHM_SP;
	else
		plat->tx_sched_algorithm = 0x0;

	queue = 0;

	for_each_child_of_node(tx_node, q_node) {
		if (queue >= plat->tx_queues_to_use)
			break;

		if (of_property_read_u32(q_node, "snps,weigh",&plat->tx_queues_cfg[queue].weight))
			plat->tx_queues_cfg[queue].weight = 0x10 + queue;

		if (of_property_read_bool(q_node,"snps,dcb-algorithm")) {
			plat->tx_queues_cfg[queue].mode_to_use = MTL_QUEUE_DCB;
		} else if (of_property_read_bool(q_node, "snps,avb-algorithm")){
			plat->tx_queues_cfg[queue].mode_to_use = MTL_QUEUE_AVB;

			if (of_property_read_u32(q_node,"snps,send_slope", &plat->tx_queues_cfg[queue].send_slope))
				plat->tx_queues_cfg[queue].send_slope =0x0;

			if (of_property_read_u32(q_node, "snps,idle_slope", &plat->tx_queues_cfg[queue].idle_slope))
				plat->tx_queues_cfg[queue].idle_slope = 0x0;

			if (of_property_read_u32(q_node, "snps,high_credit",&plat->tx_queues_cfg[queue].high_credit))
				plat->tx_queues_cfg[queue].high_credit = 0x0;

			if (of_property_read_u32(q_node, "snps,low_credit",&plat->tx_queues_cfg[queue].low_credit))
				plat->tx_queues_cfg[queue].low_credit = 0x0;

		} else
			plat->tx_queues_cfg[queue].mode_to_use = MTL_QUEUE_DCB;

		if (of_property_read_u32(q_node, "snps,priority",&plat->tx_queues_cfg[queue].prio)) {
			plat->tx_queues_cfg[queue].prio = 0;
			plat->tx_queues_cfg[queue].use_prio = false;
		} else {
			plat->tx_queues_cfg[queue].use_prio = true;
		}

		queue++;

	}


	of_node_put(rx_node);
	of_node_put(tx_node);
	of_node_put(q_node);

}


static struct x2_axi *x2_axi_setup(struct platform_device *pdev)
{
	struct device_node *np;
	struct x2_axi *axi;

	np = of_parse_phandle(pdev->dev.of_node,"snps,axi-config",0);
	if (!np)
		return NULL;

	axi = devm_kzalloc(&pdev->dev, sizeof(*axi), GFP_KERNEL);
	if (!axi) {
		of_node_put(np);
		return ERR_PTR(-ENOMEM);
	}


	axi->axi_lpi_en = of_property_read_bool(np, "snps,lpi_en");
	axi->axi_xit_frm = of_property_read_bool(np,"snps,xit_frm");
	axi->axi_kbbe = of_property_read_bool(np,"snps,axi_kbbe");
	axi->axi_fb = of_property_read_bool(np, "snps,axi_fb");
	axi->axi_mb = of_property_read_bool(np,"snps,axi_mb");
	axi->axi_rb = of_property_read_bool(np, "snps,axi_rb");

	if (of_property_read_u32(np, "snps,wr_osr_lmt",&axi->axi_wr_osr_lmt))
		axi->axi_wr_osr_lmt = 1;

	if (of_property_read_u32(np,"snps,rd_osr_lmt",&axi->axi_rd_osr_lmt))
		axi->axi_rd_osr_lmt = 1;

	of_property_read_u32_array(np,"snps,blen",axi->axi_blen, AXI_BLEN);
	of_node_put(np);
	return axi;

}
struct plat_config_data *x2_probe_config_dt(struct platform_device *pdev, const char **mac)
{
	struct device_node *np = pdev->dev.of_node;
	struct plat_config_data *plat;
	struct x2_dma_cfg *dma_cfg;
	//int ret;

	plat = devm_kzalloc(&pdev->dev, sizeof(*plat), GFP_KERNEL);
	if (!plat)
		return ERR_PTR(-ENOMEM);

	*mac = of_get_mac_address(np);
	plat->interface = of_get_phy_mode(np);


	if (of_property_read_u32(np, "max-speed", &plat->max_speed))
		plat->max_speed = -1;


	plat->bus_id = of_alias_get_id(np, "ethernet");
	if (plat->bus_id < 0)
		plat->bus_id = 0;


	plat->phy_addr = -1;

	if (of_property_read_u32(np, "snps,phy-addr", &plat->phy_addr) == 0)
		printk("snps, phy-addr property is deprecated\n");


	if (x2_dt_phy(plat, np, &pdev->dev))
		return ERR_PTR(-ENODEV);


	pinctrl = devm_pinctrl_get(&pdev->dev);
	if ( IS_ERR(pinctrl) ) {
		dev_err(&pdev->dev, "pinctrl get error\n");
		return PTR_ERR(pinctrl);
	}

	pin_eth_mux = pinctrl_lookup_state(pinctrl, "eth_state");
	if ( IS_ERR(pin_eth_mux) ) {
		dev_err(&pdev->dev, "pins_eth_mux in pinctrl state error\n");
		return PTR_ERR(pin_eth_mux);
	}

        pinctrl_select_state(pinctrl, pin_eth_mux);



	of_property_read_u32(np, "tx-fifo-depth", &plat->tx_fifo_size);
	of_property_read_u32(np, "rx-fifo-depth", &plat->rx_fifo_size);

	plat->force_sf_dma_mode = of_property_read_bool(np, "snps,force_sf_dma_mode");
	plat->en_tx_lpi_clockgating = of_property_read_bool(np, "snps,en-tx-lpi-clockgating");

	plat->maxmtu = JUMBO_LEN;

	plat->has_gmac4 = 1;
	plat->pmt = 1;
	plat->tso_en = of_property_read_bool(np,"snps,tso");



	dma_cfg = devm_kzalloc(&pdev->dev, sizeof(*dma_cfg), GFP_KERNEL);
	if (!dma_cfg)
		return ERR_PTR(-ENOMEM);

	plat->dma_cfg = dma_cfg;

	of_property_read_u32(np, "snps,pbl",&dma_cfg->pbl);
	if (!dma_cfg->pbl)
		dma_cfg->pbl = DEFAULT_DMA_PBL;

//	of_property_read_u32(np,"snps,read-requests",&dma_cfg->read_requests);
//	of_property_read_u32(np,"snps,write-requets",&dma_cfg->write_requests);

	of_property_read_u32(np, "snps,txpbl", &dma_cfg->txpbl);
	of_property_read_u32(np, "snps,rxpbl", &dma_cfg->rxpbl);
	dma_cfg->pblx8 = !of_property_read_bool(np, "snps,no-plb-x8");
	dma_cfg->aal = of_property_read_bool(np, "snps,aal");
	dma_cfg->fixed_burst = of_property_read_bool(np, "snps,fixed-burst");
	dma_cfg->mixed_burst = of_property_read_bool(np, "snps,mixed-burst");

	plat->force_thresh_dma_mode = of_property_read_bool(np,"snps,force_thresh_dma_mode");
	if (plat->force_thresh_dma_mode) {
		plat->force_sf_dma_mode = 0;
	}

	of_property_read_u32(np, "snps,ps-speed",&plat->mac_port_sel_speed);

	plat->axi = x2_axi_setup(pdev);

	x2_mtl_setup(pdev, plat);

#if 0
	plat->x2_mac_pre_div_clk = devm_clk_get(&pdev->dev, "mac_pre_div_clk");
	if (IS_ERR(plat->x2_mac_pre_div_clk)) {
		printk("mac pre div clk clock not found\n");
		goto err_out;
	}
	ret = clk_prepare_enable(plat->x2_mac_pre_div_clk);
	if (ret) {
		printk("unable to enable mac pre div clk\n");
		goto err_out;
	}

	plat->x2_mac_div_clk = devm_clk_get(&pdev->dev, "mac_div_clk");
	if (IS_ERR(plat->x2_mac_div_clk)) {
		printk("mac div clk not found\n");
		goto err_mac_div_clk;
	}

	ret = clk_prepare_enable(plat->x2_mac_div_clk);
	if (ret) {
		printk("unable to enable mac div clk\n");
		goto err_mac_div_clk;
	}


	plat->x2_phy_ref_clk = devm_clk_get(&pdev->dev, "phy_ref_clk");
	if (IS_ERR(plat->x2_phy_ref_clk)) {
		printk("uanble to get phy ref clk\n");
		goto err_clk;
	}
	ret = clk_prepare_enable(plat->x2_phy_ref_clk);

#endif
	plat->clk_ptp_rate = 50000000;// 125000000;//50000000 ;//62500000;//58125000;//500000000;
	plat->cdc_delay = 2 * ((1000000000ULL) /plat->clk_ptp_rate);
	return plat;

err_clk:
	clk_disable_unprepare(plat->x2_mac_div_clk);
err_mac_div_clk:

	clk_disable_unprepare(plat->x2_mac_pre_div_clk);

//err_out:
	return ERR_PTR(-EPROBE_DEFER);

}
static int x2_get_hw_features(void __iomem *ioaddr, struct dma_features *dma_cap)
{
	u32 hw_cap = readl(ioaddr + GMAC_HW_FEATURE0);

	/*  MAC HW feature0 */
	dma_cap->mbps_10_100 = (hw_cap & GMAC_HW_FEAT_MIISEL);
	dma_cap->mbps_1000 = (hw_cap & GMAC_HW_FEAT_GMIISEL) >> 1;
	dma_cap->half_duplex = (hw_cap & GMAC_HW_FEAT_HDSEL) >> 2;
	dma_cap->hash_filter = (hw_cap & GMAC_HW_FEAT_VLHASH) >> 4;
	dma_cap->multi_addr = (hw_cap & GMAC_HW_FEAT_ADDMAC) >> 18;
	dma_cap->pcs = (hw_cap & GMAC_HW_FEAT_PCSSEL) >> 3;
	dma_cap->sma_mdio = (hw_cap & GMAC_HW_FEAT_SMASEL) >> 5;
	dma_cap->pmt_remote_wake_up = (hw_cap & GMAC_HW_FEAT_RWKSEL) >> 6;
	dma_cap->pmt_magic_frame = (hw_cap & GMAC_HW_FEAT_MGKSEL) >> 7;
	/* MMC */
	dma_cap->rmon = (hw_cap & GMAC_HW_FEAT_MMCSEL) >> 8;
	//printk("%s, rmod:%d\n",__func__,dma_cap->rmon);
	/* IEEE 1588-2008 */
	dma_cap->atime_stamp = (hw_cap & GMAC_HW_FEAT_TSSEL) >> 12;
	/* 802.3az - Energy-Efficient Ethernet (EEE) */
	dma_cap->eee = (hw_cap & GMAC_HW_FEAT_EEESEL) >> 13;
	/* TX and RX csum */
	dma_cap->tx_coe = (hw_cap & GMAC_HW_FEAT_TXCOSEL) >> 14;
	dma_cap->rx_coe =  (hw_cap & GMAC_HW_FEAT_RXCOESEL) >> 16;
	//printk("%s, tx_coe:%d, rx_coe:%d\n",__func__,dma_cap->tx_coe, dma_cap->rx_coe);
	/* MAC HW feature1 */
	hw_cap = readl(ioaddr + GMAC_HW_FEATURE1);
	dma_cap->av = (hw_cap & GMAC_HW_FEAT_AVSEL) >> 20;
	dma_cap->tsoen = (hw_cap & GMAC_HW_TSOEN) >> 18;
	/* RX and TX FIFO sizes are encoded as log2(n / 128). Undo that by
	 * shifting and store the sizes in bytes.
	 */
//	printk("%s, av:%d\n",__func__,dma_cap->av);
//	printk("%s, tsoen:%d\n",__func__,dma_cap->tsoen);

	dma_cap->tx_fifo_size = 128 << ((hw_cap & GMAC_HW_TXFIFOSIZE) >> 6);
	dma_cap->rx_fifo_size = 128 << ((hw_cap & GMAC_HW_RXFIFOSIZE) >> 0);

//	printk("%s, tx_fifo_size:%d\n",__func__,dma_cap->tx_fifo_size);
//	printk("%s, rx_fifo_size:%d\n",__func__,dma_cap->rx_fifo_size);
	/* MAC HW feature2 */
	hw_cap = readl(ioaddr + GMAC_HW_FEATURE2);
	/* TX and RX number of channels */
	dma_cap->number_rx_channel =
		((hw_cap & GMAC_HW_FEAT_RXCHCNT) >> 12) + 1;
	dma_cap->number_tx_channel =
		((hw_cap & GMAC_HW_FEAT_TXCHCNT) >> 18) + 1;
	/* TX and RX number of queues */
	dma_cap->number_rx_queues =
		((hw_cap & GMAC_HW_FEAT_RXQCNT) >> 0) + 1;
	dma_cap->number_tx_queues =
		((hw_cap & GMAC_HW_FEAT_TXQCNT) >> 6) + 1;

	dma_cap->pps_out_num = (hw_cap & GMAC_HW_FEAT_PPSOUTNUM) >> 24;

//	printk("%s: number tx queues: %d\n",__func__,dma_cap->number_tx_queues);
//	printk("%s: number rx queues: %d\n",__func__,dma_cap->number_rx_queues);

	/*get HW feature3 */
	hw_cap = readl(ioaddr + GMAC_HW_FEATURE3);


	dma_cap->asp = (hw_cap & GMAC_HW_FEAT_ASP) >> 28;
	dma_cap->frpsel = (hw_cap & GMAC_HW_FEAT_FRPSEL) >> 10;
	dma_cap->frpes = (hw_cap & GMAC_HW_FEAT_FRPES) >> 13;

	//dma_cap->tsn_frame_preemption = (hw_cap & GMAC_HW_FEAT_FPESEL) >> 26;
	dma_cap->tbssel = (hw_cap & GMAC_HW_FEAT_TBSSEL) >> 27;
	dma_cap->fpesel = (hw_cap & GMAC_HW_FEAT_FPESEL) >> 26;
	dma_cap->estwid	= (hw_cap & GMAC_HW_FEAT_ESTWID) >> 20;
//	printk("%s, and estwid:%d\n", __func__, dma_cap->estwid);
	dma_cap->estdep = (hw_cap & GMAC_HW_FEAT_ESTDEP) >> 17;
	//dma_cap->tsn_enh_sched_traffic = (hw_cap & GMAC_HW_FEAT_ESTSEL) >> 16;
	dma_cap->estsel = (hw_cap & GMAC_HW_FEAT_ESTSEL) >> 16;
	//dma_cap->tsn = dma_cap->tsn_frame_preemption | dma_cap->tsn_enh_sched_traffic;
	dma_cap->tsn = dma_cap->fpesel | dma_cap->estsel;
//	printk("%s,and frame preemption:%d, enh:%d, tsn:%d\n",__func__,dma_cap->fpesel, dma_cap->estsel, dma_cap->tsn);
	/* IEEE 1588-2002 */
	dma_cap->time_stamp = 0;



	switch (dma_cap->estwid) {
	default:
	case 0x0:
		dma_cap->estwid = 0;
		break;
	case 0x1:
		dma_cap->estwid = 16;
		break;
	case 0x2:
		dma_cap->estwid = 20;
		break;
	case 0x3:
		dma_cap->estwid = 24;
		break;
	}

	switch (dma_cap->estdep) {
	default:
	case 0x0:
		dma_cap->estdep = 0;
		break;
	case 0x1:
		dma_cap->estdep = 64;
		break;
	case 0x2:
		dma_cap->estdep = 128;
		break;
	case 0x3:
		dma_cap->estdep = 256;
		break;
	case 0x4:
		dma_cap->estdep = 512;
		break;
	}

	switch (dma_cap->frpes) {
	default:
		dma_cap->frpes = 0;
		break;
	case 0x0:
		dma_cap->frpes = 64;
		break;
	case 0x1:
		dma_cap->frpes = 128;
		break;
	case 0x2:
		dma_cap->frpes = 256;
		break;
	}



	return 1;
}

static int x2_hw_init(struct x2_priv *priv)
{
	if (priv->plat->force_no_tx_coe)
		priv->plat->tx_coe = 0;
	else
		priv->plat->tx_coe = priv->dma_cap.tx_coe;

	if (!priv->plat->force_no_rx_coe) {
		priv->plat->rx_coe = priv->dma_cap.rx_coe;

		if (priv->dma_cap.rx_coe_type2)
			priv->plat->rx_coe = STMMAC_RX_COE_TYPE2;
		else if (priv->dma_cap.rx_coe_type1)
			priv->plat->rx_coe = STMMAC_RX_COE_TYPE1;
	} else {
		priv->plat->rx_coe = STMMAC_RX_COE_NONE;
	}

	priv->dev->priv_flags |= IFF_UNICAST_FLT;
	priv->hw_cap_support = x2_get_hw_features(priv->ioaddr, &priv->dma_cap);

	if (priv->plat->force_thresh_dma_mode)
		priv->plat->tx_coe = 0;

	if (priv->dma_cap.tsoen)
		printk("TSO supported\n");
	return 0;
}


static void x2_mdio_set_csr(struct x2_priv *priv)
{
	int rate = 20000000;//clk_get_rate(priv->plat->x2_mac_div_clk);

	if (rate <= 20000000)
		priv->csr_val = DWCEQOS_MAC_MDIO_ADDR_CR_20;
	else if (rate <= 35000000)
		priv->csr_val = DWCEQOS_MAC_MDIO_ADDR_CR_35;
	else if (rate <= 60000000)
		priv->csr_val = DWCEQOS_MAC_MDIO_ADDR_CR_60;
	else if (rate <= 100000000)
		priv->csr_val = DWCEQOS_MAC_MDIO_ADDR_CR_100;
	else if (rate <= 150000000)
		priv->csr_val = DWCEQOS_MAC_MDIO_ADDR_CR_150;
	else if (rate <= 250000000)
		priv->csr_val = DWCEQOS_MAC_MDIO_ADDR_CR_250;

}
static int x2_mdio_register(struct x2_priv *priv)
{
	int err = 0;
	struct net_device *ndev = priv->dev;
	struct device_node *mdio_node = priv->plat->mdio_node;
	struct device *dev = ndev->dev.parent;
	struct mii_bus *new_bus;
	struct resource res;
	struct x2_mdio_bus_data *mdio_bus_data = priv->plat->mdio_bus_data;

	if (!mdio_bus_data)
		return 0;


	new_bus = mdiobus_alloc();
	if (!new_bus)
		return  -ENOMEM;

	new_bus->name = "hobot-mac-mdio";
	new_bus->read = &x2_mdio_read;
	new_bus->write = &x2_mdio_write;

	of_address_to_resource(dev->of_node, 0, &res);
	snprintf(new_bus->id, MII_BUS_ID_SIZE, "%s-%llx", new_bus->name, (unsigned long long )res.start);


	new_bus->priv = ndev;
	new_bus->parent = priv->device;

	if (mdio_node)
		err = of_mdiobus_register(new_bus, mdio_node);
	else
		err = mdiobus_register(new_bus);

	if (err != 0) {
		printk("Cannot register the MDIO bus\n");
		goto err_out;
	}




	priv->mii = new_bus;

	return 0;

err_out:

	mdiobus_free(new_bus);
	return  err;
}

#define MAX_FRAME_SIZE 1522
#define MIN_FRAME_SIZE 64

#if 0
static int x2_tsn_est_write(struct x2_priv *priv, u32 reg, u32 val, bool is_gcla)
{
	u32 control = 0x0;
	int timeout = 5;

	writel(val, priv->ioaddr + MTL_EST_GCL_DATA);
	control |= reg;
	control |= is_gcla ? 0x0 : MTL_EST_GCRR;
	control |= 0x0;

	writel(control, priv->ioaddr + MTL_EST_GCL_CONTROL);

	control |= MTL_EST_SRWO;
	writel(control, priv->ioaddr + MTL_EST_GCL_CONTROL);

	while(--timeout) {
		udelay(1000);
		if (readl(priv->ioaddr + MTL_EST_GCL_CONTROL) & MTL_EST_SRWO)
			continue;
		break;
	}

	if (!timeout) {

		printk("failed to write EST reg control 0x%x\n",control);
		return -ETIMEOUT;
	}
	return 0;
}

#endif


static int x2_est_write(struct x2_priv *priv, u32 reg, u32 val, bool is_gcla)
{
	u32 control = 0x0;
	int timeout = 15;

	//printk("%s, before write and reg:0x%x, value:0x%x\n", __func__, reg, val );
	writel(val, priv->ioaddr + MTL_EST_GCL_DATA);
	control |= reg;
	control |= is_gcla ? 0x0 : MTL_EST_GCRR;

	writel(control, priv->ioaddr + MTL_EST_GCL_CONTROL);

	control |= MTL_EST_SRWO;
	writel(control, priv->ioaddr + MTL_EST_GCL_CONTROL);

//	printk("%s, after and read reg:0x%x, value:0x%x\n", __func__, reg, readl(priv->ioaddr + MTL_EST_GCL_DATA) );
//	printk("%s, GCL_DATA_REG(0xc84)  val:0x%x, GCL_CONTROL_REG(0xc80): 0x%x\n", __func__, val, readl(priv->ioaddr + MTL_EST_GCL_CONTROL));
	while(--timeout) {
		udelay(1000);
		if (readl(priv->ioaddr + MTL_EST_GCL_CONTROL) & MTL_EST_SRWO)
			continue;
		break;
	}

	control |= reg;
	control |= is_gcla ? 0x0 : MTL_EST_GCRR;
	control |= (1 << 1);

	writel(control, priv->ioaddr + MTL_EST_GCL_CONTROL);

	timeout = 15;
	while(--timeout) {
		udelay(1000);
		if (readl(priv->ioaddr + MTL_EST_GCL_CONTROL) & MTL_EST_SRWO)
			continue;
		break;
	}



	//printk("%s, after and read reg:0x%x, value:0x%x\n", __func__, reg, readl(priv->ioaddr + MTL_EST_GCL_DATA) );
	if (!timeout) {

		printk("failed to write EST reg control 0x%x\n",control);
		return -ETIMEDOUT;
	}
	return 0;
}


static u32 x2_get_hw_tstamping(struct x2_priv *priv)
{
	return readl(priv->ioaddr + PTP_TCR);
}




static u32 x2_get_ptp_period(struct x2_priv *priv, u32 ptp_clock)
{
	void __iomem *ioaddr = priv->ioaddr;
	u64 data;
	u32 value = readl(ioaddr + PTP_TCR);

	if (value & PTP_TCR_TSCFUPDT)
		data = (1000000000ULL / 50000000);
	else
		data = (1000000000ULL / ptp_clock);


	return (u32)data;
}

static u32 x2_get_ptp_subperiod(struct x2_priv *priv, u32 ptp_clock)
{
	u32 value = readl(priv->ioaddr + PTP_TCR);
	u64 data;

	if (value & PTP_TCR_TSCFUPDT)
		return 0;

	data = (1000000000ULL * 1000ULL / ptp_clock);
	return data - x2_get_ptp_period(priv, ptp_clock) * 1000;
}


static u32 x2_config_sub_second_increment(struct x2_priv *priv, u32 ptp_clock, int gmac4)
{

	void __iomem *ioaddr = priv->ioaddr;
	u32 value = readl(ioaddr + PTP_TCR);
	u32 subns, ns;
	u64 tmp;

	ns = x2_get_ptp_period(priv, ptp_clock);
	subns = x2_get_ptp_subperiod(priv, ptp_clock);

	if (!(value & PTP_TCR_TSCTRLSSR)) {
		tmp = ns * 1000;
		ns = DIV_ROUND_CLOSEST(tmp - (tmp % 465), 465);
		subns = DIV_ROUND_CLOSEST((tmp * 256) - (465 * ns * 256), 465);
	} else {

		subns = DIV_ROUND_CLOSEST(subns * 256, 1000);
	}

	ns &= PTP_SSIR_SSINC_MASK;
	subns &= PTP_SSIR_SNSINC_MASK;

	value = ns;
	value <<= GMAC4_PTP_SSIR_SSINC_SHIFT;
	value |= subns << GMAC4_PTP_SSIR_SNSINC_SHIFT;

	writel(value, ioaddr + PTP_SSIR);

//	printk("%s, PTP_SSIR(0xb04): 0x%x\n", __func__, readl(ioaddr + PTP_SSIR));
	//return value;
	return ns;
}

static int x2_config_addend(struct x2_priv *priv, u32 addend)
{
	void __iomem *ioaddr = priv->ioaddr;
	u32 value;
	int limit;

	writel(addend, ioaddr + PTP_TAR);
	value = readl(ioaddr + PTP_TCR);
	value |= PTP_TCR_TSADDREG;
	writel(value, ioaddr + PTP_TCR);

//	printk("%s, PTP_TAR(0xb18):0x%x, and PTP_TCR(0xb00):0x%x\n", __func__, readl(ioaddr + PTP_TAR), readl(ioaddr+PTP_TCR));
	limit = 10;
	while (limit--) {
		if (!(readl(ioaddr + PTP_TCR) & PTP_TCR_TSADDREG))
			break;
		mdelay(10);
	}

	if (limit < 0)
		return -EBUSY;

	return 0;
}

static int x2_init_systime(struct x2_priv *priv, u32 sec, u32 nsec)
{
	void __iomem *ioaddr = priv->ioaddr;
	int limit;
	u32 value;




	writel(sec, ioaddr + PTP_STSUR);


	/*here may be correct by dhw 2019-06-14*/
	//writel((nsec + (1 << 31)), ioaddr + PTP_STNSUR);
	writel(nsec, ioaddr + PTP_STNSUR);

	value = readl(ioaddr + PTP_TCR);
	value |= PTP_TCR_TSINIT;
	value |= PTP_TCR_TSUPDT;
	writel(value, ioaddr + PTP_TCR);

//	printk("%s,and sec:0x%x, and nsec:0x%x\n", __func__, sec, nsec);
//	printk("%s, PTP_STNSUR(0xb14):0x%x, and  PTP_TCR(0xb00):0x%x\n", __func__, readl(ioaddr + PTP_STNSUR), readl(ioaddr + PTP_TCR));
	limit = 10;
	while (limit--) {
		if (!(readl(ioaddr + PTP_TCR) & PTP_TCR_TSINIT))
			break;
		mdelay(10);
	}


//	writel(0, ioaddr + PTP_STSUR);
//	writel(0, ioaddr + PTP_STNSUR);

	if (limit < 0)
		return -EBUSY;

	return 0;
}


static void x2_config_hw_tstamping(struct x2_priv *priv, u32 data)
{
	writel(data, priv->ioaddr + PTP_TCR);
	//printk("PTP_TCR(0xb00):0x%x\n",readl(priv->ioaddr + PTP_TCR));
}

#if 0
static void x2_tsn_est_configure(struct x2_priv *priv)
{
	u32 *btr = priv->plat->est_cfg.btr_offset;
	u32 llr = priv->plat->est_cfg.gcl_size;
	u32 *ctr = priv->plat->est_cfg.ctr;
	u32 *gcl = priv->plat->est_cfg.gcl;
	u32 ter = priv->plat->est_cfg.ter;
	struct timespec64 now;
	u32 control , sec_inc;
	u64 temp;
	int i;

	ktime_get_real_ts64(&now);
	btr[0] += (u32)now.tv_nsec;
	btr[1] += (u32)now.tv_sec;

	x2_tsn_est_write(priv, MTL_EST_BTR_LOW, btr[0], false);
	x2_tsn_est_write(priv, MTL_EST_BTR_HIGH, btr[1], false);
	x2_tsn_est_write(priv, MTL_EST_CTR_LOW, ctr[0], false);
	x2_tsn_est_write(priv, MTL_EST_CTR_HIGH,ctr[1], false);
	x2_tsn_est_write(priv, MTL_EST_TER, ter, false);
	x2_tsn_est_write(priv, MTL_EST_LLR, llr, false);

	for (i = 0; i < llr; i++) {
		u32 reg = (i << MTL_EST_ADDR_OFFSET) & MTL_EST_ADDR;
		x2_tsn_est_write(priv, reg, gcl[i], true);
	}

	control = MTL_EST_EEST;
	writel(control, priv->ioaddr + MTL_EST_CONTROL);

	control |= MTL_EST_SSWL;
	writel(control, priv->ioaddr + MTL_EST_CONTROL);

	if (!(priv->dma_cap.time_stamp || priv->adv_ts)) {
		printk("No HW time stamping: Disabling EST\n");
		priv->hwts_tx_en = 0;
		priv->hwts_rx_en = 0;
		writel(0, priv->ioaddr + MTL_EST_CONTROL);
		return;
	}

	priv->hwts_tx_en = 1;
	priv->hwts_rx_en = 1;

	sec_inc = x2_config_sub_second_increment(priv, priv->plat->clk_ptp_rate, 0);
	temp = div_u64(1000000000ULL, sec_inc);

	temp = (u64)(temp << 32);
	priv->default_addend = div_u64(temp, priv->plat->clk_ptp_rate);
	x2_config_addend(priv, priv->default_addend);
	x2_init_systime(priv, (u32)now.tv_sec, now.tv_nsec);

	control = PTP_TCR_TSENA | PTP_TCR_TSINIT|PTP_TCR_TSENALL|PTP_TCR_TSCTRLSSR;
	x2_config_hw_tstamping(priv, control);
}


#endif


static int  x2_est_init(struct net_device *ndev, struct x2_priv *priv, struct x2_est_cfg *cfg, unsigned int estsel, unsigned int estdep, unsigned int estwid, bool enable, struct timespec64 *now)
{
	void __iomem *ioaddr = priv->ioaddr;
	u32 control, real_btr[2];
	u8 ptov = 0;

	//struct timespec64 now;
	int i;

	if (!estsel || !estdep || !estwid || !cfg)
		return -EINVAL;

	if (cfg->gcl_size > estdep) {
		printk("%s, Invalid EST configuration supplied\n", __func__);
		return -EINVAL;
	}


	control = readl(ioaddr + MTL_EST_CONTROL);
	control &= ~MTL_EST_EEST;

	writel(control, ioaddr + MTL_EST_CONTROL);

	if (!enable)
		return -EINVAL;


//	ktime_get_real_ts64(&now);
	real_btr[0] = cfg->btr_offset[0] + (u32)now->tv_nsec;
	real_btr[1] = cfg->btr_offset[1] + (u32)now->tv_sec;


#define EST_WRITE(__a, __b, __c) do { \
	if (x2_est_write(priv, __a, __b,__c)) \
		goto write_fail;\
	} while (0);

	EST_WRITE(MTL_EST_BTR_LOW, real_btr[0], false);
	EST_WRITE(MTL_EST_BTR_HIGH, real_btr[1], false);

	EST_WRITE(MTL_EST_CTR_LOW, cfg->ctr[0], false);
	EST_WRITE(MTL_EST_CTR_HIGH, cfg->ctr[1], false);
	EST_WRITE(MTL_EST_TER, cfg->ter, false);
	EST_WRITE(MTL_EST_LLR, cfg->gcl_size, false);

//	printk("%s, and before for ...\n", __func__);
	for (i = 0; i < cfg->gcl_size; i++) {
		u32 reg = (i << MTL_EST_ADDR_OFFSET) & MTL_EST_ADDR;
	//	printk("%s, %d gcl:0x%x\n",__func__,i, cfg->gcl[i]);
		EST_WRITE(reg, cfg->gcl[i], true);
	}

//	printk("%s, and after for ...\n", __func__);
	if (priv->plat->clk_ptp_rate) {
		ptov = (1000000000ULL)/ priv->plat->clk_ptp_rate;
		ptov *= 6;
	}

	control = MTL_EST_EEST | (ptov << 24);
	writel(control, ioaddr + MTL_EST_CONTROL);

	control |= MTL_EST_SSWL;
	writel(control, ioaddr + MTL_EST_CONTROL);


//	printk("%s,and est control reg(0xc50):0x%x\n", __func__, readl(ioaddr + MTL_EST_CONTROL));
	return 0;
write_fail:
	printk("%s:Failed to write EST config\n",__func__);
	return -ETIMEDOUT;
}


static void x2_est_intr_config(struct x2_priv *priv)
{

	writel(0x1F,priv->ioaddr + 0xc70);
}


static int x2_est_configuration(struct x2_priv *priv)
{
	struct timespec64 now;
	u32 control, sec_inc;
	int ret = -EINVAL;
	u64 temp;

	ktime_get_real_ts64(&now);


	x2_est_intr_config(priv);
#if 0
	ret = x2_est_init(priv->dev, priv, &priv->plat->est_cfg, priv->dma_cap.estsel, priv->dma_cap.estdep,priv->dma_cap.estwid,priv->plat->est_en, &now);

	if (ret) {
		priv->est_enabled = false;

	 } else
		 priv->est_enabled = true;

#endif

	if (!(priv->dma_cap.time_stamp || priv->adv_ts)) {
		printk("%s, No HW time stamping: Disabling EST\n", __func__);
		priv->hwts_tx_en = 0;
		priv->hwts_rx_en = 0;
		priv->est_enabled = false;
		writel(0, priv->ioaddr + MTL_EST_CONTROL);

		return -EINVAL;
	}

	control = 0;
	x2_config_hw_tstamping(priv, control);

	priv->hwts_tx_en = 1;
	priv->hwts_rx_en = 1;

	//control = PTP_TCR_TSCFUPDT |PTP_TCR_TSCTRLSSR;
	control = PTP_TCR_TSCTRLSSR;
	x2_config_hw_tstamping(priv, control);



	sec_inc = x2_config_sub_second_increment(priv, priv->plat->clk_ptp_rate, 0);
	temp = div_u64(1000000000ULL, sec_inc);

	temp = (u64)(temp << 32);
	priv->default_addend = div_u64(temp, priv->plat->clk_ptp_rate);
	priv->default_addend = 0;
	x2_config_addend(priv, priv->default_addend);
	x2_init_systime(priv, (u32)now.tv_sec, now.tv_nsec);

	//control = PTP_TCR_TSENA|PTP_TCR_TSCFUPDT | PTP_TCR_TSINIT|PTP_TCR_TSENALL|PTP_TCR_TSCTRLSSR;
	control = PTP_TCR_TSENA | PTP_TCR_TSINIT|PTP_TCR_TSENALL|PTP_TCR_TSCTRLSSR;
	x2_config_hw_tstamping(priv, control);

	ret = x2_est_init(priv->dev, priv, &priv->plat->est_cfg, priv->dma_cap.estsel, priv->dma_cap.estdep,priv->dma_cap.estwid,priv->plat->est_en, &now);

	if (ret) {
		priv->est_enabled = false;

	 } else
		 priv->est_enabled = true;

#if 0
	printk("%s, reg-0xb00:0x%x\n",__func__,readl(priv->ioaddr + 0xb00));
	printk("%s, reg-0xb04:0x%x\n",__func__,readl(priv->ioaddr + 0xb04));
	printk("%s, reg-0xb08:%d\n",__func__,readl(priv->ioaddr + 0xb08));
	printk("%s, reg-0xb0c:%d\n",__func__,readl(priv->ioaddr + 0xb0c));
	printk("%s, reg-0xb10:0x%x\n",__func__,readl(priv->ioaddr + 0xb10));
	printk("%s, reg-0xb14:0x%x\n",__func__,readl(priv->ioaddr + 0xb14));
	printk("%s, reg-0xb18:0x%x\n",__func__,readl(priv->ioaddr + 0xb18));
	printk("%s, reg-0xb1c:0x%x\n",__func__,readl(priv->ioaddr + 0xb1c));
	printk("%s, reg-0xb20:0x%x\n",__func__,readl(priv->ioaddr + 0xb20));
	printk("%s, reg-0xb30:0x%x\n",__func__,readl(priv->ioaddr + 0xb30));
	printk("%s, reg-0xb34:0x%x\n",__func__,readl(priv->ioaddr + 0xb34));


#endif
	return ret;

}

static int x2_pps_init(struct net_device *ndev, struct x2_priv *priv, int index, struct stmmac_pps_cfg *cfg)
{
	void __iomem *ioaddr = priv->ioaddr;
	u32 value;

	if ((cfg->ctrl_cmd & ~PPSCTRL_PPSCMD) != 0)
		return -EINVAL;

	if ((cfg->trgtmodsel & ~(TRGTMODSEL0 >> 5)) != 0)
		return -EINVAL;

	if ((cfg->target_time[0] & ~(TTSL0)) != 0)
		return -EINVAL;

	if (!cfg->enable && index)
		return -EINVAL;

	value = readl(ioaddr + MAC_PPS_CONTROL);
	value &= ~GENMASK(((index + 1) * 8) - 1, index * 8);
	value |= cfg->trgtmodsel << ((index * 8) + 5);
	if (index == 0)
		value |= cfg->enable << 4;

	writel(value, ioaddr + MAC_PPS_CONTROL);
	writel(cfg->target_time[1], ioaddr + MAC_PPSx_TARGET_TIME_SEC(index));
	if (readl(ioaddr + MAC_PPSx_TARGET_TIME_NSEC(index)) & TRGTBUSY0)
		return -EBUSY;

	writel(cfg->target_time[0], ioaddr + MAC_PPSx_TARGET_TIME_NSEC(index));
	writel(cfg->interval, ioaddr + MAC_PPSx_INTERVAL(index));
	writel(cfg->width, ioaddr + MAC_PPSx_WIDTH(index));

	value |= cfg->ctrl_cmd << (index * 8);
	writel(value, ioaddr + MAC_PPS_CONTROL);

//	printk("Enableing %s PPS for output %d\n", cfg->enable ? "Flexible": "Fixed", index);
	return 0;
}




static int x2_pps_configuration(struct x2_priv *priv, int index)
{
	struct stmmac_pps_cfg *cfg;
	int ret = -EINVAL;

	if (index >= priv->dma_cap.pps_out_num)
		return -EINVAL;

	cfg = &priv->plat->pps_cfg[index];
	ret = x2_pps_init(priv->dev, priv, index, cfg);
	return ret;
}





static void x2_tsn_fp_configure(struct x2_priv *priv)
{
	u32 control;
	u32 value;

	printk("%s\n",__func__);

	writel(0x10001000, priv->ioaddr + MTL_FPE_Advance);

	value = readl(priv->ioaddr + GMAC_INT_EN);
	value |= (1 << GMAC_INT_FPEIE_EN);
	writel(value, priv->ioaddr + GMAC_INT_EN);

	control = readl(priv->ioaddr + GMAC_Ext_CONFIG);
	control &= ~( 1<<16);
	writel(control, priv->ioaddr + GMAC_Ext_CONFIG);


	value = 1;
	value |= (3 << 8);
	writel(value, priv->ioaddr + 0xc90);


	value = readl(priv->ioaddr + 0xa4);
	value |= (1 << 24);
	writel(value, priv->ioaddr + 0xa4);


	control = readl(priv->ioaddr + GMAC_FPE_CTRL_STS);
	control |= GMAC_FPE_EFPE;
//	control |= 0xffffffff;
	writel(control, priv->ioaddr + GMAC_FPE_CTRL_STS);
//	printk("%s, and FPE CTRL STS:0x%x\n", __func__, readl(priv->ioaddr + GMAC_FPE_CTRL_STS));


}


int x2_tsn_capable(struct net_device *ndev)
{
	struct x2_priv *priv = netdev_priv(ndev);
	return priv->tsn_ready == 1;
}


int x2_tsn_link_configure(struct net_device *ndev, enum sr_class class, u16 framesize, u16 vid, u8 add_link, u8 pcp_hi, u8 pcp_lo)
{
	struct x2_priv *priv = netdev_priv(ndev);
	u32 mode_to_use;
	u32 port_rate;
	u32 queue;
	int err;
	s32 bw;

//	printk("%s,and into here\n", __func__);

	if (!x2_tsn_capable(ndev)) {
		printk("%s: NIC not capable\n",__func__);
		return -EINVAL;
	}

	if (framesize > MAX_FRAME_SIZE || framesize < MIN_FRAME_SIZE) {
		printk("%s: framesize (%u) must be [%d,%d]\n",__func__, framesize, MIN_FRAME_SIZE, MAX_FRAME_SIZE);
		return -EINVAL;
	}

	if (add_link && !priv->tsn_vlan_added) {
		rtnl_lock();
		printk("%s: adding VLAN %u to HW filter on device:%s\n",__func__,vid, ndev->name);

		err = vlan_vid_add(ndev, htons(ETH_P_8021Q),vid);
		if (err != 0)
			printk("%s: error adding vlan %u, res=%d\n",__func__,vid, err);
		rtnl_unlock();

		priv->pcp_hi = pcp_hi & 0x7;
		priv->pcp_lo = pcp_lo & 0x7;
		priv->tsn_vlan_added = 1;

	}

	port_rate = PORT_RATE_SGMII;

	switch (class) {
	case SR_CLASS_A:
		queue = AVB_CLASSA_Q;
		bw = AVB_CLASSA_BW;
		break;
	case SR_CLASS_B:
		queue = AVB_CLASSB_Q;
		bw = AVB_CLASSB_BW;
		break;
	default:
		printk("%s: x2 tsn unkown traffic-class, aborting config\n",__func__);
		return -EINVAL;
	}

	mode_to_use = priv->plat->tx_queues_cfg[queue].mode_to_use;
	if (mode_to_use != MTL_QUEUE_AVB) {
		printk("%s: x2 tsn :queue %d is no AVB /TSN\n",__func__,queue);
		return -EINVAL;
	}

	//if (priv->dma_cap.tsn_enh_sched_traffic)
	//	x2_tsn_est_configure(priv);

	//if (priv->dma_cap.tsn_frame_preemption)
	if (priv->dma_cap.fpesel && priv->plat->fp_en)
		x2_tsn_fp_configure(priv);

	return 0;
}


u16 x2_tsn_select_queue(struct net_device *ndev, struct sk_buff *skb, void *accel_priv, select_queue_fallback_t fallback)
{
	struct x2_priv *priv = netdev_priv(ndev);
	if (!priv)
		return fallback(ndev, skb);

//	dump_stack();
	if (x2_tsn_capable(ndev)) {
		switch (vlan_get_protocol(skb)) {
		case htons(ETH_P_TSN):
		//	printk("%s,and skb_prio:%d, pcp_hi:%d,pcp_lo:%d\n",__func__,skb->priority, priv->pcp_hi,priv->pcp_lo);

			if (skb->priority == 0x3)
				return AVB_CLASSA_Q;

			if (skb->priority == 0x2)
				return AVB_CLASSB_Q;

		#if 0
			if (skb->priority == priv->pcp_hi)
				return AVB_CLASSA_Q;
			if (skb->priority == priv->pcp_lo)
				return AVB_CLASSB_Q;
			if (skb->priority == 0x1)
				return AVB_PTPCP_Q;

		#endif
			return AVB_BEST_EFF_Q;
		case htons(ETH_P_1588):
			return AVB_PTPCP_Q;
		default:
			return AVB_BEST_EFF_Q;
		}
	}

	return fallback(ndev, skb);
}




static void x2_set_speed(struct x2_priv *priv)
{
	struct net_device *ndev = priv->dev;
	struct phy_device *phydev = ndev->phydev;
	u32 regval;
	int target = 0;
	int rate_L1 = 0;
	int rate_L2 = 0;

	regval = x2_reg_read(priv, REG_DWCEQOS_MAC_CFG);
	regval &= ~(DWCEQOS_MAC_CFG_PS | DWCEQOS_MAC_CFG_FES | DWCEQOS_MAC_CFG_DM);

	//printk("%s: duplex: %d,speed: %d\n",__func__,phydev->duplex, phydev->speed);
	if (phydev->duplex)
		regval |= DWCEQOS_MAC_CFG_DM;

	if (phydev->speed == SPEED_10) {
		regval |= DWCEQOS_MAC_CFG_PS;
	} else if (phydev->speed == SPEED_100) {
		regval |= DWCEQOS_MAC_CFG_PS| DWCEQOS_MAC_CFG_FES;
	} else if (phydev->speed != SPEED_1000) {
		printk("Unknown PHY speed %d\n",phydev->speed);
		return;
	}

	if (phydev->speed == SPEED_10) {
		target = 12500000;
		rate_L1 = clk_round_rate(priv->plat->x2_mac_pre_div_clk, target);
		clk_set_rate(priv->plat->x2_mac_pre_div_clk, rate_L1);

		target = 2500000;
		rate_L2 = clk_round_rate(priv->plat->x2_mac_div_clk, target);
		clk_set_rate(priv->plat->x2_mac_div_clk, rate_L2);
	} else if (phydev->speed == SPEED_100) {
		target = 125000000;
		rate_L1 = clk_round_rate(priv->plat->x2_mac_pre_div_clk, target);
		clk_set_rate(priv->plat->x2_mac_pre_div_clk, rate_L1);

		target = 25000000;
		rate_L2 = clk_round_rate(priv->plat->x2_mac_div_clk, target);
		clk_set_rate(priv->plat->x2_mac_div_clk, rate_L2);

	} else if (phydev->speed == SPEED_1000) {

		target = 125000000;
		rate_L1 = clk_round_rate(priv->plat->x2_mac_pre_div_clk, target);
		clk_set_rate(priv->plat->x2_mac_pre_div_clk, rate_L1);

		target = 125000000;
		rate_L2 = clk_round_rate(priv->plat->x2_mac_div_clk, target);
		clk_set_rate(priv->plat->x2_mac_div_clk, rate_L2);
	}

	x2_reg_write(priv, REG_DWCEQOS_MAC_CFG, regval);

//	printk("%s, priv_speed;%d, phydev_speed:%d\n",__func__,priv->speed, phydev->speed);
}

static void x2_set_rx_flow_ctrl(struct x2_priv *priv, bool enable)
{
	u32 regval;

	regval = x2_reg_read(priv, REG_DWCEQOS_MAC_RX_FLOW_CTRL);
	if (enable)
		regval |= DWCEQOS_MAC_RX_FLOW_CTRL_RFE;
	else
		regval &= ~DWCEQOS_MAC_RX_FLOW_CTRL_RFE;


	x2_reg_write(priv, REG_DWCEQOS_MAC_RX_FLOW_CTRL, regval);

}


static void x2_set_tx_flow_ctrl(struct x2_priv *priv, bool enable)
{
	u32 regval;

	regval = x2_reg_read(priv, REG_DWCEQOS_MTL_RXQ0_OPER);
	if (enable)
		regval |= DWCEQOS_MTL_RXQ_EHFC;
	else
		regval &= ~DWCEQOS_MTL_RXQ_EHFC;

	x2_reg_write(priv, REG_DWCEQOS_MTL_RXQ0_OPER, regval);
}

static void x2_link_up(struct x2_priv *priv)
{
	//struct net_device *ndev = priv->dev;
	u32 regval;
	//int phy_value = 0;

	regval = x2_reg_read(priv, REG_DWCEQOS_MAC_LPI_CTRL_STATUS);
	regval |= DWCEQOS_MAC_LPI_CTRL_STATUS_PLS;
	x2_reg_write(priv, REG_DWCEQOS_MAC_LPI_CTRL_STATUS, regval);

#if 0
	x2_mdio_write(priv->mii,0, 22,2);
	phy_value = x2_mdio_read(priv->mii,0,21);
	printk("%s, and phy_value:0x%x\n",__func__, phy_value);
	phy_value |= (1 << 4);
	x2_mdio_write(priv->mii,0, 21,phy_value);

	printk("%s, and phy_reg21:0x%x\n",__func__, x2_mdio_read(priv->mii,0,21));
	x2_mdio_write(priv->mii,0, 22,0);
	printk("%s, and phy_id:0x%x\n",__func__, x2_mdio_read(priv->mii, 0,2));
	printk("%s, and phy_id:0x%x\n",__func__, x2_mdio_read(priv->mii, 0,1));
	printk("%s, and phy_id:0x%x\n",__func__, x2_mdio_read(priv->mii, 0,0));
#endif
}


static void x2_link_down(struct x2_priv *priv)
{
	u32 regval;

	regval = x2_reg_read(priv, REG_DWCEQOS_MAC_LPI_CTRL_STATUS);
	regval &= ~DWCEQOS_MAC_LPI_CTRL_STATUS_PLS;
	x2_reg_write(priv, REG_DWCEQOS_MAC_LPI_CTRL_STATUS, regval);
}

static void x2_adjust_link(struct net_device *ndev)
{
	struct x2_priv *priv = netdev_priv(ndev);
	struct phy_device *phydev = ndev->phydev;
	int status_change = 0;

//	printk("%s\n",__func__);
	//printk("%s,and phy reg0:0x%x\n", __func__, x2_mdio_read(priv->mii,3,0));
	//printk("%s,and phy reg1:0x%x\n", __func__, x2_mdio_read(priv->mii,3,1));


	if (phydev->link) {
		if ((priv->speed != phydev->speed) || (priv->duplex != phydev->duplex)) {
			x2_set_speed(priv);

			priv->speed = phydev->speed;
			priv->duplex = phydev->duplex;
			status_change = 1;
			//printk("%s, priv_speed;%d, phydev_speed:%d\n",__func__,priv->speed, phydev->speed);
		}

		if (priv->pause) {
			priv->flow_ctrl_rx = phydev->pause || phydev->asym_pause;
			priv->flow_ctrl_tx = phydev->pause || phydev->asym_pause;
		}

		if (priv->flow_ctrl_rx != priv->flow_current_rx) {
			if (priv->dma_cap.av) {
				priv->flow_ctrl_rx = false;

			}

				x2_set_rx_flow_ctrl(priv, priv->flow_ctrl_rx);

				priv->flow_current_rx = priv->flow_ctrl_rx;

		}

		if (priv->flow_ctrl_tx != priv->flow_current_tx) {
			if (priv->dma_cap.av) {
				priv->flow_ctrl_tx = false;
			}
			x2_set_tx_flow_ctrl(priv, priv->flow_ctrl_tx);
			priv->flow_current_tx = priv->flow_ctrl_tx;
		}
	}


	if (phydev->link != priv->link) {
		priv->link = phydev->link;
		status_change = 1;
	}

	if (status_change) {
		if (phydev->link) {
			netif_trans_update(priv->dev);
			x2_link_up(priv);
		} else {
			x2_link_down(priv);
		}


		printk("speed:%s, duplex:%s\n",phy_speed_to_str(phydev->speed),phy_duplex_to_str(phydev->duplex));
		phy_print_status(phydev);
	}
}
static int x2_init_phy(struct net_device *ndev)
{
	struct x2_priv *priv = netdev_priv(ndev);
	u32 tx_cnt = priv->plat->tx_queues_to_use;
	struct phy_device *phydev;
	int interface = priv->plat->interface;

	priv->link = false;
	priv->speed = SPEED_UNKNOWN;
	priv->duplex = DUPLEX_UNKNOWN;


	if (priv->plat->phy_node) {
		phydev = of_phy_connect(ndev, priv->plat->phy_node, &x2_adjust_link, 0, interface);

		if (!phydev) {
			printk("no phy founded\n");
			return -1;
		}
	} else {
		printk("No PHY configured\n");
		return -ENODEV;
	}

	phy_init_hw(phydev);
	phy_attached_info(phydev);

	phydev->supported &= PHY_GBIT_FEATURES | SUPPORTED_Pause | SUPPORTED_Asym_Pause;

	if (tx_cnt > 1) {
		phydev->supported &= ~(SUPPORTED_1000baseT_Half | SUPPORTED_100baseT_Half|SUPPORTED_10baseT_Half);
		if (priv->dma_cap.tsn) {
			phydev->advertising &= ~(ADVERTISED_Pause | ADVERTISED_Asym_Pause);
		}
	}
	priv->pause = AUTONEG_ENABLE;


	return 0;
}

static void x2_free_rx_buffer(struct x2_priv *priv, u32 queue, int i)
{
	struct x2_rx_queue *rx_q  = &priv->rx_queue[queue];

	if (rx_q->rx_skbuff[i]) {
		dma_unmap_single(priv->device, rx_q->rx_skbuff_dma[i], priv->dma_buf_sz, DMA_FROM_DEVICE);
		dev_kfree_skb_any(rx_q->rx_skbuff[i]);
	}

	rx_q->rx_skbuff[i] = NULL;

}
static void dma_free_rx_skbufs(struct x2_priv *priv, u32 queue)
{
	int i;

	for (i = 0; i < DMA_RX_SIZE; i++)
		x2_free_rx_buffer(priv, queue, i);
}

static void free_dma_rx_desc_resources(struct x2_priv *priv)
{
	u32 rx_count = priv->plat->rx_queues_to_use;
	u32 queue;

	for (queue = 0; queue < rx_count; queue++) {
		struct x2_rx_queue *rx_q = &priv->rx_queue[queue];

		dma_free_rx_skbufs(priv,queue);

		if (!priv->extend_desc)
			dma_free_coherent(priv->device, DMA_RX_SIZE * sizeof(struct dma_desc), rx_q->dma_rx, rx_q->dma_rx_phy);
		else
			dma_free_coherent(priv->device, DMA_RX_SIZE * sizeof(struct dma_desc), rx_q->dma_erx, rx_q->dma_rx_phy);

		kfree(rx_q->rx_skbuff_dma);
		kfree(rx_q->rx_skbuff);
	}
}

static int alloc_dma_rx_desc_resources(struct x2_priv *priv)
{
	u32 rx_count = priv->plat->rx_queues_to_use;
	int ret = -ENOMEM;
	u32 queue;

	for (queue = 0; queue < rx_count; queue++) {
		struct x2_rx_queue *rx_q = &priv->rx_queue[queue];

		rx_q->queue_index = queue;
		rx_q->priv_data = priv;
		rx_q->rx_skbuff_dma = kmalloc_array(DMA_RX_SIZE, sizeof(dma_addr_t), GFP_KERNEL);
		if (!rx_q->rx_skbuff_dma)
			goto err_dma;

		rx_q->rx_skbuff = kmalloc_array(DMA_RX_SIZE, sizeof(struct sk_buff*), GFP_KERNEL);
		if (!rx_q->rx_skbuff)
			goto err_dma;

		if (priv->extend_desc) {
			rx_q->dma_erx = dma_zalloc_coherent(priv->device, DMA_RX_SIZE * sizeof(struct dma_ext_desc),&rx_q->dma_rx_phy, GFP_KERNEL);
			if (!rx_q->dma_erx)
				goto err_dma;
		} else {
			rx_q->dma_rx = dma_zalloc_coherent(priv->device, DMA_RX_SIZE * sizeof(struct dma_desc), &rx_q->dma_rx_phy, GFP_KERNEL);
			if (!rx_q->dma_rx)
				goto err_dma;
		}
	}

	return 0;
err_dma:
	free_dma_rx_desc_resources(priv);
	return ret;
}


static void x2_free_tx_buffer(struct x2_priv *priv, u32 queue, int i)
{
	struct x2_tx_queue *tx_q  = &priv->tx_queue[queue];

	if (tx_q->tx_skbuff_dma[i].buf) {
		if (tx_q->tx_skbuff_dma[i].map_as_page)
			dma_unmap_page(priv->device, tx_q->tx_skbuff_dma[i].buf, tx_q->tx_skbuff_dma[i].len, DMA_TO_DEVICE);
		else
			dma_unmap_single(priv->device, tx_q->tx_skbuff_dma[i].buf, tx_q->tx_skbuff_dma[i].len, DMA_TO_DEVICE);
	}

	if (tx_q->tx_skbuff[i]) {
		dev_kfree_skb_any(tx_q->tx_skbuff[i]);
		tx_q->tx_skbuff[i] = NULL;
		tx_q->tx_skbuff_dma[i].buf = 0;
		tx_q->tx_skbuff_dma[i].map_as_page = false;
	}
}
static void dma_free_tx_skbufs(struct x2_priv *priv, u32 queue)
{
	int i;

	for (i = 0; i < DMA_TX_SIZE; i++) {
		x2_free_tx_buffer(priv, queue, i);
	}
}
static void free_dma_tx_desc_resources(struct x2_priv *priv)
{
	u32 tx_count = priv->plat->tx_queues_to_use;
	u32 queue;

	for (queue = 0; queue < tx_count; queue++) {
		struct x2_tx_queue *tx_q  = &priv->tx_queue[queue];

		dma_free_tx_skbufs(priv, queue);

		if (!priv->extend_desc)
			dma_free_coherent(priv->device, DMA_TX_SIZE * sizeof(struct dma_desc), tx_q->dma_tx, tx_q->dma_tx_phy);
		else
			dma_free_coherent(priv->device, DMA_TX_SIZE * sizeof(struct dma_ext_desc), tx_q->dma_etx, tx_q->dma_tx_phy);
		kfree(tx_q->tx_skbuff_dma);
		kfree(tx_q->tx_skbuff);
	}
}


static int alloc_dma_tx_desc_resources(struct x2_priv *priv)
{
	u32 tx_count = priv->plat->tx_queues_to_use;
	int ret = -ENOMEM;
	u32 queue;

	for (queue = 0; queue < tx_count; queue++) {
		struct x2_tx_queue *tx_q = &priv->tx_queue[queue];

		tx_q->queue_index = queue;
		tx_q->priv_data = priv;

		tx_q->tx_skbuff_dma = kmalloc_array(DMA_TX_SIZE, sizeof(*tx_q->tx_skbuff_dma), GFP_KERNEL);
		if (!tx_q->tx_skbuff_dma)
			goto err_dma;

		tx_q->tx_skbuff = kmalloc_array(DMA_TX_SIZE, sizeof(struct sk_buff *), GFP_KERNEL);
		if (!tx_q->tx_skbuff)
			goto err_dma;

		if (priv->extend_desc) {
			tx_q->dma_etx = dma_zalloc_coherent(priv->device, DMA_TX_SIZE * sizeof(struct dma_ext_desc),&tx_q->dma_tx_phy,GFP_KERNEL);
			if (!tx_q->dma_etx)
				goto err_dma;
		} else {
			tx_q->dma_tx = dma_zalloc_coherent(priv->device, DMA_TX_SIZE * sizeof(struct dma_desc),&tx_q->dma_tx_phy,GFP_KERNEL);

			if (!tx_q->dma_tx)
				goto err_dma;
		}

	}


	return 0;

err_dma:
	free_dma_tx_desc_resources(priv);

	return ret;
}
static int alloc_dma_desc_resources(struct x2_priv *priv)
{
	int ret;
	ret = alloc_dma_rx_desc_resources(priv);
	if (ret)
		return ret;

	ret = alloc_dma_tx_desc_resources(priv);

	return ret;
}

static int x2_init_rx_buffers(struct x2_priv *priv, struct dma_desc *p, int i, u32 queue)
{
	struct x2_rx_queue *rx_q = &priv->rx_queue[queue];
	struct sk_buff *skb;


	skb = __netdev_alloc_skb_ip_align(priv->dev, priv->dma_buf_sz, GFP_KERNEL);
	if (!skb) {
		printk("%s, Rx init failed: skb is NULL\n",__func__);
		return -ENOMEM;
	}

	rx_q->rx_skbuff[i] = skb;
	rx_q->rx_skbuff_dma[i] = dma_map_single(priv->device, skb->data, priv->dma_buf_sz, DMA_FROM_DEVICE);
	if (dma_mapping_error(priv->device, rx_q->rx_skbuff_dma[i])) {
		printk("%s, DMA mapping error\n",__func__);
		dev_kfree_skb_any(skb);
		return -EINVAL;
	}

	p->des0 = cpu_to_le32(rx_q->rx_skbuff_dma[i]);

	return 0;
}

static void x2_clear_rx_descriptors(struct x2_priv *priv, u32 queue)
{
	struct x2_rx_queue *rx_q = &priv->rx_queue[queue];
	int i;
	struct dma_desc *p;

	for (i = 0; i < DMA_RX_SIZE; i++) {

		if (priv->extend_desc) {
			p = &rx_q->dma_erx[i].basic;

		} else {

			p = &rx_q->dma_rx[i];
			p->des3 = cpu_to_le32(RDES3_OWN | RDES3_BUFFER1_VALID_ADDR);
			p->des3 |= cpu_to_le32(RDES3_INT_ON_COMPLETION_EN);
		}
	}

}

static void x2_clear_tx_descriptors(struct x2_priv *priv, u32 queue)
{
	struct x2_tx_queue *tx_q = &priv->tx_queue[queue];
	int i;
	struct dma_desc *p;

	for (i = 0; i < DMA_TX_SIZE; i++) {

		if (priv->extend_desc) {
			p = &tx_q->dma_etx[i].basic;

			p->des0 = 0;
			p->des1 = 0;
			p->des2 = 0;
			p->des3 = 0;

		} else {
			p = &tx_q->dma_tx[i];
			p->des0 = 0;
			p->des1 = 0;
			p->des2 = 0;
			p->des3 = 0;
		}
	}
}
static int init_dma_rx_desc_rings(struct net_device *ndev)
{
	struct x2_priv *priv = netdev_priv(ndev);
	u32 rx_count = priv->plat->rx_queues_to_use;
	u32 queue;
	int ret = -ENOMEM;
	int i;
	priv->dma_buf_sz = DEFAULT_BUFSIZE;

	for (queue = 0; queue < rx_count; queue++) {
		struct x2_rx_queue *rx_q = &priv->rx_queue[queue];

		for (i = 0; i < DMA_RX_SIZE; i++) {
			struct dma_desc *p;

			if (priv->extend_desc)
				p = &((rx_q->dma_erx + i)->basic);
			else
				p = rx_q->dma_rx + i;

			ret = x2_init_rx_buffers(priv, p, i, queue);
			if (ret)
				goto err_init_rx_buffers;


		}

		rx_q->cur_rx = 0;
		rx_q->dirty_rx = (unsigned int)(i - DMA_RX_SIZE);

		x2_clear_rx_descriptors(priv, queue);


	}

	return 0;

err_init_rx_buffers:
	while (queue >= 0) {
		while (--i >= 0)
			x2_free_rx_buffer(priv, queue, i);

		if (queue == 0)
			break;

		i = DMA_RX_SIZE;
		queue--;
	}

	return ret;
}

static int init_dma_tx_desc_rings(struct net_device *ndev)
{
	struct x2_priv *priv = netdev_priv(ndev);
	u32 tx_count = priv->plat->tx_queues_to_use;
	u32 queue;
	int i;

	for (queue = 0; queue < tx_count; queue++) {
		struct x2_tx_queue *tx_q = &priv->tx_queue[queue];

		for (i = 0; i < DMA_TX_SIZE; i++) {
			struct dma_desc *p;

			if (priv->extend_desc)
				p = &((tx_q->dma_etx + i)->basic);
			else
				p = tx_q->dma_tx + i;

			p->des0 = 0;
			p->des1 = 0;
			p->des2 = 0;
			p->des3 = 0;

			tx_q->tx_skbuff_dma[i].buf = 0;
			tx_q->tx_skbuff_dma[i].map_as_page = false;
			tx_q->tx_skbuff_dma[i].len = 0;
			tx_q->tx_skbuff_dma[i].last_segment = false;
			tx_q->tx_skbuff[i] = NULL;
		}

		tx_q->dirty_tx = 0;
		tx_q->cur_tx = 0;
		netdev_tx_reset_queue(netdev_get_tx_queue(priv->dev,queue));
	}

	return 0;
}


static void x2_clear_descriptors(struct x2_priv *priv)
{
	u32 rx_queue_count = priv->plat->rx_queues_to_use;
	u32 tx_queue_count = priv->plat->tx_queues_to_use;
	u32 queue;

	for (queue = 0; queue < rx_queue_count; queue++)
		x2_clear_rx_descriptors(priv, queue);

	for (queue = 0; queue < tx_queue_count; queue++)
		x2_clear_tx_descriptors(priv, queue);
}
static int init_dma_desc_rings(struct net_device *ndev)
{
	struct x2_priv *priv = netdev_priv(ndev);
	int ret;

	ret = init_dma_rx_desc_rings(ndev);
	if (ret)
		return ret;

	ret = init_dma_tx_desc_rings(ndev);

	x2_clear_descriptors(priv);

	return ret;
}
static int x2_dma_reset(void __iomem *ioaddr)
{
	u32 value = readl(ioaddr + DMA_BUS_MODE);
	int limit;

	value |= DMA_BUS_MODE_SFT_RESET;
	limit = 10;
	while (limit--) {
		if (!(readl(ioaddr + DMA_BUS_MODE) & DMA_BUS_MODE_SFT_RESET))
			break;

		mdelay(10);
	}
	if (limit < 0)
		return -EBUSY;

	return 0;


}
static void x2_dma_init(void __iomem *ioaddr, struct x2_dma_cfg *dma_cfg, u32 dma_tx, u32 dma_rx, int atds)
{
	u32 value = readl(ioaddr + DMA_SYS_BUS_MODE);

	if (dma_cfg->fixed_burst)
		value |= DMA_SYS_BUS_FB;
	if (dma_cfg->mixed_burst)
		value |= DMA_SYS_BUS_MB;
	if (dma_cfg->aal)
		value |= DMA_SYS_BUS_AAL;

	writel(value, ioaddr + DMA_SYS_BUS_MODE);
}

static void x2_init_rx_chan(void __iomem *ioaddr, struct x2_dma_cfg *dma_cfg, u32 dma_rx_phy, u32 chan)
{
	u32 value;
	u32 rxpl = dma_cfg->rxpbl ?: dma_cfg->pbl;

	value = readl(ioaddr + DMA_CHAN_RX_CONTROL(chan));
	value = value | (rxpl << DMA_BUS_MODE_RPBL_SHIFT);
	writel(value, ioaddr + DMA_CHAN_RX_CONTROL(chan));

	writel(dma_rx_phy, ioaddr + DMA_CHAN_RX_BASE_ADDR(chan));
}

static void x2_set_rx_tail_ptr(void __iomem *ioaddr, u32 tail_ptr, u32 chan)
{
	writel(tail_ptr, ioaddr + DMA_CHAN_RX_END_ADDR(chan));
}
static void x2_set_tx_tail_ptr(void __iomem *ioaddr, u32 tail_ptr, u32 chan)
{
	writel(tail_ptr, ioaddr + DMA_CHAN_TX_END_ADDR(chan));
}

static void x2_init_chan(void __iomem *ioaddr, struct x2_dma_cfg *dma_cfg, u32 chan)
{
	u32 value;

	value = readl(ioaddr + DMA_CHAN_CONTROL(chan));
	if (dma_cfg->pblx8)
		value |= value | DMA_BUS_MODE_PBL;

	writel(value, ioaddr + DMA_CHAN_CONTROL(chan));

	writel(DMA_CHAN_INTR_DEFAULT_MASK, ioaddr + DMA_CHAN_INTR_ENA(chan));

}


static void x2_init_tx_chan(void __iomem *ioaddr, struct x2_dma_cfg *dma_cfg, u32 dma_tx_phy, u32 chan)
{
	u32 value;

	u32 txpbl = dma_cfg->txpbl ?: dma_cfg->pbl;

	value = readl(ioaddr + DMA_CHAN_TX_CONTROL(chan));
	value = value | (txpbl << DMA_BUS_MODE_PBL_SHIFT);
	writel(value, ioaddr + DMA_CHAN_TX_CONTROL(chan));
	writel(dma_tx_phy, ioaddr + DMA_CHAN_TX_BASE_ADDR(chan));

	//printk("%s, chan:%d, txpbl:0x%x, regvalue:0x%x\n", __func__,chan, txpbl, readl(ioaddr + DMA_CHAN_TX_CONTROL(chan)));
}

static void x2_set_dma_axi(void __iomem *ioaddr, struct x2_axi *axi)
{
	u32 value = readl(ioaddr + DMA_SYS_BUS_MODE);
	int i;
	/*later add this code**/

	if (axi->axi_lpi_en)
		value |= DMA_AXI_EN_LPI;

	if (axi->axi_xit_frm)
		value |= DMA_AXI_LPI_XIT_FRM;

	value &= ~DMA_AXI_WR_OSR_LMT;
	value |= (axi->axi_wr_osr_lmt & 0xf) << 24;

	value &= ~DMA_AXI_RD_OSR_LMT;
	value |= (axi->axi_rd_osr_lmt & 0xf) << 16;

	for (i = 0; i < AXI_BLEN; i++) {
		switch (axi->axi_blen[i]) {
		case 256:
			value |= DMA_AXI_BLEN256;break;
		case 128:
			value |= DMA_AXI_BLEN128;break;
		case 64:
			value |= DMA_AXI_BLEN64;break;
		case 32:
			value |= DMA_AXI_BLEN32;break;
		case 16:
			value |= DMA_AXI_BLEN16;break;
		case 8:
			value |= DMA_AXI_BLEN8;break;
		case 4:
			value |= DMA_AXI_BLEN4;break;
		}
	}


//	value |= DMA_AXI_BLEN128| DMA_AXI_BLEN64| DMA_AXI_BLEN32| DMA_AXI_BLEN16 |DMA_AXI_BLEN8 |DMA_AXI_BLEN4| DMA_AXI_BLEN256;
	writel(value, ioaddr + DMA_SYS_BUS_MODE);

}
static int x2_init_dma_engine(struct x2_priv *priv)
{
	int ret;
	u32 rx_channel_count = priv->plat->rx_queues_to_use;
	u32 tx_channel_count = priv->plat->tx_queues_to_use;
	struct x2_rx_queue *rx_q;
	struct x2_tx_queue *tx_q;
	u32 chan = 0;

	ret = x2_dma_reset(priv->ioaddr);
	if (ret) {
		printk("%s: Failed to reset dma\n",__func__);
		return ret;
	}

	x2_dma_init(priv->ioaddr, priv->plat->dma_cfg,0,0,0);

	for (chan = 0; chan < rx_channel_count; chan++) {
		rx_q = &priv->rx_queue[chan];
		x2_init_rx_chan(priv->ioaddr, priv->plat->dma_cfg, rx_q->dma_rx_phy, chan);

		rx_q->rx_tail_addr = rx_q->dma_rx_phy + (DMA_RX_SIZE * sizeof(struct dma_desc));
		x2_set_rx_tail_ptr(priv->ioaddr, rx_q->rx_tail_addr, chan);
	}


	for (chan = 0; chan < tx_channel_count; chan++) {
		tx_q = &priv->tx_queue[chan];

		x2_init_chan(priv->ioaddr, priv->plat->dma_cfg, chan);
		x2_init_tx_chan(priv->ioaddr, priv->plat->dma_cfg, tx_q->dma_tx_phy, chan);
		tx_q->tx_tail_addr = tx_q->dma_tx_phy + (DMA_TX_SIZE *sizeof(struct dma_desc));
		x2_set_tx_tail_ptr(priv->ioaddr, tx_q->tx_tail_addr, chan);
	}

	x2_set_dma_axi(priv->ioaddr, priv->plat->axi);

	return 0;
}

static void x2_set_umac_addr(void __iomem *ioaddr, unsigned char *addr, unsigned int reg_n)
{
	unsigned int high, low;
	unsigned long data;

	high = GMAC_ADDR_HIGH(reg_n);
	low = GMAC_ADDR_LOW(reg_n);

	data = (addr[5] << 8) | addr[4];
	data |= (STMMAC_CHAN0 << GMAC_HI_DCS_SHIFT);
	writel(data|GMAC_HI_REG_AE, ioaddr + high);

	data = (addr[3] << 24) | (addr[2] << 16) | (addr[1] << 8) | addr[0];
	writel(data, ioaddr + low);

}

static void x2_core_init(struct x2_priv *priv, int mtu)
{
	void __iomem *ioaddr = priv->ioaddr;

	u32 value = readl(ioaddr + GMAC_CONFIG);

	value |= GMAC_CORE_INIT;

	if (mtu > 1500)
		value |= GMAC_CONFIG_2K;
	if (mtu > 2000)
		value |= GMAC_CONFIG_JE;

	if (priv->link) {
		value |= GMAC_CONFIG_TE;
		value |= priv->speed;
	}

	writel(value, ioaddr + GMAC_CONFIG);

	value = GMAC_INT_DEFAULT_MASK;
	value |= GMAC_INT_PMT_EN | GMAC_PCS_IRQ_DEFAULT;

	writel(value, ioaddr + GMAC_INT_EN);


}

static void x2_set_tx_queue_weight(struct x2_priv *priv)
{
	u32 tx_count = priv->plat->tx_queues_to_use;
	u32 weight;
	u32 queue;
	u32 value;

	for (queue = 0; queue < tx_count; queue++) {
		weight = priv->plat->tx_queues_cfg[queue].weight;

		value = readl(priv->ioaddr + MTL_TXQX_WEIGHT_BASE_ADDR(queue));
		value &= ~MTL_TXQ_WEIGHT_ISCQW_MASK;
		value |= weight & MTL_TXQ_WEIGHT_ISCQW_MASK;
		writel(value, priv->ioaddr + MTL_TXQX_WEIGHT_BASE_ADDR(queue));
	}
}

static void x2_pro_rx_algo(struct x2_priv *priv, u32 rx_alg)
{
	void __iomem *ioaddr = priv->ioaddr;

	u32 value = readl(ioaddr + MTL_OPERATION_MODE);
	value &= ~MTL_OPERATION_RAA;

	switch (rx_alg) {
	case MTL_RX_ALGORITHM_SP:
		value |= MTL_OPERATION_RAA_SP;
		break;
	case MTL_RX_ALGORITHM_WSP:
		value |= MTL_OPERATION_RAA_WSP;
		break;
	default:
		break;
	}

	writel(value, ioaddr + MTL_OPERATION_MODE);
}

static void x2_pro_tx_algo(struct x2_priv *priv, u32 tx_alg)
{
	void __iomem *ioaddr = priv->ioaddr;
	u32 value = readl(ioaddr + MTL_OPERATION_MODE);

	value &= ~MTL_OPERATION_SCHALG_MASK;
	switch (tx_alg) {
	case MTL_TX_ALGORITHM_WRR:
		value |= MTL_OPERATION_SCHALG_WRR;
		break;
	case MTL_TX_ALGORITHM_WFQ:
		value |= MTL_OPERATION_SCHALG_WFQ;
		break;
	case MTL_TX_ALGORITHM_DWRR:
		value |= MTL_OPERATION_SCHALG_DWRR;
		break;
	case MTL_TX_ALGORITHM_SP:
		value |= MTL_OPERATION_SCHALG_SP;
		break;
	default:
		break;
	}

	writel(value, ioaddr + MTL_OPERATION_MODE);

}

static void x2_dma_config_cbs(struct x2_priv *priv, int send_slope, u32 idle_slope, u32 high_credit, int low_credit, u32 queue)
{
	void __iomem *ioaddr = priv->ioaddr;
	u32 value;
	struct timespec64 now;
	u32 control, sec_inc;
	u64 temp;


	value = readl(ioaddr + MTL_ETSX_CTRL_BASE_ADDR(queue));
	value |= MTL_ETS_CTRL_AVALG;
	value |= MTL_ETS_CTRL_CC;
	writel(value, ioaddr + MTL_ETSX_CTRL_BASE_ADDR(queue));

	value = readl(ioaddr + MTL_SEND_SLP_CREDX_BASE_ADDR(queue));
	value &= ~MTL_SEND_SLP_CRED_SSC_MASK;
	value |= send_slope & MTL_SEND_SLP_CRED_SSC_MASK;
	writel(value, ioaddr + MTL_SEND_SLP_CREDX_BASE_ADDR(queue));

	value = readl(ioaddr + MTL_TXQX_WEIGHT_BASE_ADDR(queue));
	value &= ~MTL_TXQ_WEIGHT_ISCQW_MASK;
	value |= idle_slope & MTL_TXQ_WEIGHT_ISCQW_MASK;
	writel(value ,ioaddr + MTL_TXQX_WEIGHT_BASE_ADDR(queue));


	value = readl(ioaddr + MTL_HIGH_CREDX_BASE_ADDR(queue));
	value &= ~MTL_HIGH_CRED_HC_MASK;
	value |= high_credit & MTL_HIGH_CRED_HC_MASK;
	writel(value, ioaddr + MTL_HIGH_CREDX_BASE_ADDR(queue));

	value = readl(ioaddr + MTL_LOW_CREDX_BASE_ADDR(queue));
	value &= ~MTL_HIGH_CRED_LC_MASK;
	value |= low_credit & MTL_HIGH_CRED_LC_MASK;
	writel(value, ioaddr + MTL_LOW_CREDX_BASE_ADDR(queue));

/*must be open, so can read the average bit per slot*/
#if 1
	priv->hwts_tx_en = 1;
	priv->hwts_rx_en = 1;


	ktime_get_real_ts64(&now);
	sec_inc = x2_config_sub_second_increment(priv, priv->plat->clk_ptp_rate, 0);
	temp = div_u64(1000000000ULL, sec_inc);

	temp = (u64)(temp << 32);
	priv->default_addend = div_u64(temp, priv->plat->clk_ptp_rate);
	x2_config_addend(priv, priv->default_addend);
	x2_init_systime(priv, (u32)now.tv_sec, now.tv_nsec);

	control = PTP_TCR_TSENA | PTP_TCR_TSINIT|PTP_TCR_TSENALL|PTP_TCR_TSCTRLSSR;
	x2_config_hw_tstamping(priv, control);
/*for average bit */
#endif
	value = readl(ioaddr + MTL_ETSX_CTRL_BASE_ADDR(queue));
	value |= (1 << 4);
	writel(value, ioaddr + MTL_ETSX_CTRL_BASE_ADDR(queue));


}

static u32 x2_config_cbs(struct x2_priv *priv)
{
	u32 tx_queues_count = priv->plat->tx_queues_to_use;
	u32 queues_av = 0;
	u32 mode_to_use;
	u32 queue;

	//u32 value;
	//void __iomem *ioaddr = priv->ioaddr;

	/*queue 0 is for legacy triffc*/
	for (queue = 1; queue < tx_queues_count; queue++) {
		mode_to_use = priv->plat->tx_queues_cfg[queue].mode_to_use;

		if (mode_to_use == MTL_QUEUE_DCB)
			continue;
		else
			queues_av++;

		x2_dma_config_cbs(priv, priv->plat->tx_queues_cfg[queue].send_slope, priv->plat->tx_queues_cfg[queue].idle_slope, priv->plat->tx_queues_cfg[queue].high_credit, priv->plat->tx_queues_cfg[queue].low_credit, queue);

	}

	return queues_av;
}

static void x2_rx_queue_dma_chan_map(struct x2_priv *priv)
{
	u32 rx_queues_count = priv->plat->rx_queues_to_use;
	u32 queue;
	u32 chan;
	void __iomem *ioaddr = priv->ioaddr;
	u32 value;

	for (queue = 0; queue < rx_queues_count; queue++) {
		chan = priv->plat->rx_queues_cfg[queue].chan;

		if (queue < 4)
			value = readl(ioaddr + MTL_RXQ_DMA_MAP0);
		else
			value = readl(ioaddr + MTL_RXQ_DMA_MAP1);

		if (queue == 0 || queue == 4) {
			value &= ~MTL_RXQ_DMA_Q04MDMACH_MASK;
			value |= MTL_RXQ_DMA_Q04MDMACH(chan);
		} else {
			value &= ~MTL_RXQ_DMA_QXMDMACH_MASK(queue);
			value |= MTL_RXQ_DMA_QXMDMACH(chan, queue);
		}

		if (queue < 4)
			writel(value, ioaddr + MTL_RXQ_DMA_MAP0);
		else
			writel(value, ioaddr + MTL_RXQ_DMA_MAP1);
	}
}

static void x2_mac_enable_rx_queues(struct x2_priv *priv)
{
	u32 rx_queues_count = priv->plat->rx_queues_to_use;
	int queue;
	u8 mode;
	void __iomem *ioaddr = priv->ioaddr;
	u32 value;

	for (queue = 0; queue < rx_queues_count; queue++) {
		mode = priv->plat->rx_queues_cfg[queue].mode_to_use;
		value = readl(ioaddr + GMAC_RXQ_CTRL0);
		value &= GMAC_RX_QUEUE_CLEAR(queue);
		if (mode == MTL_QUEUE_AVB) //1
			value |= GMAC_RX_AV_QUEUE_ENABLE(queue);
		else if (mode == MTL_QUEUE_DCB)//0
			value |= GMAC_RX_DCB_QUEUE_ENABLE(queue);
//		printk("%s, and queue:%d, value:0x%x, mode:0x%x\n",__func__, queue, value, mode);

		writel(value, ioaddr + GMAC_RXQ_CTRL0);
	//	printk("%s, queue:%d, value:0x%x\n", __func__, queue, readl(ioaddr+GMAC_RXQ_CTRL0));
	}
}

static void x2_mac_config_rx_queues_prio(struct x2_priv *priv)
{
	void __iomem *ioaddr = priv->ioaddr;
	u32 rx_count = priv->plat->rx_queues_to_use;
	u32 queue;
	u32 prio;
	u32 value;
	u32 base_reg;


	for (queue = 0; queue < rx_count; queue++) {
		if (!priv->plat->rx_queues_cfg[queue].use_prio)
			continue;

		prio = priv->plat->rx_queues_cfg[queue].prio;
		base_reg = (queue < 4) ? GMAC_RXQ_CTRL2:GMAC_RXQ_CTRL3;
		value = readl(ioaddr + base_reg);
		value &= ~GMAC_RXQCTRL_PSRQX_MASK(queue);
		value |= (prio << GMAC_RXQCTRL_PSRQX_SHIFT(queue)) & GMAC_RXQCTRL_PSRQX_MASK(queue);
		writel(value, ioaddr + base_reg);
	}
}

static void x2_mac_config_tx_queues_prio(struct x2_priv *priv)
{
	u32 tx_count = priv->plat->tx_queues_to_use;
	u32 queue;
	u32 prio;
	void __iomem *ioaddr = priv->ioaddr;
	u32 base_reg;
	u32 value;

	for (queue = 0; queue < tx_count; queue++) {
		if (!priv->plat->tx_queues_cfg[queue].use_prio)
			continue;

		prio = priv->plat->tx_queues_cfg[queue].prio;

		base_reg = (queue < 4) ? GMAC_TXQ_PRTY_MAP0:GMAC_TXQ_PRTY_MAP1;
		value = readl(ioaddr + base_reg);
		value &= ~GMAC_TXQCTRL_PSTQX_MASK(queue);
		value |= (prio << GMAC_TXQCTRL_PSTQX_SHIFT(queue)) & GMAC_TXQCTRL_PSTQX_MASK(queue);
		writel(value, ioaddr + base_reg);

	}
}


static void x2_mac_config_rx_queues_routing(struct x2_priv *priv)
{
	u32 rx_count = priv->plat->rx_queues_to_use;
	u32 queue;
	u8 packet;
	u32 value;
	void __iomem *ioaddr = priv->ioaddr;

	static const struct x2_rx_routing route_possibilities[] = {
		{ GMAC_RXQCTRL_AVCPQ_MASK, GMAC_RXQCTRL_AVCPQ_SHIFT },
		{ GMAC_RXQCTRL_PTPQ_MASK, GMAC_RXQCTRL_PTPQ_SHIFT },
		{ GMAC_RXQCTRL_DCBCPQ_MASK, GMAC_RXQCTRL_DCBCPQ_SHIFT },
		{ GMAC_RXQCTRL_UPQ_MASK, GMAC_RXQCTRL_UPQ_SHIFT },
		{ GMAC_RXQCTRL_MCBCQ_MASK, GMAC_RXQCTRL_MCBCQ_SHIFT },
	};


	for (queue = 0; queue < rx_count; queue++) {
		if (priv->plat->rx_queues_cfg[queue].pkt_route == 0x0)
			continue;

		packet = priv->plat->rx_queues_cfg[queue].pkt_route;


		value = readl(ioaddr + GMAC_RXQ_CTRL1);
		value &= ~route_possibilities[packet - 1].reg_mask;
		value |= (queue << route_possibilities[packet-1].reg_shift) & route_possibilities[packet - 1].reg_mask;

		if (packet == PACKET_AVCPQ) {
			value &= ~GMAC_RXQCTRL_TACPQE;
			value |= 0x1 << GMAC_RXQCTRL_TACPQE_SHIFT;
		} else if (packet == PACKET_MCBCQ) {
			value &= ~GMAC_RXQCTRL_MCBCQEN;
			value |= 0x1 << GMAC_RXQCTRL_MCBCQEN_SHIFT;
		}
		value |= 1 << 24;
		writel(value, ioaddr + GMAC_RXQ_CTRL1);
	}
}

static void x2_configure_tsn(struct x2_priv *priv)
{
	u32 tx_cnt = priv->plat->tx_queues_to_use;
	u32 queues_av = 0;

	priv->sra_idleslope_res = 0;
	priv->srb_idleslope_res = 0;
	priv->tsn_vlan_added = 0;
	priv->tsn_ready = 0;

	if (!priv->dma_cap.av)
		return;

	if (tx_cnt > 1)
		queues_av = x2_config_cbs(priv);

	if (queues_av < MIN_AVB_QUEUES) {
		printk("Not enuough queues for AVB (only %d)\n",queues_av);
		return ;
	}

	/*disable tx and rx flow control*/

	priv->tsn_ready = 1;
}


static void x2_mtl_config(struct x2_priv *priv)
{
	u32 rx_queues_count = priv->plat->rx_queues_to_use;
	u32 tx_queues_count = priv->plat->tx_queues_to_use;



	if (tx_queues_count > 1) {

		x2_set_tx_queue_weight(priv);//这个是好的
		x2_pro_tx_algo(priv, priv->plat->tx_sched_algorithm);//这个也是好的
	}


	if (rx_queues_count > 1)
		x2_pro_rx_algo(priv, priv->plat->rx_sched_algorithm);


//	if (tx_queues_count > 1)
//		x2_config_cbs(priv);

/*add by dhw*/
	x2_configure_tsn(priv);//也是ok的



	x2_rx_queue_dma_chan_map(priv);

	x2_mac_enable_rx_queues(priv);


	if (rx_queues_count > 1) {
		x2_mac_config_rx_queues_prio(priv);
		x2_mac_config_rx_queues_routing(priv);
	}


	if (tx_queues_count > 1) {
		x2_mac_config_tx_queues_prio(priv);//这个测试有点问题.
	}
}



#if 0
static void hobot_config_cbs(struct x2_priv *priv, int send_slope, u32 idle_slope, u32 high_credit, int low_credit, u32 queue)
{
	u32 value;
	void __iomem *ioaddr = priv->ioaddr;

	value = readl(ioaddr + MTL_ETSX_CTRL_BASE_ADDR(queue));
	value |= MTL_ETS_CTRL_AVALG;
	value |= MTL_ETS_CTRL_CC;


	value = readl(ioaddr + MTL_SEND_SLP_CREDX_BASE_ADDR(queue));
	value &= ~MTL_SEND_SLP_CRED_SSC_MASK;
	value |= send_slop & MTL_SEND_SLP_CRED_SSC_MASK;
	writel(value, ioaddr + MTL_SEND_SLP_CREDX_BASE_ADDR(queue));

	value = readl(ioaddr + MTL_TXQX_WEIGHT_BASE_ADDR(queue));
	value &= ~MTL_TXQ_WEIGHT_ISCQW_MASK;
	value |= idle_slope & MTL_TXQ_WEIGHT_ISCQW_MASK;
	writel(value, ioaddr + MTL_TXQX_WEIGHT_BASE_ADDR(queue));


	value = readl(ioaddr + MTL_HIGH_CREDX_BASE_ADDR(queue));
	value &= ~MTL_HIGH_CRED_HC_MASK;
	value |= high_credit & MTL_HIGH_CRED_HC_MASK;
	writel(value, ioaddr + MTL_HIGH_CREDX_BASE_ADDR(queue));


	value = readl(ioaddr + MTL_LOW_CREDX_BASE_ADDR(queue));
	value &= ~MTL_HIGH_CRED_LC_MASK;
	value |= low_credit & MTL_HIGH_CRED_LC_MASK;
	writel(value, ioaddr + MTL_LOW_CREDX_BASE_ADDR(queue));

}

#endif

static int x2_rx_ipc_enable(struct x2_priv *priv)
{
	void __iomem *ioaddr = priv->ioaddr;
	u32 value = readl(ioaddr + GMAC_CONFIG);
	if (priv->rx_csum)
		value |= GMAC_CONFIG_IPC;
	else
		value &= ~GMAC_CONFIG_IPC;

	value |= (1 << 21); //for RX FCS enable
	writel(value, ioaddr + GMAC_CONFIG);

	value = readl(ioaddr + GMAC_CONFIG);
	return !!(value & GMAC_CONFIG_IPC);

}

static void x2_set_mac(void __iomem *ioaddr, bool enable)
{
	u32 value = readl(ioaddr + MAC_CTRL_REG);

	if (enable)
		value |= MAC_ENABLE_RX | MAC_ENABLE_TX;
	else
		value &= ~(MAC_ENABLE_TX | MAC_ENABLE_RX);
	writel(value , ioaddr + MAC_CTRL_REG);
}

static void x2_dma_rx_chan_op_mode(void __iomem *ioaddr, int mode, u32 chan, int rxfifosz, u8 qmode)
{
	unsigned int rqs = (rxfifosz / 256) - 1;
	u32 mtl_rx_op, mtl_rx_int;

	rqs = 7;

	mtl_rx_op = readl(ioaddr + MTL_CHAN_RX_OP_MODE(chan));
	mtl_rx_op |= MTL_OP_MODE_RSF;
	mtl_rx_op &= ~MTL_OP_MODE_RQS_MASK;
	printk("%s, and rqs:0x%x\n",__func__, rqs);
	mtl_rx_op |= rqs << MTL_OP_MODE_RQS_SHIFT;

	if (rxfifosz >= 4096 && qmode != MTL_QUEUE_AVB ) {
		unsigned int rfd, rfa;
		mtl_rx_op |= MTL_OP_MODE_EHFC;
		switch (rxfifosz) {
			case 4096:
				rfd = 0x03;
				rfa = 0x01;
				break;
			case 8192:
				rfd = 0x06;
				rfa = 0x0a;
				break;
			case 16384:
				rfd = 0x06;
				rfa = 0x12;
				break;
			default:
				rfd = 0x06;
				rfa = 0x1e;
				break;
		}

		mtl_rx_op &= ~MTL_OP_MODE_RFD_MASK;
		mtl_rx_op |= rfd << MTL_OP_MODE_RFD_SHIFT;
		mtl_rx_op &= ~MTL_OP_MODE_RFA_MASK;
		mtl_rx_op |= rfa << MTL_OP_MODE_RFA_SHIFT;
	}

//	printk("%s, adn mtl_rx_op:0x%x\n",__func__, mtl_rx_op);
	writel(mtl_rx_op, ioaddr + MTL_CHAN_RX_OP_MODE(chan));
	mtl_rx_int = readl(ioaddr + MTL_CHAN_INT_CTRL(chan));
	writel(mtl_rx_int | MTL_RX_OVERFLOW_INT_EN, ioaddr + MTL_CHAN_INT_CTRL(chan));


}

static void x2_dma_tx_chan_op_mode(void __iomem *ioaddr, int mode, u32 chan, int fifosz, u8 qmode)
{
	u32 mtl_tx_op = readl(ioaddr + MTL_CHAN_TX_OP_MODE(chan));//0xd00
	unsigned int tqs = fifosz / 256 - 1;

	mtl_tx_op |= MTL_OP_MODE_TSF;
	//mtl_tx_op |= MTL_OP_MODE_TXQEN | MTL_OP_MODE_TQS_MASK;

	mtl_tx_op &= ~MTL_OP_MODE_TXQEN_MASK;
	if (qmode != MTL_QUEUE_AVB)
		mtl_tx_op |= MTL_OP_MODE_TXQEN;
	else
		mtl_tx_op |= MTL_OP_MODE_TXQEN_AV;

	mtl_tx_op &= ~MTL_OP_MODE_TQS_MASK;
	//mtl_tx_op |= tqs << MTL_OP_MODE_TQS_SHIFT;
	mtl_tx_op |= 15 << MTL_OP_MODE_TQS_SHIFT;
	writel(mtl_tx_op, ioaddr + MTL_CHAN_TX_OP_MODE(chan));


}
static void x2_dma_operation_mode(struct x2_priv *priv)
{
	u32 rx_count = priv->plat->rx_queues_to_use;
	u32 tx_count = priv->plat->tx_queues_to_use;
	int rxfifosz = priv->plat->rx_fifo_size;
	int txfifosz = priv->plat->tx_fifo_size;
	//void __iomem *ioaddr = priv->ioaddr;
	//u32 txmode = 0;
	u32 rxmode = 0;
	u32 chan;
	u8 qmode;

	if (rxfifosz == 0)
		rxfifosz = priv->dma_cap.rx_fifo_size;

	if (txfifosz == 0)
		txfifosz = priv->dma_cap.tx_fifo_size;

	rxfifosz /= rx_count;
	txfifosz /= tx_count;


	for (chan = 0; chan < rx_count; chan++) {
		rxmode = SF_DMA_MODE;
		qmode = priv->plat->rx_queues_cfg[chan].mode_to_use;
		x2_dma_rx_chan_op_mode(priv->ioaddr, rxmode, chan, rxfifosz,qmode);

	}


	for (chan = 0; chan < tx_count; chan++) {

		qmode = priv->plat->tx_queues_cfg[chan].mode_to_use;
		x2_dma_tx_chan_op_mode(priv->ioaddr, SF_DMA_MODE, chan, txfifosz, qmode);

	}


}
static void x2_mmc_intr_all_mask(void __iomem *mmcaddr)
{
	writel(MMC_DEFAULT_MASK, mmcaddr + MMC_RX_INTR_MASK);
	writel(MMC_DEFAULT_MASK, mmcaddr + MMC_TX_INTR_MASK);
	writel(MMC_DEFAULT_MASK, mmcaddr + MMC_RX_IPC_INTR_MASK);
}

static void x2_mmc_setup(struct x2_priv *priv)
{
	//unsigned int mode = MMC_CNTRL_RESET_ON_READ | MMC_CNTRL_COUNTER_RESET |MMC_CNTRL_PRESET | MMC_CNTRL_FULL_HALF_PRESET;

	priv->ptpaddr = priv->ioaddr + PTP_GMAC4_OFFSET;
	priv->mmcaddr = priv->ioaddr + MMC_GMAC4_OFFSET;

	x2_mmc_intr_all_mask(priv->mmcaddr);
}

static void x2_start_rx_dma(struct x2_priv *priv, u32 chan)
{
	void __iomem *ioaddr = priv->ioaddr;
	u32 value = readl(ioaddr + DMA_CHAN_RX_CONTROL(chan));

	value |= DMA_CONTROL_SR;

	writel(value, ioaddr + DMA_CHAN_RX_CONTROL(chan));

	value = readl(ioaddr + GMAC_CONFIG);
	value |= GMAC_CONFIG_RE;
	writel(value, ioaddr + GMAC_CONFIG);
}


static void x2_start_tx_dma(struct x2_priv *priv, u32 chan)
{
	void __iomem *ioaddr = priv->ioaddr;
	u32 value = readl(ioaddr + DMA_CHAN_TX_CONTROL(chan));

	value |= DMA_CONTROL_ST;
	writel(value, ioaddr + DMA_CHAN_TX_CONTROL(chan));

	value = readl(ioaddr + GMAC_CONFIG);
	value |= GMAC_CONFIG_TE;
	writel(value, ioaddr + GMAC_CONFIG);

}

static void x2_start_all_dma(struct x2_priv *priv)
{
	u32 rx_count = priv->plat->rx_queues_to_use;
	u32 tx_count = priv->plat->tx_queues_to_use;
	u32 chan = 0;

	for (chan = 0; chan < rx_count; chan++)
		x2_start_rx_dma(priv, chan);

	for (chan = 0; chan < tx_count; chan++)
		x2_start_tx_dma(priv, chan);
}

static void x2_set_rings_length(struct x2_priv *priv)
{
	u32 rx_count = priv->plat->rx_queues_to_use;
	u32 tx_count = priv->plat->tx_queues_to_use;
	u32 chan;
	void __iomem *ioaddr = priv->ioaddr;
	u32 len;

	for (chan = 0; chan < tx_count; chan++){
		len = DMA_TX_SIZE - 1;
		writel(len, ioaddr + DMA_CHAN_TX_RING_LEN(chan));
	}

	for (chan = 0; chan < rx_count; chan++) {
		len = DMA_RX_SIZE - 1;
		writel(len, ioaddr + DMA_CHAN_RX_RING_LEN(chan));
	}

}

static int x2_set_time(struct ptp_clock_info *ptp, const struct timespec64 *ts)
{
	struct x2_priv *priv = container_of(ptp, struct x2_priv, ptp_clock_ops);
	unsigned long flags;

	spin_lock_irqsave(&priv->ptp_lock, flags);

	x2_init_systime(priv, ts->tv_sec, ts->tv_nsec);

	spin_unlock_irqrestore(&priv->ptp_lock, flags);

	return 0;
}

static u64 x2_get_systime(struct x2_priv *priv)
{
	u64 ns;

	ns = readl(priv->ioaddr + PTP_STNSR);
	ns += readl(priv->ioaddr + PTP_STSR) * 1000000000ULL;
	return ns;
}

static int x2_get_time(struct ptp_clock_info *ptp, struct timespec64 *ts)
{
	struct x2_priv *priv = container_of(ptp, struct x2_priv, ptp_clock_ops);
	unsigned long flags;
	u64 ns;

	spin_lock_irqsave(&priv->ptp_lock, flags);

	ns = x2_get_systime(priv);

	spin_unlock_irqrestore(&priv->ptp_lock, flags);

	*ts = ns_to_timespec64(ns);
	return 0;
}

static int x2_adjust_systime(struct x2_priv *priv, u32 sec, u32 nsec, int add_sub, int gmac)
{
	u32 value;
	int limit;

	if (add_sub) {
		sec = (1000000000ULL - sec);

		value = readl(priv->ioaddr + PTP_TCR);
		if (value & PTP_TCR_TSCTRLSSR)
			nsec = (PTP_DIGITAL_ROLLOVER_MODE - nsec);
		else
			nsec = (PTP_BINARY_ROLLOVER_MODE - nsec);


	}


	writel(sec, priv->ioaddr + PTP_STSUR);
	value = (add_sub << PTP_STNSUR_ADDSUB_SHIFT) | nsec;
	writel(value, priv->ioaddr + PTP_STNSUR);

	value = readl(priv->ioaddr + PTP_TCR);
	value |= PTP_TCR_TSUPDT;
	writel(value, priv->ioaddr + PTP_TCR);

	limit = 10;
	while(limit--) {
		if (!(readl(priv->ioaddr + PTP_TCR) & PTP_TCR_TSUPDT))
			break;
		mdelay(10);
	}

	if (limit < 0)
		return -EBUSY;

	return 0;
}


static int x2_adjust_time(struct ptp_clock_info *ptp, s64 delta)
{
	struct x2_priv *priv = container_of(ptp, struct x2_priv, ptp_clock_ops);
	unsigned long flags;
	u32 sec, nsec;
	u32 quotient, reminder;
	int neg_adj = 0;

	if (delta < 0) {
		neg_adj = 1;
		delta = -delta;
	}

	quotient = div_u64_rem(delta, 1000000000ULL, &reminder);
	sec = quotient;
	nsec = reminder;

	spin_lock_irqsave(&priv->ptp_lock, flags);

	x2_adjust_systime(priv, sec, nsec, neg_adj, 1);

	spin_unlock_irqrestore(&priv->ptp_lock, flags);
	return 0;
}

static int x2_flex_pps_config(struct x2_priv *priv, int index, struct x2_pps_cfg *cfg, bool enable, u32 sub_second_inc, u32 systime_flags)
{
	u32 tnsec = readl(priv->ioaddr + MAC_PPSx_TARGET_TIME_NSEC(index));
	u32 val = readl(priv->ioaddr + MAC_PPS_CONTROL);
	u64 period;

	if (!cfg->available)
		return -EINVAL;

	if (tnsec & TRGTBUSY0)
		return -EBUSY;

	if (!sub_second_inc || !systime_flags)
		return -EINVAL;

	val &= ~PPSx_MASK(index);
	if (!enable) {
		val |= PPSCMDx(index, 0x5);
		writel(val, priv->ioaddr + MAC_PPS_CONTROL);
		return 0;
	}

	val |= PPSCMDx(index, 0x2);
	val |= TRGTMODSELx(index, 0x2);
	val |= PPSEN0;
	writel(cfg->start.tv_sec, priv->ioaddr + MAC_PPSx_TARGET_TIME_SEC(index));

	if (!(systime_flags & PTP_TCR_TSCTRLSSR))
		cfg->start.tv_nsec = (cfg->start.tv_nsec * 1000)/465;

	writel(cfg->start.tv_nsec, priv->ioaddr + MAC_PPSx_TARGET_TIME_NSEC(index));

	period = cfg->period.tv_sec * 1000000000;
	period += cfg->period.tv_nsec;
	do_div(period , sub_second_inc);

	if (period <= 1)
		return -EINVAL;

	writel(period - 1, priv->ioaddr + MAC_PPSx_INTERVAL(index));

	period >>= 1;
	if (period <= 1)
		return -EINVAL;

	writel(period - 1, priv->ioaddr + MAC_PPSx_WIDTH(index));
	writel(val, priv->ioaddr + MAC_PPS_CONTROL);
	return 0;
}

static int x2_ptp_enable(struct ptp_clock_info *ptp, struct ptp_clock_request *rq, int on)
{
	struct x2_priv *priv = container_of(ptp, struct x2_priv, ptp_clock_ops);
	struct x2_pps_cfg *cfg;
	int ret = -EOPNOTSUPP;
	unsigned long flags;

#if 1
	switch(rq->type) {
	case PTP_CLK_REQ_PEROUT:
		cfg = &priv->pps[rq->perout.index];
		cfg->start.tv_sec = rq->perout.start.sec;
		cfg->start.tv_nsec = rq->perout.start.nsec;
		cfg->period.tv_sec = rq->perout.period.sec;
		cfg->period.tv_nsec = rq->perout.period.nsec;
		spin_lock_irqsave(&priv->ptp_lock, flags);
		ret = x2_flex_pps_config(priv,rq->perout.index, cfg, on, priv->sub_second_inc, priv->systime_flags);
		spin_unlock_irqrestore(&priv->ptp_lock, flags);
		break;
	default:
		break;
	}

#endif
	return ret;
}


static int x2_adjust_freq(struct ptp_clock_info *ptp, s32 ppb)
{
	struct x2_priv *priv = container_of(ptp, struct x2_priv, ptp_clock_ops);
	unsigned long flags;

	u32 diff, addend;
	int neg_adj = 0;
	u64 adj;

	if (ppb < 0) {
		neg_adj = 1;
		ppb =  -ppb;
	}

	addend = priv->default_addend;
	adj = addend;
	adj *= ppb;

	diff = div_u64(adj, 1000000000ULL);
	addend = neg_adj ? (addend - diff) : (addend + diff);

	spin_lock_irqsave(&priv->ptp_lock, flags);
	x2_config_addend(priv, addend);
	spin_unlock_irqrestore(&priv->ptp_lock, flags);
	return 0;
}

static struct ptp_clock_info x2_ptp_clock_ops = {
	.owner = THIS_MODULE,
	.name = "x2_ptp_clock",
	.max_adj = 62500000,
	.n_alarm = 0,
	.n_ext_ts = 0,
	.n_per_out = 0,
	.n_pins = 0,
	.pps = 0,
	.adjfreq = x2_adjust_freq,
	.adjtime = x2_adjust_time,
	.gettime64 = x2_get_time,
	.settime64 = x2_set_time,
	.enable = x2_ptp_enable,

};


static void x2_ptp_register(struct x2_priv *priv)
{

	int i;

	for (i = 0; i < priv->dma_cap.pps_out_num; i++) {
		if (i >= X2_PPS_MAX)
			break;

		priv->pps[i].available = true;
	}

	x2_ptp_clock_ops.n_per_out = priv->dma_cap.pps_out_num;

	spin_lock_init(&priv->ptp_lock);

	priv->ptp_clock_ops = x2_ptp_clock_ops;
	priv->ptp_clock = ptp_clock_register(&priv->ptp_clock_ops, priv->device);

	if (IS_ERR(priv->ptp_clock)) {
		printk("ptp_clock_register failed by hobot\n");
		priv->ptp_clock = NULL;
	} else if (priv->ptp_clock)
		printk("registered PTP clock by hobot successfully\n");
}
static int x2_init_ptp(struct x2_priv *priv)
{

	if (!(priv->dma_cap.time_stamp || priv->dma_cap.atime_stamp))
		return -EOPNOTSUPP;

	priv->adv_ts = 1;

	if (priv->dma_cap.time_stamp)
		printk("IEEE 1588-2002 Timestamp supported by Hobot Ethernet Network Card\n");
	if (priv->adv_ts)
		printk("IEEE 1588-2008 Advanced Timestamp supported by Hobot Ethernet Network Card\n");

	//priv->hw->ptp = &x2_ptp;

	priv->hwts_tx_en = 0;
	priv->hwts_rx_en = 0;

	x2_ptp_register(priv);

	return 0;
}

static int x2_hw_setup(struct net_device *ndev, bool init_ptp)
{
	struct x2_priv *priv = netdev_priv(ndev);
	//u32 rx_cnt = priv->plat->rx_queues_to_use;
	u32 tx_cnt = priv->plat->tx_queues_to_use;
	int ret;
	u32 chan;


	ret = x2_init_dma_engine(priv);
	if (ret < 0) {
		printk("%s, DMA engine initilization failed\n",__func__);
		return ret;
	}

	x2_set_umac_addr(priv->ioaddr, ndev->dev_addr, 0);

	x2_core_init(priv, ndev->mtu);

	x2_mtl_config(priv);

//	x2_est_configuration(priv);

	ret = x2_rx_ipc_enable(priv);
	if (!ret) {
		printk("RX IPC checksum offload disabled\n");
		priv->plat->rx_coe = STMMAC_RX_COE_NONE;
		priv->rx_csum = 0;
	}

	x2_set_mac(priv->ioaddr, true);

	x2_dma_operation_mode(priv);

	x2_mmc_setup(priv);

	if (init_ptp) {
		#if 0
		ret = clk_prepare_enable(priv->plat->clk_ptp_ref);
		if (ret < 0)
			printk("failed to enable PTP reference clock\n");
		#endif
		ret = x2_init_ptp(priv);
		if (ret == -EOPNOTSUPP) {
			printk("PTP not supported by HW\n");
		} else if(ret)
			printk("PTP init failed\n");
	}

	x2_set_rings_length(priv);
	x2_start_all_dma(priv);

	if (priv->tso) {
		u32 value;
		for (chan = 0; chan < tx_cnt; chan++) {
			value = readl(priv->ioaddr + DMA_CHAN_TX_CONTROL(chan));
			writel(value | DMA_CONTROL_TSE, priv->ioaddr + DMA_CHAN_TX_CONTROL(chan));

		}
	}



	return 0;
}


static void free_dma_desc_resources(struct x2_priv *priv)
{
	free_dma_rx_desc_resources(priv);
	free_dma_tx_desc_resources(priv);

}

#if 0
static void x2_hw_teardown(struct net_device *ndev)
{
//	struct x2_priv *priv = netdev_priv(ndev);


}
#endif

static void x2_enable_all_queues(struct x2_priv *priv)
{
	u32 rx_cnt = priv->plat->rx_queues_to_use;
	u32 queue;

	for (queue = 0; queue < rx_cnt; queue++) {
		struct x2_rx_queue *rx_q = &priv->rx_queue[queue];
		napi_enable(&rx_q->napi);
	}
}

static void x2_start_all_queues(struct x2_priv *priv)
{
	u32 tx_cnt = priv->plat->tx_queues_to_use;
	u32 queue;

	for (queue = 0; queue < tx_cnt; queue++)
		netif_tx_start_queue(netdev_get_tx_queue(priv->dev, queue));
}

static int x2_dma_interrupt(void __iomem *ioaddr, struct x2_extra_stats *x, u32 chan)
{
	int ret = 0;
	u32 intr_status = readl(ioaddr + DMA_CHAN_STATUS(chan)); //0x1160

	//printk("%s, chan:%d, intr_status:0x%x\n",__func__, chan, intr_status);
	if (intr_status & DMA_CHAN_STATUS_AIS) {
		if (intr_status & DMA_CHAN_STATUS_RBU) {
			printk("%s, rx_buf_unavailble\n",__func__);
			x->rx_buf_unav_irq++;
		}
		if (intr_status & DMA_CHAN_STATUS_RPS)
			x->rx_process_stopped_irq++;

		if (intr_status & DMA_CHAN_STATUS_RWT)
			x->rx_watchdog_irq++;

		if (intr_status & DMA_CHAN_STATUS_ETI)
			x->tx_early_irq++;

		if (intr_status & DMA_CHAN_STATUS_TPS) {
			x->tx_process_stopped_irq++;
			ret = tx_hard_error;
		}


		if (intr_status & DMA_CHAN_STATUS_FBE) {
			x->fatal_bus_error_irq++;
			ret = tx_hard_error;
		}
	}


	if (intr_status & DMA_CHAN_STATUS_NIS) {
		x->normal_irq_n++;
		if (intr_status & DMA_CHAN_STATUS_RI) {
			u32 value;

			value = readl(ioaddr + DMA_CHAN_INTR_ENA(chan));
			if (value & DMA_CHAN_INTR_ENA_RIE) {
				x->rx_normal_irq_n++;
				ret |= handle_rx;
			}
		}

		if (intr_status & DMA_CHAN_STATUS_TI) {
			x->tx_normal_irq_n++;
			ret |= handle_tx;
		}

		if (intr_status & DMA_CHAN_STATUS_ERI)
			x->rx_early_irq++;
	}

	writel((intr_status & 0x3fffc7), ioaddr + DMA_CHAN_STATUS(chan));
	return ret;
}

static void x2_disable_dma_irq(struct x2_priv *priv, u32  chan)
{
	writel(0, priv->ioaddr + DMA_CHAN_INTR_ENA(chan));
}


static void x2_init_tx_desc(struct dma_desc *p, u32 cnt)
{
	p->des0 = 0;
	p->des1 = 0;
	p->des2 = 0;
	p->des3 = 0;
}

static void x2_stop_tx_dma(struct x2_priv *priv, u32 chan)
{
	void __iomem *ioaddr = priv->ioaddr;

	u32 value = readl(ioaddr + DMA_CHAN_TX_CONTROL(chan));

	value &= ~DMA_CONTROL_ST;
	writel(value, ioaddr + DMA_CHAN_TX_CONTROL(chan));

	value = readl(ioaddr + GMAC_CONFIG);
	value &= ~GMAC_CONFIG_TE;
	writel(value, ioaddr + GMAC_CONFIG);

}


static void x2_tx_err(struct x2_priv *priv, u32 chan)
{
	struct x2_tx_queue *tx_q = &priv->tx_queue[chan];
	int i;

	netif_tx_stop_queue(netdev_get_tx_queue(priv->dev, chan));
	x2_stop_tx_dma(priv, chan);
	dma_free_tx_skbufs(priv, chan);

	//printk("%s,by dhw\n",__func__);
	for (i = 0; i < DMA_TX_SIZE; i++)
		if (priv->extend_desc) {
			x2_init_tx_desc(&tx_q->dma_etx[i].basic, (i == DMA_TX_SIZE -1 ));
		} else {

			x2_init_tx_desc(&tx_q->dma_tx[i], (i == DMA_TX_SIZE - 1));
		}

	tx_q->dirty_tx = 0;
	tx_q->cur_tx = 0;

	netdev_tx_reset_queue(netdev_get_tx_queue(priv->dev, chan));
	x2_start_tx_dma(priv, chan);
	priv->dev->stats.tx_errors++;
	netif_tx_wake_queue(netdev_get_tx_queue(priv->dev, chan));

}

static inline void x2_pcs_isr(void __iomem *ioaddr, u32 reg, unsigned int intr_status, struct x2_extra_stats *x)
{
	u32 val = readl(ioaddr + GMAC_AN_STATUS(reg));

//	printk("%s, stautus: 0x%x, val:0x%x\n",__func__,intr_status,val);
//	printk("%s,and ane:0x%x\n",__func__,readl(ioaddr + 0xe0));
	if (intr_status & PCS_ANE_IRQ) {
		x->irq_pcs_ane_n++;
		if (val & GMAC_AN_STATUS_ANC)
			printk("x2 pcs: ANE process completed\n");
	}

	if (intr_status & PCS_LINK_IRQ) {
		x->irq_pcs_link_n++;
		if (val & GMAC_AN_STATUS_LS)
			printk("x2 pcs: link up\n");
		else
			printk("x2 pcs: Link down\n");
	}
}


static void x2_phystatus(void __iomem *ioaddr, struct x2_extra_stats *x)
{
	u32 status;

	status = readl(ioaddr + GMAC_PHYIF_CONTROL_STATUS);
	x->irq_rgmii_n++;

	if (status & GMAC_PHYIF_CTRLSTATUS_LNKSTS) {
		int speed_value;

		x->pcs_link = 1;

		speed_value = ((status & GMAC_PHYIF_CTRLSTATUS_SPEED) >> GMAC_PHYIF_CTRLSTATUS_SPEED_SHIFT);

		if (speed_value == GMAC_PHYIF_CTRLSTATUS_SPEED_125)
			x->pcs_speed = SPEED_1000;
		else if (speed_value == GMAC_PHYIF_CTRLSTATUS_SPEED_25)
			x->pcs_speed = SPEED_100;
		else
			x->pcs_speed = SPEED_10;

		x->pcs_duplex = ((status & GMAC_PHYIF_CTRLSTATUS_LNKMOD ) >> GMAC_PHYIF_CTRLSTATUS_LNKMOD_MASK);
		printk("Link is Up - %d/%s \n",(int)x->pcs_speed, x->pcs_duplex ? "Full": "Half");
	} else {
		x->pcs_link = 0;
		printk("Link is Down \n");
	}
//	printk("%s, and mac_phy_status_reg-0xf8: 0x%x\n",__func__, readl(ioaddr + 0xf8));
//	printk("%s, and mac_AN_stauts_reg-0xe4: 0x%x\n",__func__, readl(ioaddr + 0xe4));

}
static int x2_host_irq_status(struct x2_priv *priv, struct x2_extra_stats *x)
{
	void __iomem *ioaddr = priv->ioaddr;

	u32 intr_status = readl(ioaddr + GMAC_INT_STATUS);
	u32 intr_enable = readl(ioaddr + GMAC_INT_EN);
	int ret = 0;

	intr_status &= intr_enable;

//	printk("%s, intr status:0x%x\n",__func__,intr_status);
	if (intr_status & mmc_tx_irq) {
		x->mmc_tx_irq_n++;
	//	printk("%s, mmc_tx_irq_n: %d\n",__func__,x->mmc_tx_irq_n);

	}
	if (intr_status & mmc_rx_csum_offload_irq) {
		x->mmc_rx_csum_offload_irq_n++;
	//	printk("%s, mmc_rx_csum_offload_irq_n: %d\n",__func__,x->mmc_rx_csum_offload_irq_n);
	}
	if (intr_status & pmt_irq) {
		readl(ioaddr + GMAC_PMT);
		x->irq_receive_pmt_irq_n++;

	//	printk("%s, irq receive pmt irq%d\n",__func__);
	}

	if (intr_status & mac_fpeis) {
		u32 value = readl(ioaddr + GMAC_FPE_CTRL_STS);
		printk("%s, Frame preemption interrupt, fpe_ctrl_sts:0x%x\n", __func__,value);
		printk("%s, frg cntr:%d, hlod:%d\n", __func__, readl(ioaddr + MMC_TX_FPE_Frg_Cntr),readl(ioaddr + MMC_TX_HOLD_Req_Cntr));
		writel(value, ioaddr + GMAC_FPE_CTRL_STS);
	}
	x2_pcs_isr(ioaddr, GMAC_PCS_BASE, intr_status, x);

//	printk("%s, and intr-status:0x%x, pcs_rgsmiiis_irq:0x%x\n",__func__, intr_status, PCS_RGSMIIIS_IRQ);
	if (intr_status & PCS_RGSMIIIS_IRQ) {
	//	printk("%s, and phy status\n", __func__);

		x2_phystatus(ioaddr, x);

	}

	return ret;
}

static int x2_host_mtl_irq_status(struct x2_priv *priv, u32 chan)
{
	void __iomem *ioaddr = priv->ioaddr;
	u32 mtl_irq_status;
	int ret = 0;

	mtl_irq_status = readl(ioaddr + MTL_INT_STATUS);
//	printk("%s, chan:%d, mtl_irq_status:0x%x\n",__func__,chan,mtl_irq_status);
	if (mtl_irq_status & MTL_INT_QX(chan)) {
		u32 status = readl(ioaddr + MTL_CHAN_INT_CTRL(chan));
	//	printk("%s, and int MTL_CHAN_INT_CTRL: 0x%x\n", __func__, status);
		if (status & MTL_RX_OVERFLOW_INT) {
			writel(status | MTL_RX_OVERFLOW_INT, ioaddr + MTL_CHAN_INT_CTRL(chan));
			ret = CORE_IRQ_MTL_RX_OVERFLOW;
		}

		if (status & MTL_ABPSIS_INT) {
			if (readl(ioaddr + MTL_ETSX_STATUS_BASE_ADDR(chan)))
			;	//printk("queue:%d, averege bit/ %d slot number: 0x%x\n",chan,(readl(ioaddr + MTL_ETSX_CTRL_BASE_ADDR(chan)) >> 4 & 0x7),readl(ioaddr + MTL_ETSX_STATUS_BASE_ADDR(chan)));
			writel(status, ioaddr + MTL_CHAN_INT_CTRL(chan));
		}
	}


	if (mtl_irq_status & MTL_ESTIS) {
		u32 status = readl(ioaddr + 0xc58);
	//	printk("%s, MTL EST intterupt here, status:0x%x\n",__func__,status);
		if (status & MTL_STATUS_CGSN) {
	//		printk("est current gcl slot num:%d\n",((status & MTL_STATUS_CGSN) >> 16) & 0xf);
			;
		}

		if (status & MTL_STATUS_BTRL) {
		//	printk("btr error loop count:%d\n", ((status & MTL_STATUS_BTRL) >> 8) & 0xf);
			;
		}

		if (status & MTL_STATUS_SWOL)
			printk("gate control list %ld own by sofware\n", (status & MTL_STATUS_SWOL >> 7)& 0x1);
		else
			printk("gcl 0 own by software\n");

		if (status & MTL_STATUS_CGCE) {
			printk("constant gate control error\n");
		}
		if (status & MTL_STATUS_HLBS)
			printk("head of line blocking due scheduling\n");

		if (status & MTL_STATUS_HLBF) {

			u32 queue = readl(ioaddr + MTL_EST_Frm_Size_Error);
			//u32 frame = readl(ioaddr + MTL_EST_Frm_Size_Capture);
	//		printk("head of line bocking due to frame size, and queue:%d\n",queue);
	//		printk("HOF block frame size:%d, and the queue:%d\n",frame & 0x3fff, (frame >> 16));
			writel(queue, ioaddr + MTL_EST_Frm_Size_Error);
		}
		if (status & MTL_STATUS_BTRE)
			printk("BTR error\n");

		if (status & MTL_STATUS_SWLC)
			printk("switch to S/W owned list complete\n");

		writel(status, ioaddr + 0xc58);
	}

	return ret;
}



static irqreturn_t x2_interrupt(int irq, void *dev_id)
{

	struct net_device *ndev = (struct net_device *)dev_id;
	struct x2_priv *priv = netdev_priv(ndev);
	u32 rx_cnt = priv->plat->rx_queues_to_use;
	u32 tx_cnt = priv->plat->tx_queues_to_use;
	u32 queues_count;
	u32 queue;
	int mtl_status = 0;
	int status;
	u32 chan;

//	int i;


#if 0
	/*read dma tx enable*/
	printk("%s: mac_conf-reg0:0x%x\n",__func__,readl(priv->ioaddr + 0));

	printk("%s,and rxq_ctrl0-reg0xa0:0x%x\n",__func__,readl(priv->ioaddr + 0xa0));
	printk("%s, and txq0_opera_reg0xd00:0x%x\n",__func__, readl(priv->ioaddr + 0xd00));


	printk("%s, and txq0_debug_reg0xd08:0x%x\n",__func__, readl(priv->ioaddr + 0xd08));

	for(i = 1; i < tx_cnt; i++){
		printk("%s, txq(%d)_operation_mode:0x%x\n", __func__, i, readl(priv->ioaddr + 0xd00 + 0x40 * i));
		printk("%s, txq(%d)_debug_mode:0x%x\n", __func__, i, readl(priv->ioaddr + 0xd08 + 0x40 * i));

	}

	for (i = 0; i < 4; i++) {
		printk("%s, and DMA_CH%d_int_enable:0x%x\n", __func__,i, readl(priv->ioaddr + 0x80*i + 0x1134));
	}
#endif


	queues_count = (rx_cnt > tx_cnt) ? rx_cnt : tx_cnt;

//	printk("%s\n",__func__);
	status = x2_host_irq_status(priv, &priv->xstats);

	if (status) {
		if (status & CORE_IRQ_TX_PATH_IN_LPI_MODE)
			priv->tx_path_in_lpi_mode = true;

		if (status & CORE_IRQ_TX_PATH_EXIT_LPI_MODE)
			priv->tx_path_in_lpi_mode = false;
	}

	for (queue = 0; queue < queues_count; queue++) {
		struct x2_rx_queue *rx_q = &priv->rx_queue[queue];
		//status |= x2_host_mtl_irq_status(priv, queue);
		mtl_status = status | x2_host_mtl_irq_status(priv, queue);

		//if (status & CORE_IRQ_MTL_RX_OVERFLOW) {
		if (mtl_status & CORE_IRQ_MTL_RX_OVERFLOW) {
			x2_set_rx_tail_ptr(priv->ioaddr, rx_q->rx_tail_addr, queue);
		}
	}


	if (priv->xstats.pcs_link)
		netif_carrier_on(ndev);
	else
		netif_carrier_off(ndev);


	for (chan = 0; chan < tx_cnt; chan++) {
		struct x2_rx_queue *rx_q = &priv->rx_queue[chan];
		status = x2_dma_interrupt(priv->ioaddr, &priv->xstats, chan);

		//printk("%s, and status:0x%x, handler_rx:0x%x, handle_tx:0x%x\n",__func__,status, handle_rx, handle_tx);
		if (likely((status & handle_rx)) || (status & handle_tx) || (mtl_status & CORE_IRQ_MTL_RX_OVERFLOW)) {
			if (napi_schedule_prep(&rx_q->napi)) {
				x2_disable_dma_irq(priv, chan);
				__napi_schedule(&rx_q->napi);
			}
		}

		if (status & tx_hard_error) {
			x2_tx_err(priv, chan);
		}
	}


	return IRQ_HANDLED;
}

static int x2_ioctl_get_qmode(struct x2_priv *priv, void __user *data)
{
	u32 tx_qcount = priv->plat->tx_queues_to_use;
	struct x2_qmode_cfg mode;

	u8 qmode;

	if (copy_from_user(&mode, data, sizeof(mode)))
		return -EINVAL;

	if (mode.queue_idx >= tx_qcount)
		return -EINVAL;

	qmode = priv->plat->tx_queues_cfg[mode.queue_idx].mode_to_use;
	switch (qmode) {
	case MTL_QUEUE_DCB:
		mode.queue_mode = STMMAC_QMODE_DCB;
		break;
	case MTL_QUEUE_AVB:
		mode.queue_mode = STMMAC_QMODE_AVB;
		break;
	default:
		return -EINVAL;
	}


	if (copy_to_user(data, &mode, sizeof(mode)))
		return -EFAULT;

	return 0;
}

static int x2_set_est(struct x2_priv *priv, void __user *data)
{
	struct x2_est_cfg *cfg = &priv->plat->est_cfg;
	struct x2_est_cfg *est;
	int ret = 0;

	est = kzalloc(sizeof(*est), GFP_KERNEL);
	if (!est)
		return -ENOMEM;

	if (copy_from_user(est, data, sizeof(*est))) {
		ret = -EFAULT;
		goto out_free;
	}

	if (est->gcl_size > STMMAC_EST_GCL_MAX_ENTRIES) {
		ret = -EINVAL;
		goto out_free;
	}

	if (est->enabled) {
		cfg->btr_offset[0] = est->btr_offset[0];
		cfg->btr_offset[1] = est->btr_offset[1];
		cfg->ctr[0] = est->ctr[0];
		cfg->ctr[1] = est->ctr[1];
		cfg->ter = est->ter;
		cfg->gcl_size = est->gcl_size;

		memcpy(cfg->gcl, est->gcl, cfg->gcl_size * sizeof(*cfg->gcl));
	} else {

		cfg->btr_offset[0] = 0;
		cfg->btr_offset[1] = 0;
		cfg->ctr[0] = 0;
		cfg->ctr[1] = 0;
		cfg->ter = 0;
		cfg->gcl_size = 0;
		memset(cfg->gcl, 0, sizeof(cfg->gcl));

	}

	priv->plat->est_en = est->enabled;
	ret = x2_est_configuration(priv);
	if (!est->enabled)
		ret = 0;
	else {
//		printk("%s, dma_cap.fpesel:%d, plat->fp_en:%d\n", __func__, priv->dma_cap.fpesel, priv->plat->fp_en);
//		if (priv->dma_cap.fpesel && priv->plat->fp_en)
		x2_tsn_fp_configure(priv);
	}


out_free:
	kfree(est);
	return ret;

}
static void x2_est_read(struct x2_priv *priv, u32 reg, bool is_gcla)
{
	u32 control = 0x0;
	int timeout = 15;


	control = readl(priv->ioaddr + MTL_EST_GCL_CONTROL);
	control |= reg;
	control |= is_gcla ? 0x0 : MTL_EST_GCRR;
	control |= (1 << 1);
	control |= MTL_EST_SRWO;

	writel(control, priv->ioaddr + MTL_EST_GCL_CONTROL);

	while(--timeout) {
		udelay(1000);
		if (readl(priv->ioaddr + MTL_EST_GCL_CONTROL) & MTL_EST_SRWO)
			continue;
		break;
	}



//	printk("%s, reg:0x%x, value:0x%x\n", __func__, reg, readl(priv->ioaddr + MTL_EST_GCL_DATA) );

}


static int x2_get_est(struct x2_priv *priv, void __user *data)
{
	struct x2_est_cfg *est;
	int ret = 0;
	int i;

	est = kzalloc(sizeof(*est), GFP_KERNEL);
	if (!est)
		return -ENOMEM;


//	printk("%s,and est_enabled:%d\n", __func__,priv->est_enabled);
	est->enabled = priv->est_enabled;
	est->estwid = priv->dma_cap.estwid;
	est->estdep = priv->dma_cap.estdep;
	est->btr_offset[0] = priv->plat->est_cfg.btr_offset[0];
	est->btr_offset[1] = priv->plat->est_cfg.btr_offset[1];
	est->ctr[0] = priv->plat->est_cfg.ctr[0];
	est->ctr[1] = priv->plat->est_cfg.ctr[1];
	est->ter = priv->plat->est_cfg.ter;
	est->gcl_size = priv->plat->est_cfg.gcl_size;
	memcpy(est->gcl, priv->plat->est_cfg.gcl, est->gcl_size * sizeof(*est->gcl));

	x2_est_read(priv, MTL_EST_BTR_LOW, false);
	x2_est_read(priv, MTL_EST_BTR_HIGH, false);

	x2_est_read(priv, MTL_EST_CTR_LOW, false);
	x2_est_read(priv, MTL_EST_CTR_HIGH, false);
	x2_est_read(priv, MTL_EST_TER,  false);
	x2_est_read(priv, MTL_EST_LLR, false);

	//printk("%s, before\n",__func__);

	for (i = 0; i < 5; i++) {
		u32 reg = (i << MTL_EST_ADDR_OFFSET) & MTL_EST_ADDR;
		x2_est_read(priv, reg, true);
	}



	if (copy_to_user(data, est, sizeof(*est))) {
		ret = -EFAULT;
		goto out_free;
	}
	//printk("%s, reg-0x8ac:%d\n\n",__func__,readl(priv->ioaddr + 0x8ac));
	//printk("%s, reg-0x8d0:%d\n",__func__,readl(priv->ioaddr + 0x8d0));
out_free:
	kfree(est);
	return ret;
}


static int x2_ioctl_get_pps(struct x2_priv *priv, void __user *data)
{
	u32 ptp_period = x2_get_ptp_period(priv, priv->plat->clk_ptp_rate);
	struct x2_ioctl_pps_cfg pps;
	struct stmmac_pps_cfg *cfg;
	bool dig;

	memset(&pps, 0, sizeof(pps));

	if (copy_from_user(&pps, data, sizeof(pps)))
		return -EFAULT;

	if (pps.index >= X2_PPS_MAX)
		return -EINVAL;

	if (pps.index >= priv->dma_cap.pps_out_num)
		return -EINVAL;

	cfg = &priv->plat->pps_cfg[pps.index];
	dig = x2_get_hw_tstamping(priv) & PTP_TCR_TSCTRLSSR;
	pps.enabled = cfg->enable;
	pps.ctrl_cmd = cfg->ctrl_cmd;

	if (pps.enabled) {
		pps.trgtmodsel = cfg->trgtmodsel;
		pps.target_time[0] = cfg->target_time[0];
		pps.target_time[1] = cfg->target_time[1];
		pps.interval = cfg->interval * ptp_period;
		pps.width = cfg->width * ptp_period;
	} else {
		pps.freq = 0x1 << pps.ctrl_cmd;
		if (dig)
			pps.freq /= 2;
	}

	if (copy_from_user(data, &pps, sizeof(pps)))
		return -EFAULT;

	return 0;
}

static int x2_ioctl_set_pps(struct x2_priv *priv, void __user *data)
{
       u32 ptp_period = x2_get_ptp_period(priv, priv->plat->clk_ptp_rate);
       struct x2_ioctl_pps_cfg pps;
       struct stmmac_pps_cfg *cfg;

       memset(&pps, 0, sizeof(pps));

       if (copy_from_user(&pps, data, sizeof(pps)))
               return -EFAULT;
       if (pps.index >= X2_PPS_MAX)
               return -EINVAL;
       if (pps.index >= priv->dma_cap.pps_out_num)
               return -EINVAL;

       cfg = &priv->plat->pps_cfg[pps.index];

       cfg->enable = pps.enabled;
       cfg->ctrl_cmd = pps.ctrl_cmd;
       cfg->trgtmodsel = pps.trgtmodsel;
       cfg->target_time[0] = pps.target_time[0];
       cfg->target_time[1] = pps.target_time[1];
       cfg->interval = pps.interval / ptp_period;
       cfg->width = pps.width / ptp_period;

	return x2_pps_configuration(priv, pps.index);
}


static int x2_extension_ioctl(struct x2_priv *priv, void __user *data)
{
	u32 tx_cnt = priv->plat->tx_queues_to_use;
	int txfifosz = priv->plat->tx_fifo_size;
	struct x2_qmode_cfg mode;
	struct x2_cbs_cfg cbs;
	u32 txmode = 0;
	u8 qmode;
	u32 cmd;
	//u32 value = 0;

//	printk("%s,and here\n",__func__);
	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (copy_from_user(&cmd, data, sizeof(cmd)))
		return -EFAULT;

	if (txfifosz == 0)
		txfifosz = priv->dma_cap.tx_fifo_size;

	txfifosz /= tx_cnt;

	switch (cmd) {
	case STMMAC_GET_QMODE:
		return x2_ioctl_get_qmode(priv, data);

	case STMMAC_SET_QMODE:
		if (copy_from_user(&mode, data, sizeof(mode)))
			return -EFAULT;
		if (mode.queue_idx >= tx_cnt)
			return -EINVAL;

		txmode = SF_DMA_MODE;

		switch (mode.queue_mode) {
		case STMMAC_QMODE_DCB:
			qmode = MTL_QUEUE_DCB;
			break;
		case STMMAC_QMODE_AVB:
			qmode = MTL_QUEUE_AVB;
			break;
		default:
			return -EINVAL;
		}

		priv->plat->tx_queues_cfg[mode.queue_idx].mode_to_use = qmode;

		x2_dma_tx_chan_op_mode(priv->ioaddr, txmode, mode.queue_idx, txfifosz, qmode);

		break;

	case STMMAC_SET_CBS:

		if (copy_from_user(&cbs, data, sizeof(cbs)))
			return -EFAULT;
		if (cbs.queue_idx >= tx_cnt)
			return -EINVAL;

		qmode = priv->plat->tx_queues_cfg[cbs.queue_idx].mode_to_use;
		if (qmode != MTL_QUEUE_AVB)
			return -EINVAL;

//		printk("%s,and set cbs\n",__func__);
		priv->plat->tx_queues_cfg[cbs.queue_idx].send_slope = cbs.send_slope;
		priv->plat->tx_queues_cfg[cbs.queue_idx].idle_slope = cbs.idle_slope;
		priv->plat->tx_queues_cfg[cbs.queue_idx].high_credit = cbs.high_credit;
		priv->plat->tx_queues_cfg[cbs.queue_idx].low_credit = cbs.low_credit;
		priv->plat->tx_queues_cfg[cbs.queue_idx].percentage = cbs.percentage;
		x2_dma_config_cbs(priv, cbs.send_slope, cbs.idle_slope, cbs.high_credit, cbs.low_credit, cbs.queue_idx);

#if 0
		printk("%s, queue_idx:%d\n",__func__, cbs.queue_idx);
		value = readl(priv->ioaddr + MTL_CHAN_INT_CTRL(cbs.queue_idx));
		value |= (1 << 9);
		writel(value, priv->ioaddr + MTL_CHAN_INT_CTRL(cbs.queue_idx));
#endif
		break;
	case STMMAC_GET_CBS:
		if (copy_from_user(&cbs, data, sizeof(cbs)))
			return -EFAULT;
		if (cbs.queue_idx >= tx_cnt)
			return -EINVAL;

		cbs.send_slope = priv->plat->tx_queues_cfg[cbs.queue_idx].send_slope;
		cbs.idle_slope = priv->plat->tx_queues_cfg[cbs.queue_idx].idle_slope;
		cbs.high_credit = priv->plat->tx_queues_cfg[cbs.queue_idx].high_credit;
		cbs.low_credit = priv->plat->tx_queues_cfg[cbs.queue_idx].low_credit;
		cbs.percentage = priv->plat->tx_queues_cfg[cbs.queue_idx].percentage;

		if (copy_to_user(data, &cbs, sizeof(cbs)))
			return -EFAULT;
		break;

	case STMMAC_GET_EST:
		return x2_get_est(priv, data);

	case STMMAC_SET_EST:
		return x2_set_est(priv, data);

	case STMMAC_GET_PPS:
		return x2_ioctl_get_pps(priv, data);
	case STMMAC_SET_PPS:
		return x2_ioctl_set_pps(priv, data);
	default:
		return -EINVAL;
	}

	return 0;
}


static int x2_open(struct net_device *ndev)
{
	struct x2_priv *priv = netdev_priv(ndev);
	int ret;

//	u32 value;


	ret = x2_init_phy(ndev);
	if (ret < 0) {
		printk("%s, init phy error \n",__func__);
		return ret;
	}

	priv->dma_buf_sz =3000;//16384;//1536;

	ret = alloc_dma_desc_resources(priv);
	if (ret < 0) {
		printk("%s, DMA descriptor alloc failed\n",__func__);
		goto dma_desc_error;
	}


	ret = init_dma_desc_rings(ndev);//, GFP_KERNEL);
	if (ret < 0) {
		printk("%s: DMA desc rings initalize failed\n",__func__);
		goto init_error;
	}

	ret = x2_hw_setup(ndev, true);
	if (ret < 0) {
		printk("%s, Hw setup failed\n",__func__);
		goto init_error;
	}

	//if (ndev->phydev)
	phy_start(ndev->phydev);

#if 0
	ret = devm_request_irq(priv->device, ndev->irq, &x2_interrupt, 0, ndev->name, ndev);
	if (ret < 0) {
		printk("%s: error allocating the IRQ\n",__func__);
		goto irq_error;
	}
#endif

	x2_enable_all_queues(priv);
	x2_start_all_queues(priv);

	printk("%s: successfully\n",__func__);
	return 0;


init_error:
	free_dma_desc_resources(priv);
dma_desc_error:

	if (ndev->phydev)
		phy_disconnect(ndev->phydev);

	return ret;
}

static void x2_stop_all_queues(struct x2_priv *priv)
{
	u32 tx_cnt = priv->plat->tx_queues_to_use;
	u32 queue;

	for (queue = 0; queue < tx_cnt; queue++)
		netif_tx_stop_queue(netdev_get_tx_queue(priv->dev, queue));
}

static void x2_disable_all_queues(struct x2_priv *priv)
{
	u32 rx_cnt = priv->plat->rx_queues_to_use;
	u32 queue;

	for (queue = 0; queue < rx_cnt; queue++) {
		struct x2_rx_queue *rx_q = &priv->rx_queue[queue];

		napi_disable(&rx_q->napi);
	}
}

static void x2_stop_rx_dma(struct x2_priv *priv, u32 chan)
{
	void __iomem *ioaddr = priv->ioaddr;

	u32 value = readl(ioaddr + DMA_CHAN_RX_CONTROL(chan));

	value &= ~DMA_CONTROL_SR;
	writel(value, ioaddr + DMA_CHAN_RX_CONTROL(chan));

	value = readl(ioaddr + GMAC_CONFIG);
	value &= ~GMAC_CONFIG_RE;
	writel(value, ioaddr + GMAC_CONFIG);
}
#if 0
static void x2_stop_tx_dma(struct x2_priv *priv, u32 chan)
{
	void __iomem *ioaddr = priv->ioaddr;

	u32 value = readl(ioaddr + DMA_CHAN_TX_CONTROL(chan));

	value &= ~DMA_CONTROL_ST;
	writel(value, ioaddr + DMA_CHAN_TX_CONTROL(chan));

	value = readl(ioaddr + GMAC_CONFIG);
	value &= ~GMAC_CONFIG_TE;
	writel(value, ioaddr + GMAC_CONFIG);

}
#endif

static void x2_stop_all_dma(struct x2_priv *priv)
{
	u32 rx_cnt = priv->plat->rx_queues_to_use;
	u32 tx_cnt = priv->plat->tx_queues_to_use;
	u32 chan = 0;

	for (chan = 0; chan < rx_cnt; chan++)
		x2_stop_rx_dma(priv, chan);

	for (chan = 0; chan < tx_cnt; chan++)
		x2_stop_tx_dma(priv, chan);
}

static int x2_release(struct net_device *ndev)
{
	struct x2_priv *priv = netdev_priv(ndev);

	if (ndev->phydev) {
		phy_stop(ndev->phydev);
		phy_disconnect(ndev->phydev);
	}

	x2_stop_all_queues(priv);
	x2_disable_all_queues(priv);

//	free_irq(ndev->irq, ndev);
//	printk("%s, and free irq by dhw\n",__func__);
	x2_stop_all_dma(priv);
	free_dma_desc_resources(priv);

	x2_set_mac(priv->ioaddr, false);
	netif_carrier_off(ndev);

	return 0;
}

static inline u32 x2_tx_avail(struct x2_priv *priv, u32 queue)
{
	struct x2_tx_queue *tx_q = &priv->tx_queue[queue];
	u32 avail;

	if (tx_q->dirty_tx > tx_q->cur_tx)
		avail = tx_q->dirty_tx - tx_q->cur_tx - 1;
	else
		avail = DMA_TX_SIZE - tx_q->cur_tx + tx_q->dirty_tx - 1;
	return avail;
}


static void x2_prepare_tx_desc(struct dma_desc *p, int is_fs, int len, bool csum_flags, int tx_own, bool ls, unsigned int tot_pkt_len)
{
	unsigned int tdes3 = le32_to_cpu(p->des3);

	p->des2 |= cpu_to_le32(len & TDES2_BUFFER1_SIZE_MASK);

	tdes3 |= tot_pkt_len & TDES3_PACKET_SIZE_MASK;

	if (is_fs)
		tdes3 |= TDES3_FIRST_DESCRIPTOR;
	else
		tdes3 &= ~TDES3_FIRST_DESCRIPTOR;

	if (csum_flags)
		tdes3 |= (TX_CIC_FULL << TDES3_CHECKSUM_INSERTION_SHIFT);
	else
		tdes3 &= ~(TX_CIC_FULL << TDES3_CHECKSUM_INSERTION_SHIFT);


	if (ls)
		tdes3 |= (TDES3_LAST_DESCRIPTOR);
	else
		tdes3 &= ~(TDES3_LAST_DESCRIPTOR);

	if (tx_own)
		tdes3 |= TDES3_OWN;

	if (is_fs & tx_own)
		dma_wmb();

	p->des3 = cpu_to_le32(tdes3);

}


static void x2_enable_tx_timestamp(struct dma_desc *p)
{
	p->des2 |= cpu_to_le32(TDES2_TIMESTAMP_ENABLE);
}


static void x2_set_mss(struct dma_desc *p, unsigned int mss)
{
	p->des0 = 0;
	p->des1 = 0;
	p->des2 =cpu_to_le32(mss);
	p->des3 = cpu_to_le32(TDES3_CONTEXT_TYPE | TDES3_CTXT_TCMSSV);

//	printk("%s,and mss:%d\n", __func__, mss);
}

static void x2_prepare_tso_tx_desc(struct dma_desc *p, int is_fs, int len1, int len2, bool tx_own, bool ls, unsigned int tcphdrlen, unsigned int tcppayloadlen)
{
	unsigned int tdes3 = le32_to_cpu(p->des3);

//	printk("%s, is_fs:%d, len1:%d, len2:%d, ls:%d, tcphdr:%d, tcppayloadlen:%d\n", __func__, is_fs, len1, len2, ls, tcphdrlen,tcppayloadlen);
	if (len1)
		p->des2 |= cpu_to_le32((len1 & TDES2_BUFFER1_SIZE_MASK));

	if (len2)
		p->des2 |= cpu_to_le32((len2 << TDES2_BUFFER2_SIZE_MASK_SHIFT) & TDES2_BUFFER2_SIZE_MASK);

	if (is_fs) {
		tdes3 |= TDES3_FIRST_DESCRIPTOR | TDES3_TCP_SEGMENTATION_ENABLE | ((tcphdrlen << TDES3_HDR_LEN_SHIFT) & TDES3_SLOT_NUMBER_MASK) | ((tcppayloadlen & TDES3_TCP_PKT_PAYLOAD_MASK));

	} else  {
		tdes3 &= ~TDES3_FIRST_DESCRIPTOR;
	}

	if (ls)
		tdes3 |= TDES3_LAST_DESCRIPTOR;
	else
		tdes3 &= ~TDES3_LAST_DESCRIPTOR;

	if (tx_own)
		tdes3 |= TDES3_OWN;

	if (is_fs && tx_own)
		dma_wmb();

	p->des3 = cpu_to_le32(tdes3);
}

static void x2_tso_allocator(struct x2_priv *priv, unsigned int des, int total_len, bool last_segment, u32 queue)
{
	struct x2_tx_queue *tx_q = &priv->tx_queue[queue];
	struct dma_desc *desc;
	u32 buff_size;
	int tmp_len;

	tmp_len = total_len;

//	printk("%s, and tmp_len:%d, total_len:%d\n",__func__, tmp_len,total_len);
	while (tmp_len > 0) {
		tx_q->cur_tx = X2_GET_ENTRY(tx_q->cur_tx, DMA_TX_SIZE);
		desc = tx_q->dma_tx + tx_q->cur_tx;
		desc->des0 = cpu_to_le32(des + (total_len - tmp_len));
		buff_size = tmp_len >= TSO_MAX_BUFF_SIZE ? TSO_MAX_BUFF_SIZE:tmp_len;

	//	printk("%s, and in while,tmp_len:%d,  buffer_size:%d\n", __func__,tmp_len, buff_size);
		x2_prepare_tso_tx_desc(desc, 0, buff_size, 0, 1, (last_segment) && (tmp_len <= TSO_MAX_BUFF_SIZE), 0,0);
		tmp_len -= TSO_MAX_BUFF_SIZE;

	}
//	printk("%s,return from tso allocator\n", __func__);
}

//static int flags = 0;

static netdev_tx_t x2_tso_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct dma_desc *desc, *first, *mss_desc = NULL;
	struct x2_priv *priv = netdev_priv(ndev);
	int nfrags = skb_shinfo(skb)->nr_frags;
	u32 queue = skb_get_queue_mapping(skb);
	unsigned int first_entry, des;
	struct x2_tx_queue *tx_q;
	int tmp_pay_len = 0;
	u32 pay_len, mss;
	u8 proto_hdr_len;
	int i;

	tx_q = &priv->tx_queue[queue];

//	printk("%s: here\n",__func__);
	proto_hdr_len = skb_transport_offset(skb) + tcp_hdrlen(skb);
//	printk("%s, and transport offset:%d, tcphdrlen:%d\n", __func__, skb_transport_offset(skb), tcp_hdrlen(skb));
	if (unlikely(x2_tx_avail(priv, queue) < (((skb->len - proto_hdr_len) / TSO_MAX_BUFF_SIZE + 1)))) {
		if (!netif_tx_queue_stopped(netdev_get_tx_queue(ndev, queue))) {
			netif_tx_stop_queue(netdev_get_tx_queue(priv->dev, queue));
			printk("tx ring full when queue awake\n");
		}

		return NETDEV_TX_BUSY;
	}


	pay_len = skb_headlen(skb) - proto_hdr_len;
	mss = skb_shinfo(skb)->gso_size;
	if (mss != priv->mss) {
		mss_desc = tx_q->dma_tx + tx_q->cur_tx;
		x2_set_mss(mss_desc, mss);
		priv->mss = mss;
		tx_q->cur_tx = X2_GET_ENTRY(tx_q->cur_tx, DMA_TX_SIZE);
	}

	first_entry = tx_q->cur_tx;
	desc = tx_q->dma_tx + first_entry;
	first = desc;

//	printk("%s,prot_hdr_len:%d, pay_len:%d, headlen:%d, nfrags:%d\n",__func__, proto_hdr_len, pay_len, skb_headlen(skb),nfrags);
	des = dma_map_single(priv->device, skb->data, skb_headlen(skb), DMA_TO_DEVICE);
	if (dma_mapping_error(priv->device, des))
		goto dma_map_err;

	tx_q->tx_skbuff_dma[first_entry].buf = des;
	tx_q->tx_skbuff_dma[first_entry].len = skb_headlen(skb);

	first->des0 = cpu_to_le32(des);

	if (pay_len)
		first->des1 = cpu_to_le32(des + proto_hdr_len);

	tmp_pay_len = pay_len - TSO_MAX_BUFF_SIZE;

	x2_tso_allocator(priv, des, tmp_pay_len, (nfrags == 0), queue);

	for(i = 0; i < nfrags; i++) {
		const skb_frag_t *frag = &skb_shinfo(skb)->frags[i];

		des = skb_frag_dma_map(priv->device, frag, 0, skb_frag_size(frag), DMA_TO_DEVICE);
		if (dma_mapping_error(priv->device, des))
			goto dma_map_err;

		x2_tso_allocator(priv, des, skb_frag_size(frag), (i == nfrags - 1), queue);

		tx_q->tx_skbuff_dma[tx_q->cur_tx].buf = des;
		tx_q->tx_skbuff_dma[tx_q->cur_tx].len = skb_frag_size(frag);
		tx_q->tx_skbuff[tx_q->cur_tx] = NULL;
		tx_q->tx_skbuff_dma[tx_q->cur_tx].map_as_page = true;
	}


	tx_q->tx_skbuff_dma[tx_q->cur_tx].last_segment = true;

	tx_q->tx_skbuff[tx_q->cur_tx] = skb;

	tx_q->cur_tx = X2_GET_ENTRY(tx_q->cur_tx, DMA_TX_SIZE);
	if (unlikely(x2_tx_avail(priv, queue) <= (MAX_SKB_FRAGS + 1))) {
		netif_tx_stop_queue(netdev_get_tx_queue(priv->dev, queue));
	}

	ndev->stats.tx_bytes += skb->len;
	priv->xstats.tx_tso_frames++;
	priv->xstats.tx_tso_nfrags += nfrags;

	desc->des2 |= cpu_to_le32(TDES2_INTERRUPT_ON_COMPLETION);
	priv->xstats.tx_set_ic_bit++;


	skb_tx_timestamp(skb);

	if (unlikely((skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP) && priv->hwts_tx_en)) {
		skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
		x2_enable_tx_timestamp(first);
	}


	//printk("%s, before prepare tso tx desc,and is_fs is 1, last is 1 by dhw\n",__func__);

	x2_prepare_tso_tx_desc(first, 1, proto_hdr_len, pay_len, 1, tx_q->tx_skbuff_dma[first_entry].last_segment, tcp_hdrlen(skb)/4, (skb->len - proto_hdr_len));

	if (mss_desc) {
		dma_wmb();
		mss_desc->des3 |= cpu_to_le32(TDES3_OWN);
	}

	dma_wmb();

	netdev_tx_sent_queue(netdev_get_tx_queue(ndev, queue), skb->len);
	x2_set_tx_tail_ptr(priv->ioaddr, tx_q->tx_tail_addr, queue);


//	printk("%s, tso return\n",__func__);
	return NETDEV_TX_OK;

dma_map_err:
	dev_kfree_skb(skb);
	priv->dev->stats.tx_dropped++;
	return NETDEV_TX_OK;
}



//static int cnt = 0;

static netdev_tx_t x2_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct x2_priv *priv = netdev_priv(ndev);
	unsigned int nopaged_len = skb_headlen(skb);
	int nfrags = skb_shinfo(skb)->nr_frags;
	int entry;
	unsigned int first_entry;
	u32 queue = skb_get_queue_mapping(skb);
	struct x2_tx_queue *tx_q;
	struct dma_desc *desc, *first;
	unsigned int des, enh_desc;
	int i, csum_insert = 0, is_jumbo = 0;
	struct timespec64 now;



	tx_q = &priv->tx_queue[queue];


//	printk("%s, reg-0x718:%d\n",__func__,readl(priv->ioaddr + 0x718));
//	printk("%s, reg-0x71c:%d\n",__func__,readl(priv->ioaddr + 0x71c));
//	printk("%s, reg-0x8ac:%d\n\n",__func__,readl(priv->ioaddr + 0x8ac));
//	printk("%s, reg-0x8d0:%d\n",__func__,readl(priv->ioaddr + 0x8d0));
//	printk("%s, reg-0x8a8:%d\n",__func__,readl(priv->ioaddr + 0x8a8));

#if 0
	x2_est_read(priv, MTL_EST_BTR_LOW, false);
	x2_est_read(priv, MTL_EST_BTR_HIGH, false);

	x2_est_read(priv, MTL_EST_CTR_LOW, false);
	x2_est_read(priv, MTL_EST_CTR_HIGH, false);
	x2_est_read(priv, MTL_EST_TER,  false);
	x2_est_read(priv, MTL_EST_LLR, false);

	printk("%s, current slot number:%d\n", __func__, readl(priv->ioaddr + 0xc58) >> 16 & 0xf);
#endif
#if 0
	cnt++;

	if (cnt > 10) {
		if (queue != 0)	{
			printk("%s,queue:%d\n", __func__,queue);

		}
		cnt = 0;

	}


//	printk("skb len:%d, skb is gso:%d, priv->tso:%d, skb_shinfo->gso_type:0x%x\n",skb->len,skb_is_gso(skb),priv->tso,skb_shinfo(skb)->gso_type);
	if (readl(priv->ioaddr + MTL_ETSX_STATUS_BASE_ADDR(queue)))
		printk("%s, queue:%d, averege bit/ %d slot number: 0x%x\n",__func__,queue,(readl(priv->ioaddr + MTL_ETSX_CTRL_BASE_ADDR(queue)) >> 4 & 0x7),readl(priv->ioaddr + MTL_ETSX_STATUS_BASE_ADDR(queue)));
#endif

#if 0
	printk("%s, regb00:0x%x\n",__func__,readl(priv->ioaddr + 0xb00));
	printk("%s, regb04:0x%x\n",__func__,readl(priv->ioaddr + 0xb04));



	printk("%s, reg-0xb08:%d\n",__func__,readl(priv->ioaddr + 0xb08));
	printk("%s, reg-0xb0c:%d\n",__func__,readl(priv->ioaddr + 0xb0c));
	printk("%s, reg-0xb10:0x%x\n",__func__,readl(priv->ioaddr + 0xb10));
	printk("%s, reg-0xb14:0x%x\n",__func__,readl(priv->ioaddr + 0xb14));
	printk("%s, reg-0xb18:0x%x\n",__func__,readl(priv->ioaddr + 0xb18));
	printk("%s, reg-0xb1c:0x%x\n",__func__,readl(priv->ioaddr + 0xb1c));
	printk("%s, reg-0xb20:0x%x\n",__func__,readl(priv->ioaddr + 0xb20));
	printk("%s, reg-0xb30:0x%x\n",__func__,readl(priv->ioaddr + 0xb30));
	printk("%s, reg-0xb34:0x%x\n",__func__,readl(priv->ioaddr + 0xb34));
	printk("%s, reg-0xb44:0x%x\n",__func__,readl(priv->ioaddr + 0xb40));
	printk("%s, reg-0xb48:0x%x\n",__func__,readl(priv->ioaddr + 0xb48));
	printk("%s, reg-0xb4c:0x%x\n",__func__,readl(priv->ioaddr + 0xb4c));
	printk("%s, reg-0xb50:0x%x\n",__func__,readl(priv->ioaddr + 0xb50));
	printk("%s, reg-0xb54:0x%x\n",__func__,readl(priv->ioaddr + 0xb54));
	printk("%s, reg-0xb58:0x%x\n",__func__,readl(priv->ioaddr + 0xb58));
	printk("%s, reg-0xb5c:0x%x\n",__func__,readl(priv->ioaddr + 0xb5c));
	printk("%s, reg-0xb60:0x%x\n",__func__,readl(priv->ioaddr + 0xb60));
	printk("%s, reg-0xb64:0x%x\n",__func__,readl(priv->ioaddr + 0xb64));
	printk("%s, reg-0xb68:0x%x\n",__func__,readl(priv->ioaddr + 0xb68));

	printk("%s, reg-0xb6c:0x%x\n",__func__,readl(priv->ioaddr + 0xb6c));
	printk("%s, reg-0xb70:0x%x\n",__func__,readl(priv->ioaddr + 0xb70));
	printk("%s, reg-0xbc0:0x%x\n",__func__,readl(priv->ioaddr + 0xbc0));

#endif



	//printk("%s, sec: %d, nsec:%d\n", __func__,readl(priv->ioaddr + PTP_STSR),readl(priv->ioaddr + PTP_STNSR));




	if (skb_is_gso(skb) && priv->tso) {
		if (skb_shinfo(skb)->gso_type & (SKB_GSO_TCPV4 | SKB_GSO_TCPV6)) {

			skb_set_queue_mapping(skb,0);
			return x2_tso_xmit(skb, ndev);
		}
	}

//	printk("%s,and queue:%d\n",__func__,queue);
//	printk("%s, and avali:%d, nfrags:%d, queue:%d\n",__func__, x2_tx_avail(priv, queue), nfrags,queue);
	if ((x2_tx_avail(priv, queue) < nfrags + 1)) {
		if (!netif_tx_queue_stopped(netdev_get_tx_queue(ndev, queue))) {
			netif_tx_stop_queue(netdev_get_tx_queue(priv->dev, queue));
			printk("%s and tx stop queue\n",__func__);
		}
		return NETDEV_TX_BUSY;
	}

	entry = tx_q->cur_tx;
	first_entry = entry;

	//printk("%s,tx queue:%d, entry:%d\n",__func__,queue,entry);
	csum_insert = (skb->ip_summed == CHECKSUM_PARTIAL);

	if (priv->extend_desc)
		desc = (struct dma_desc *)(tx_q->dma_etx + entry);
	else
		desc = tx_q->dma_tx + entry;

	first = desc;
	//tx_q->tx_skbuff[first_entry] = skb;

	enh_desc = priv->plat->enh_desc;

	if (enh_desc) {
		printk("%s, and enh_desc",__func__);
	}

	for (i = 0; i < nfrags; i++) {
		const skb_frag_t *frag = &skb_shinfo(skb)->frags[i];
		int len = skb_frag_size(frag);
		bool last_segment = (i == (nfrags - 1));

		entry = X2_GET_ENTRY(entry, DMA_TX_SIZE);

		if (priv->extend_desc)
			desc = (struct dma_desc *)(tx_q->dma_etx + entry);
		else
			desc = tx_q->dma_tx + entry;

		des = skb_frag_dma_map(priv->device, frag, 0, len, DMA_TO_DEVICE);
		if (dma_mapping_error(priv->device, des)) {
			goto dma_map_err;
		}

		tx_q->tx_skbuff[entry] = NULL;
		tx_q->tx_skbuff_dma[entry].buf = des;

		desc->des0 = cpu_to_le32(des);

		tx_q->tx_skbuff_dma[entry].map_as_page = true;
		tx_q->tx_skbuff_dma[entry].len = len;
		tx_q->tx_skbuff_dma[entry].last_segment = last_segment;

		x2_prepare_tx_desc(desc, 0, len, csum_insert, 1, last_segment, skb->len);

	}

	tx_q->tx_skbuff[entry] = skb;
	entry = X2_GET_ENTRY(entry, DMA_TX_SIZE);
	tx_q->cur_tx = entry;

//	printk("%s, and after cur_tx:%d\n",__func__,tx_q->cur_tx);
	if (netif_msg_pktdata(priv)) {
		void *tx_head;

		if (priv->extend_desc)
			tx_head = (void *) tx_q->dma_etx;
		else
			tx_head = (void *)tx_q->dma_tx;

		printk("frame to be transmited\n");
		//print_pkt(skb->data, skb->len);
	}

	if (x2_tx_avail(priv, queue) <= (MAX_SKB_FRAGS + 1)) { //MAX_SKB_FRAGS:17
		printk("%s, stop transmited packets\n",__func__);
		netif_tx_stop_queue(netdev_get_tx_queue(priv->dev, queue));
	}

	//printk("%s, and csum: %d,and len:%d\n", __func__, csum_insert, skb->len);
	ndev->stats.tx_bytes += skb->len;
	priv->tx_count_frames += nfrags + 1;

	priv->tx_count_frames = 0;

	desc->des2 |= cpu_to_le32(TDES2_INTERRUPT_ON_COMPLETION);


	skb_tx_timestamp(skb);

	if (!is_jumbo) {
		bool last_segment = (nfrags == 0);
		des = dma_map_single(priv->device, skb->data, nopaged_len, DMA_TO_DEVICE);
		if (dma_mapping_error(priv->device, des))
			goto dma_map_err;

		tx_q->tx_skbuff_dma[first_entry].buf = des;
		first->des0 = cpu_to_le32(des);

		tx_q->tx_skbuff_dma[first_entry].len = nopaged_len;
		tx_q->tx_skbuff_dma[first_entry].last_segment = last_segment;

		if ((skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP) && priv->hwts_tx_en) {
			skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
		//	printk("%s,need tx timestamp\n",__func__);
			x2_enable_tx_timestamp(first);

			ktime_get_real_ts64(&now);
			//printk("%s, second:%d, nsec:%d\n", __func__, now.tv_sec, now.tv_nsec);
		}

		x2_prepare_tx_desc(first, 1, nopaged_len, csum_insert, 1, last_segment, skb->len);
		dma_wmb();
	}


	netdev_tx_sent_queue(netdev_get_tx_queue(ndev, queue), skb->len);

	x2_set_tx_tail_ptr(priv->ioaddr, tx_q->tx_tail_addr, queue);

	netif_trans_update(ndev);
//	printk("%s, and after cur_tx:%d\n",__func__,tx_q->cur_tx);

//	printk("%s, sucessfuly\n",__func__);
	return NETDEV_TX_OK;


dma_map_err:
	dev_kfree_skb(skb);
	priv->dev->stats.tx_dropped++;
	return NETDEV_TX_OK;
}


static int x2_ptp_get_ts_config(struct net_device *ndev, struct ifreq *rq)
{
	struct x2_priv *priv = netdev_priv(ndev);
	struct hwtstamp_config *config = &priv->tstamp_config;

	if (!(priv->dma_cap.time_stamp || priv->dma_cap.atime_stamp))
		return -EOPNOTSUPP;

	return copy_to_user(rq->ifr_data, config, sizeof(*config)) ? -EFAULT : 0;

}


static int x2_ptp_set_ts_config(struct net_device *ndev, struct ifreq *ifr)
{
	struct x2_priv *priv = netdev_priv(ndev);
	struct hwtstamp_config config;
	struct timespec64 now;
	u64 temp = 0;
	u32 ptp_v2 = 0;
	u32 tstamp_all = 0;
	u32 ptp_over_ipv4_udp = 0;
	u32 ptp_over_ipv6_udp = 0;
	u32 ptp_over_ethernet = 0;
	u32 snap_type_sel = 0;
	u32 ts_master_en = 0;
	u32 ts_event_en = 0;
	u32 sec_inc = 0;
	u32 value = 0;
	bool xmac;

	xmac = 1;

	if (!(priv->dma_cap.time_stamp || priv->adv_ts)) {
		printk("No support for HW time stamping\n");
		priv->hwts_tx_en = 0;
		priv->hwts_rx_en = 0;
		return -EOPNOTSUPP;
	}

	if (copy_from_user(&config, ifr->ifr_data, sizeof(config)))
		return -EFAULT;


//	printk("%s, config flags: 0x%x, tx_type:0x%x, rx_filter:0x%x\n",__func__,config.flags, config.tx_type, config.rx_filter);

	if (config.flags)
		return -EINVAL;

	if (config.tx_type != HWTSTAMP_TX_OFF && config.tx_type != HWTSTAMP_TX_ON)
		return -ERANGE;


	if (priv->adv_ts) {

		switch (config.rx_filter) {
		case HWTSTAMP_FILTER_NONE:
			config.rx_filter = HWTSTAMP_FILTER_NONE;
			break;
		case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
			config.rx_filter = HWTSTAMP_FILTER_PTP_V1_L4_EVENT;
			snap_type_sel = PTP_GMAC4_TCR_SNAPTYPSEL_1;
			ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
			ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
			break;
		case HWTSTAMP_FILTER_PTP_V1_L4_SYNC:
			config.rx_filter = HWTSTAMP_FILTER_PTP_V1_L4_SYNC;
			ts_event_en = PTP_TCR_TSEVNTENA;
			ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
			ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
			break;

		case HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ:
			config.rx_filter = HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ;
			ts_master_en = PTP_TCR_TSMSTRENA;
			ts_event_en = PTP_TCR_TSEVNTENA;
			ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
			ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
			break;
		case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
			config.rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_EVENT;
			ptp_v2 = PTP_TCR_TSVER2ENA;
			snap_type_sel = PTP_GMAC4_TCR_SNAPTYPSEL_1;
			ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
			ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
			break;
		case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
			config.rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_SYNC;
			ptp_v2 = PTP_TCR_TSVER2ENA;
			ts_event_en = PTP_TCR_TSEVNTENA;
			ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
			ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
			break;

		case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
			config.rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ;
			ptp_v2 = PTP_TCR_TSVER2ENA;
			ts_master_en = PTP_TCR_TSMSTRENA;
			ptp_v2 = PTP_TCR_TSVER2ENA;
			ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
			ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
			break;
		case HWTSTAMP_FILTER_PTP_V2_EVENT:
			config.rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
			ptp_v2 = PTP_TCR_TSVER2ENA;
			snap_type_sel = PTP_GMAC4_TCR_SNAPTYPSEL_1;
			ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
			ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
			ptp_over_ethernet = PTP_TCR_TSIPENA;
			break;

		case HWTSTAMP_FILTER_PTP_V2_SYNC:
			config.rx_filter = HWTSTAMP_FILTER_PTP_V2_SYNC;
			ptp_v2 = PTP_TCR_TSVER2ENA;
			ts_event_en = PTP_TCR_TSEVNTENA;
			ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
			ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
			ptp_over_ethernet = PTP_TCR_TSIPENA;
			break;

		case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
			config.rx_filter = HWTSTAMP_FILTER_PTP_V2_SYNC;
			ts_master_en = PTP_TCR_TSMSTRENA;
			ptp_v2 = PTP_TCR_TSVER2ENA;
			ts_event_en = PTP_TCR_TSEVNTENA;
			ptp_over_ipv4_udp = PTP_TCR_TSIPV4ENA;
			ptp_over_ipv6_udp = PTP_TCR_TSIPV6ENA;
			ptp_over_ethernet = PTP_TCR_TSIPENA;
			break;
		case HWTSTAMP_FILTER_NTP_ALL:
		case HWTSTAMP_FILTER_ALL:
			config.rx_filter = HWTSTAMP_FILTER_ALL;
			tstamp_all = PTP_TCR_TSENALL;
			break;
		default:
			return -ERANGE;


		}

	} else {

		switch (config.rx_filter) {
		case HWTSTAMP_FILTER_NONE:
			config.rx_filter = HWTSTAMP_FILTER_NONE;
			break;
		default:
			config.rx_filter = HWTSTAMP_FILTER_PTP_V1_L4_EVENT;
			break;
		}

	}




	priv->hwts_rx_en = ((config.rx_filter == HWTSTAMP_FILTER_NONE) ? 0 : 1);
	priv->hwts_tx_en = config.tx_type == HWTSTAMP_TX_ON;

	if (!priv->hwts_tx_en && !priv->hwts_rx_en)
		x2_config_hw_tstamping(priv, 0);
	else {

		x2_config_hw_tstamping(priv, 0);

	//	value = (PTP_TCR_TSENA|PTP_TCR_TSCFUPDT|PTP_TCR_TSCTRLSSR|tstamp_all|ptp_v2|ptp_over_ethernet|ptp_over_ipv6_udp|ptp_over_ipv4_udp|ts_event_en|ts_master_en|snap_type_sel);
		value = (PTP_TCR_TSENA|PTP_TCR_TSCTRLSSR|tstamp_all|ptp_v2|ptp_over_ethernet|ptp_over_ipv6_udp|ptp_over_ipv4_udp|ts_event_en|ts_master_en|snap_type_sel);
		value |= (1<<19) | (1<<14);
		value &= ~(1 << 17);
		value |= 1<<1;

//		printk("%s,and set value is 0x%x\n",__func__,value);
		x2_config_hw_tstamping(priv, value);


		sec_inc = x2_config_sub_second_increment(priv,priv->plat->clk_ptp_rate, xmac);
		temp = div_u64(1000000000ULL, sec_inc);
		priv->sub_second_inc = sec_inc;
		priv->systime_flags = value;
		temp = (u64) (temp << 32);
		priv->default_addend = div_u64(temp, priv->plat->clk_ptp_rate);
		priv->default_addend = 0xCCCCB8CD;//div_u64(temp, priv->plat->clk_ptp_rate);

		x2_config_addend(priv,  priv->default_addend);

		ktime_get_real_ts64(&now);
	//	printk("%s,and now secod: %d, and nsec:%d\n", __func__, now.tv_sec, now.tv_nsec);
		x2_init_systime(priv,  (u32)now.tv_sec, now.tv_nsec);


	}


//	printk("%s, regb00:0x%x\n",__func__,readl(priv->ioaddr + 0xb00));
//	printk("%s, regb04:0x%x\n",__func__,readl(priv->ioaddr + 0xb04));



#if 0
	printk("%s, reg-0xb08:%d\n",__func__,readl(priv->ioaddr + 0xb08));
	printk("%s, reg-0xb0c:%d\n",__func__,readl(priv->ioaddr + 0xb0c));
	printk("%s, reg-0xb10:0x%x\n",__func__,readl(priv->ioaddr + 0xb10));
	printk("%s, reg-0xb14:0x%x\n",__func__,readl(priv->ioaddr + 0xb14));
	printk("%s, reg-0xb18:0x%x\n",__func__,readl(priv->ioaddr + 0xb18));
	printk("%s, reg-0xb1c:0x%x\n",__func__,readl(priv->ioaddr + 0xb1c));
	printk("%s, reg-0xb20:0x%x\n",__func__,readl(priv->ioaddr + 0xb20));
	printk("%s, reg-0xb30:0x%x\n",__func__,readl(priv->ioaddr + 0xb30));
	printk("%s, reg-0xb34:0x%x\n",__func__,readl(priv->ioaddr + 0xb34));
	printk("%s, reg-0xb44:0x%x\n",__func__,readl(priv->ioaddr + 0xb40));
	printk("%s, reg-0xb48:0x%x\n",__func__,readl(priv->ioaddr + 0xb48));
	printk("%s, reg-0xb4c:0x%x\n",__func__,readl(priv->ioaddr + 0xb4c));
	printk("%s, reg-0xb50:0x%x\n",__func__,readl(priv->ioaddr + 0xb50));
	printk("%s, reg-0xb54:0x%x\n",__func__,readl(priv->ioaddr + 0xb54));
	printk("%s, reg-0xb58:0x%x\n",__func__,readl(priv->ioaddr + 0xb58));
	printk("%s, reg-0xb5c:0x%x\n",__func__,readl(priv->ioaddr + 0xb5c));
	printk("%s, reg-0xb60:0x%x\n",__func__,readl(priv->ioaddr + 0xb60));
	printk("%s, reg-0xb64:0x%x\n",__func__,readl(priv->ioaddr + 0xb64));
	printk("%s, reg-0xb68:0x%x\n",__func__,readl(priv->ioaddr + 0xb68));

	printk("%s, reg-0xb6c:0x%x\n",__func__,readl(priv->ioaddr + 0xb6c));
	printk("%s, reg-0xb70:0x%x\n",__func__,readl(priv->ioaddr + 0xb70));
	printk("%s, reg-0xbc0:0x%x\n",__func__,readl(priv->ioaddr + 0xbc0));

#endif


	memcpy(&priv->tstamp_config, &config, sizeof(config));
	return copy_to_user(ifr->ifr_data, &config, sizeof(config)) ? -EFAULT: 0;
}


static int x2_ioctl(struct net_device *ndev, struct ifreq *rq, int cmd)
{
	struct x2_priv *priv = netdev_priv(ndev);
	int ret = -EOPNOTSUPP;

	if (!netif_running(ndev))
		return -EINVAL;

	switch (cmd) {
	case SIOCSTIOCTL:
		if (!priv || !rq)
			return -EINVAL;

		ret = x2_extension_ioctl(priv, rq->ifr_data);
		break;
//	case SIOCGHWTSTAMP:
//		return x2_ptp_get_ts_config(ndev, rq);
	case SIOCSHWTSTAMP:
		return x2_ptp_set_ts_config(ndev, rq);
	default:
		break;
	}

	return ret;
}

static void x2_set_rx_mode(struct net_device *ndev)
{
	struct x2_priv *priv = netdev_priv(ndev);
	void __iomem *ioaddr = priv->ioaddr;
	unsigned int value = 0;

	if (ndev->flags & IFF_PROMISC) {
		value = GMAC_PACKET_FILTER_PR;
	} else if ((ndev->flags & IFF_ALLMULTI) || (netdev_mc_count(ndev) > HASH_TABLE_SIZE)) {
		value = GMAC_PACKET_FILTER_PM;
		writel(0xffffffff, ioaddr + GMAC_HASH_TAB_0_31);
		writel(0xffffffff, ioaddr + GMAC_HASH_TAB_32_63);
	} else if (!netdev_mc_empty(ndev)) {
		u32 mc_filter[2];
		struct netdev_hw_addr *ha;

		value = GMAC_PACKET_FILTER_HMC;
		memset(mc_filter, 0, sizeof(mc_filter));
		netdev_for_each_mc_addr(ha, ndev) {
			int bit_nr = (bitrev32(~crc32_le(~0, ha->addr, 6)) >> 26);
			mc_filter[bit_nr >> 5] |= (1 << (bit_nr & 0x1F));
		}

		writel(mc_filter[0], ioaddr + GMAC_HASH_TAB_0_31);
		writel(mc_filter[1], ioaddr + GMAC_HASH_TAB_32_63);
	}

	if (netdev_uc_count(ndev) > GMAC_MAX_PERFECT_ADDRESSES) {

		value |= GMAC_PACKET_FILTER_PR;
	}else if (!netdev_uc_empty(ndev)) {
		int reg = 1;
		struct netdev_hw_addr *ha;

		netdev_for_each_uc_addr(ha, ndev) {
			x2_set_umac_addr(priv->ioaddr, ha->addr, reg);
			reg++;
		}
	}

	writel(value, ioaddr + GMAC_PACKET_FILTER);
}

static const struct net_device_ops x2_netdev_ops = {
	.ndo_open = x2_open,
	.ndo_stop = x2_release,
	.ndo_start_xmit = x2_xmit,
	.ndo_do_ioctl = x2_ioctl,
	.ndo_set_rx_mode = x2_set_rx_mode,
	.ndo_set_mac_address = eth_mac_addr,

	.ndo_tsn_capable = x2_tsn_capable,
	.ndo_tsn_link_configure = x2_tsn_link_configure,
	.ndo_select_queue = x2_tsn_select_queue,
};

static void x2_get_drv_info(struct net_device *ndev, struct ethtool_drvinfo *ed)
{
	const struct x2_priv *priv = netdev_priv(ndev);
	strcpy(ed->driver, priv->device->driver->name);
	strcpy(ed->version, DRIVER_VERSION);
}


static void x2_get_strings(struct net_device *ndev, u32 stringset, u8 *data)
{
	size_t i;
	if (stringset != ETH_SS_STATS)
		return;

	for (i = 0; i < STMMAC_STATS_LEN; ++i) {
		memcpy(data, stmmac_gstrings_stats[i].stat_name, ETH_GSTRING_LEN);
		data += ETH_GSTRING_LEN;
	}
}


static void x2_get_ethtool_stats(struct net_device *ndev, struct ethtool_stats *dummy, u64 *data)
{
	struct x2_priv *priv = netdev_priv(ndev);
	//u32 rx_cnt = priv->plat->rx_queues_to_use;
	//u32 tx_cnt = priv->plat->tx_queues_to_use;
	int i, j = 0;

	for (i = 0; i < STMMAC_STATS_LEN; i++) {
		char *p = (char *)priv + stmmac_gstrings_stats[i].stat_offset;
		data[j++] = (stmmac_gstrings_stats[i].sizeof_stat == sizeof(u64)) ? (*(u64*)p) : (*(u32*)p);
	}
}


static int x2_ethtool_get_regs_len(struct net_device *ndev)
{

	return REG_SPACE_SIZE;
}

static void x2_dump_mac_regs(void __iomem *ioaddr, u32 *reg_space)
{
	int i;

	for (i = 0; i < GMAC_REG_NUM; i++)
		reg_space[i] = readl(ioaddr + i * 4);
}

static void __x2_dump_dma_regs(void __iomem *ioaddr, u32 channel, u32 *reg_space)
{
	reg_space[DMA_CHAN_CONTROL(channel) / 4] = readl(ioaddr + DMA_CHAN_CONTROL(channel));
	reg_space[DMA_CHAN_TX_CONTROL(channel) / 4] = readl(ioaddr + DMA_CHAN_TX_CONTROL(channel));

	reg_space[DMA_CHAN_RX_CONTROL(channel) / 4]= readl(ioaddr + DMA_CHAN_RX_CONTROL(channel));

	reg_space[DMA_CHAN_TX_BASE_ADDR(channel) / 4] = readl(ioaddr + DMA_CHAN_TX_BASE_ADDR(channel));
	reg_space[DMA_CHAN_RX_BASE_ADDR(channel) / 4] = readl(ioaddr + DMA_CHAN_RX_BASE_ADDR(channel));

	reg_space[DMA_CHAN_TX_END_ADDR(channel) / 4] = readl(ioaddr + DMA_CHAN_TX_END_ADDR(channel));
	reg_space[DMA_CHAN_RX_END_ADDR(channel) / 4] = readl(ioaddr + DMA_CHAN_RX_END_ADDR(channel));

	reg_space[DMA_CHAN_TX_RING_LEN(channel) / 4] = readl(ioaddr + DMA_CHAN_TX_RING_LEN(channel));
	reg_space[DMA_CHAN_RX_RING_LEN(channel) / 4] = readl(ioaddr + DMA_CHAN_RX_RING_LEN(channel));

	reg_space[DMA_CHAN_INTR_ENA(channel) / 4] = readl(ioaddr + DMA_CHAN_INTR_ENA(channel));

	reg_space[DMA_CHAN_RX_WATCHDOG(channel) / 4] = readl(ioaddr + DMA_CHAN_RX_WATCHDOG(channel));

	reg_space[DMA_CHAN_SLOT_CTRL_STATUS(channel) / 4] = readl(ioaddr + DMA_CHAN_SLOT_CTRL_STATUS(channel));

	reg_space[DMA_CHAN_CUR_TX_DESC(channel) / 4] = readl(ioaddr + DMA_CHAN_CUR_TX_DESC(channel));
	reg_space[DMA_CHAN_CUR_RX_DESC(channel) / 4] = readl(ioaddr + DMA_CHAN_CUR_RX_DESC(channel));

	reg_space[DMA_CHAN_CUR_TX_BUF_ADDR(channel) / 4] = readl(ioaddr + DMA_CHAN_CUR_TX_BUF_ADDR(channel));
	reg_space[DMA_CHAN_CUR_RX_BUF_ADDR(channel) / 4] = readl(ioaddr + DMA_CHAN_CUR_RX_BUF_ADDR(channel));

	reg_space[DMA_CHAN_STATUS(channel) / 4] = readl(ioaddr + DMA_CHAN_STATUS(channel));
}

static void  x2_dump_dma_regs(void __iomem *ioaddr, u32 *reg_space)
{
	int i;

	for (i = 0; i < DMA_CHANNEL_NB_MAX; i++)
		__x2_dump_dma_regs(ioaddr, i, reg_space);
}


static void x2_ethtool_gregs(struct net_device *ndev, struct ethtool_regs *regs, void *space)
{
	u32 *reg_space = (u32 *)space;
	struct x2_priv *priv = netdev_priv(ndev);
	int i;

	memset(reg_space, 0x0, REG_SPACE_SIZE);

	//for (i = 0; i < GMAC_REG_NUM; i++)
	for (i = 0; i < 0x1368 / 4; i++)
		reg_space[i] = readl(priv->ioaddr + i * 4);

//	x2_dump_mac_regs(priv->ioaddr, reg_space);
//	x2_dump_dma_regs(priv->ioaddr, reg_space);
//	memcpy(&reg_space[ETHTOOL_DMA_OFFSET], &reg_space[DMA_BUS_MODE/4], NUM_DWMAC1000_DMA_REGS * 4);

}

static int x2_get_ts_info(struct net_device *dev, struct ethtool_ts_info *info)
{
	struct x2_priv *priv = netdev_priv(dev);

	if ((priv->dma_cap.time_stamp || priv->dma_cap.atime_stamp)) {
		info->so_timestamping = SOF_TIMESTAMPING_TX_SOFTWARE | SOF_TIMESTAMPING_TX_HARDWARE|
					SOF_TIMESTAMPING_RX_SOFTWARE | SOF_TIMESTAMPING_RX_HARDWARE|
					SOF_TIMESTAMPING_SOFTWARE | SOF_TIMESTAMPING_RAW_HARDWARE;

		if (priv->ptp_clock)
			info->phc_index = ptp_clock_index(priv->ptp_clock);
		info->tx_types = (1 << HWTSTAMP_TX_OFF) | (1 << HWTSTAMP_TX_ON);
		info->rx_filters = ((1<<HWTSTAMP_FILTER_NONE) | (1<<HWTSTAMP_FILTER_PTP_V1_L4_EVENT)|
				 (1<<HWTSTAMP_FILTER_PTP_V1_L4_SYNC)|
				 (1<<HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ)|
				 (1<<HWTSTAMP_FILTER_PTP_V2_L4_EVENT)|
				 (1<<HWTSTAMP_FILTER_PTP_V2_L4_SYNC)|
				 (1<<HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ)|
				 (1<<HWTSTAMP_FILTER_PTP_V2_EVENT)|
				 (1<<HWTSTAMP_FILTER_PTP_V2_SYNC)|
				 (1<<HWTSTAMP_FILTER_PTP_V2_DELAY_REQ)|
				 (1<<HWTSTAMP_FILTER_ALL));
		return 0;
	} else
		return ethtool_op_get_ts_info(dev, info);
}


static const struct ethtool_ops x2_ethtool_ops = {
	.get_drvinfo = x2_get_drv_info,
	.get_link = ethtool_op_get_link,
	.get_link_ksettings = phy_ethtool_get_link_ksettings,
	.set_link_ksettings = phy_ethtool_set_link_ksettings,
//	.get_strings = x2_get_strings,
	.get_ethtool_stats = x2_get_ethtool_stats,
	.get_regs_len = x2_ethtool_get_regs_len,
	.get_regs = x2_ethtool_gregs,
	.get_ts_info = x2_get_ts_info,

};

static int x2_tx_status(struct net_device_stats *data, struct x2_extra_stats *x, struct dma_desc *p, void __iomem *ioaddr)
{
	struct net_device_stats *stats = (struct net_device_stats *)data;
	unsigned int tdes3;
	int ret = tx_done;

	tdes3 = le32_to_cpu(p->des3);

	if (tdes3 & TDES3_OWN)
		return tx_dma_own;

	if (!(tdes3 & TDES3_LAST_DESCRIPTOR)) {
	//	printk("%s, and 1...\n",__func__);
		return tx_not_ls;
	}
	if (tdes3 & TDES3_ERROR_SUMMARY) {
		if (tdes3 & TDES3_JABBER_TIMEOUT){

			printk("%s, and 2...\n",__func__);
			x->tx_jabber++;
		}
		if (tdes3 & TDES3_PACKET_FLUSHED) {

			printk("%s, and 3...\n",__func__);
			x->tx_frame_flushed++;
		}

		if (tdes3 & TDES3_LOSS_CARRIER) {

			printk("%s, and 4...\n",__func__);
			x->tx_losscarrier++;
			stats->tx_carrier_errors++;
		}

		if (tdes3 & TDES3_NO_CARRIER) {

			printk("%s, and 5...\n",__func__);
			x->tx_carrier++;
			stats->tx_carrier_errors++;
		}

		if ((tdes3 & TDES3_LATE_COLLISION) || (tdes3 & TDES3_EXCESSIVE_COLLISION)) {

			printk("%s, and 6...\n",__func__);
			stats->collisions += (tdes3 & TDES3_COLLISION_COUNT_MASK) >> TDES3_COLLISION_COUNT_SHIFT;
		}

		if (tdes3 & TDES3_UNDERFLOW_ERROR) {

			printk("%s, and 7...\n",__func__);
			x->tx_underflow++;
		}

		if (tdes3 & TDES3_IP_HDR_ERROR) {

			printk("%s, and 8...\n",__func__);
			x->tx_ip_header_error++;
		}

		if (tdes3 & TDES3_PAYLOAD_ERROR) {

			printk("%s, and 9...\n",__func__);
			x->tx_payload_error++;
		}

		ret = tx_err;
	}

	if (tdes3 & TDES3_DEFERRED) {

		printk("%s, and 10...\n",__func__);
		x->tx_deferred++;
	}

	return ret;
}

static int x2_get_tx_timestamp_status(struct dma_desc *p)
{
	if (le32_to_cpu(p->des3) & TDES3_CONTEXT_TYPE)
		return 0;

	if (le32_to_cpu(p->des3) & TDES3_TIMESTAMP_STATUS)
		return 1;

	return 0;
}

static void x2_get_tx_hwstamp(struct x2_priv *priv, struct dma_desc *p, struct sk_buff *skb)
{
	struct skb_shared_hwtstamps shhwtstamp;
	u64 ns;
	int status;

	if (!priv->hwts_tx_en)
		return;

	if (!skb || !((skb_shinfo(skb)->tx_flags & SKBTX_IN_PROGRESS)))
		return;

	status = x2_get_tx_timestamp_status(p);
	if (status) {
		ns = le32_to_cpu(p->des0);// x2_get_timestamp(p, priv->adv_ts);
		ns += le32_to_cpu(p->des1) * 1000000000ULL;

		memset(&shhwtstamp, 0, sizeof(struct skb_shared_hwtstamps));
		shhwtstamp.hwtstamp = ns_to_ktime(ns);


	//	printk("%s, and ns:%d, hwtstamp:0x%x\n", __func__,ns, shhwtstamp.hwtstamp);
		skb_tstamp_tx(skb, &shhwtstamp);
	}

	return;
}

//static int cnt_tx = 0;

static void x2_tx_clean(struct x2_priv *priv, u32 queue)
{
	struct x2_tx_queue *tx_q = &priv->tx_queue[queue];
	unsigned int bytes_compl = 0, pkts_compl = 0;
	unsigned int entry;

	netif_tx_lock(priv->dev);

	priv->xstats.tx_clean++;
	entry = tx_q->dirty_tx;

//	printk("%s and queue:%d, dirty_tx:%d\n",__func__,queue,entry);
	while(entry != tx_q->cur_tx) {
		struct sk_buff *skb = tx_q->tx_skbuff[entry];
		struct dma_desc *p;
		int status;

		if (priv->extend_desc) {
			p = (struct dma_desc*)(tx_q->dma_etx + entry);
		} else
			p = tx_q->dma_tx + entry;


#if 0
		printk("%s, entry:%d, queue: %d, tx_q->cur_tx:%d, p->des3:0x%x\n",__func__,entry,queue,tx_q->cur_tx,p->des3);

		printk("%s,dma_debug0:0x%llx\n",__func__, readl(priv->ioaddr + 0x100c));
		printk("%s,dma_debug1:0x%llx\n",__func__, readl(priv->ioaddr + 0x1010));
		printk("%s,dma_debug2:0x%llx\n",__func__, readl(priv->ioaddr + 0x1014));
		printk("%s,MTL_tx0_debug:0x%llx\n",__func__, readl(priv->ioaddr + 0xd08));
#endif

		status = x2_tx_status(&priv->dev->stats, &priv->xstats, p, priv->ioaddr);
	//	printk("%s, queue:%d\n",__func__,queue);

		if (status & tx_dma_own) {
		//	printk("%s,queue:%d, status: 0x%x, and hw own tx dma\n",__func__,queue,status);
		//	printk("%s, and queue is stopped ?,  val:%d\n",__func__,netif_queue_stopped(priv->dev));
			break;
		}

#if 0
		cnt_tx++;

		if (cnt_tx > 50) {
			printk("%s, mtl fpe ctrl sts:0x%x\n", __func__, readl(priv->ioaddr + 0xc90));
			printk("%s, frg cntr:%d, hlod:%d\n", __func__, readl(priv->ioaddr + MMC_TX_FPE_Frg_Cntr),readl(priv->ioaddr + MMC_TX_HOLD_Req_Cntr));
			cnt_tx = 0;
		}

#endif
		dma_rmb();

		if (!(status & tx_not_ls)) {
			if (status & tx_err) {
				priv->dev->stats.tx_errors++;
			} else
				priv->dev->stats.tx_packets++;

			x2_get_tx_hwstamp(priv, p, skb);
		}

		if (tx_q->tx_skbuff_dma[entry].buf) {
			if (tx_q->tx_skbuff_dma[entry].map_as_page)
				dma_unmap_page(priv->device, tx_q->tx_skbuff_dma[entry].buf, tx_q->tx_skbuff_dma[entry].len, DMA_TO_DEVICE);
			else
				dma_unmap_single(priv->device, tx_q->tx_skbuff_dma[entry].buf, tx_q->tx_skbuff_dma[entry].len, DMA_TO_DEVICE);

			tx_q->tx_skbuff_dma[entry].buf = 0;
			tx_q->tx_skbuff_dma[entry].len = 0;
			tx_q->tx_skbuff_dma[entry].map_as_page = false;
		}

		tx_q->tx_skbuff_dma[entry].last_segment = false;
		tx_q->tx_skbuff_dma[entry].is_jumbo = false;

		if (skb != NULL) {
			pkts_compl++;
			bytes_compl += skb->len;
			dev_consume_skb_any(skb);
			tx_q->tx_skbuff[entry] = NULL;
		}

		p->des2 = 0;
		p->des3 = 0;//	x2_release_tx_desc(p);
		entry = X2_GET_ENTRY(entry, DMA_TX_SIZE);
	}

	tx_q->dirty_tx = entry;

	netdev_tx_completed_queue(netdev_get_tx_queue(priv->dev, queue),pkts_compl, bytes_compl);

	if (netif_tx_queue_stopped(netdev_get_tx_queue(priv->dev,queue)) && x2_tx_avail(priv, queue) > STMMAC_TX_THRESH) {
		printk("%s: restart transmit\n",__func__);
		netif_tx_wake_queue(netdev_get_tx_queue(priv->dev, queue));
	}

	netif_tx_unlock(priv->dev);
}


static int x2_get_rx_status(void *data, struct x2_extra_stats *x, struct dma_desc *p)
{
	struct net_device_stats *stats = (struct net_device_stats *)data;
	//unsigned int rdes0 = le32_to_cpu(p->des0);
	unsigned int rdes1 = le32_to_cpu(p->des1);
	unsigned int rdes2 = le32_to_cpu(p->des2);
	unsigned int rdes3 = le32_to_cpu(p->des3);

	int message_type;
	int ret = good_frame;

//	printk("%s,and rdes0:0x%x, rdes1:0x%x, rdes2:0x%x, rdes3:0x%x\n",__func__, rdes0,rdes1, rdes2, rdes3);
	if (rdes3 & RDES3_OWN)
		return dma_own;

	if (!(rdes3 & RDES3_LAST_DESCRIPTOR)) {
	//	printk("%s, 1.....\n",__func__);
		return discard_frame;
	}
	if (rdes3 &  RDES3_ERROR_SUMMARY) {
		if (rdes3 & RDES3_GIANT_PACKET) {

			printk("%s, 2.....\n",__func__);
			stats->rx_length_errors++;
		}
		if (rdes3 & RDES3_OVERFLOW_ERROR) {

			printk("%s, 3 overflow\n",__func__);
			x->rx_gmac_overflow++;
		}

		if (rdes3 & RDES3_RECEIVE_WATCHDOG) {

			printk("%s, 4.....\n",__func__);
			x->rx_watchdog++;
		}
		if (rdes3 & RDES3_RECEIVE_ERROR) {

			printk("%s, 5.....\n",__func__);
			x->rx_mii++;
		}

		if (rdes3 & RDES3_CRC_ERROR) {

			printk("%s, 6.....\n",__func__);
			x->rx_crc_errors++;
			stats->rx_crc_errors++;
		}

		if (rdes3 & RDES3_DRIBBLE_ERROR) {

			printk("%s, 7.....\n",__func__);
			x->dribbling_bit++;
		}

		printk("%s, 8.....\n",__func__);
		ret = discard_frame;
	}

	message_type = (rdes1 & ERDES4_MSG_TYPE_MASK) >> 8;

//	printk("%s, and message_type:0x%x\n",__func__,message_type);

//	printk("%s,payloadtype:0x%x\n", __func__,rdes1 & 0x7);

	if (rdes1 & RDES1_IP_HDR_ERROR) {

		printk("%s, 9.....\n",__func__);
		x->ip_hdr_err++;
	}

	if (rdes1 & RDES1_IP_CSUM_BYPASSED) {
		printk("%s, 10.....\n",__func__);
		x->ip_csum_bypassed++;
	}

	if (rdes1 & RDES1_IPV4_HEADER) {

		printk("%s, 11.....\n",__func__);
		x->ipv4_pkt_rcvd++;
	}

	if (rdes1 & RDES1_IPV6_HEADER) {

		printk("%s, 12.....\n",__func__);
		x->ipv6_pkt_rcvd++;
	}




	if (message_type == RDES_EXT_NO_PTP) {

	//	printk("%s, 13.....\n",__func__);
		x->no_ptp_rx_msg_type_ext++;

	} else if (message_type == RDES_EXT_SYNC) {
	//	printk("%s, ptp rx sync\n",__func__);
		x->ptp_rx_msg_type_sync++;

	}else if (message_type == RDES_EXT_DELAY_REQ) {

	//	printk("%s, ptp delay req\n",__func__);
		x->ptp_rx_msg_type_delay_req++;

	}else if (message_type == RDES_EXT_FOLLOW_UP) {

	//	printk("%s, ptp follow up\n",__func__);
		x->ptp_rx_msg_type_follow_up++;
	} else if (message_type == RDES_EXT_DELAY_RESP) {

		//printk("%s, ptp delay resp\n",__func__);
		x->ptp_rx_msg_type_delay_resp++;

	}else if (message_type == RDES_EXT_PDELAY_REQ) {

	//	printk("%s, ptp pdelay_req\n",__func__);
		x->ptp_rx_msg_type_pdelay_req++;

	}else if (message_type == RDES_EXT_PDELAY_RESP) {

	//	printk("%s, ptp pdelay_resp\n",__func__);
		x->ptp_rx_msg_type_pdelay_resp++;
	}
	else if (message_type == RDES_EXT_PDELAY_FOLLOW_UP) {

	//	printk("%s, ptp pdelay_follow up\n",__func__);
		x->ptp_rx_msg_type_pdelay_follow_up++;
	}
	else if (message_type == RDES_PTP_ANNOUNCE) {

	//	printk("%s, ptp rx announce\n",__func__);
		x->ptp_rx_msg_type_announce++;
	}
	else if (message_type == RDES_PTP_MANAGEMENT) {

	//	printk("%s, ptp  management\n",__func__);
		x->ptp_rx_msg_type_management++;
	} else if (message_type == RDES_PTP_PKT_RESERVED_TYPE) {

	//	printk("%s, ptp rx reserved\n",__func__);
		x->ptp_rx_msg_pkt_reserved_type++;
	}


	if (rdes1 & RDES1_PTP_PACKET_TYPE) {

	//	printk("%s, ptp frame\n",__func__);
		x->ptp_frame_type++;
	}

	if (rdes1 & RDES1_PTP_VER) {

		//printk("%s, ptp ver1\n",__func__);
		x->ptp_ver++;
	}
	if (rdes1 & RDES1_TIMESTAMP_DROPPED)
		x->timestamp_dropped++;

	if (rdes2 & RDES2_SA_FILTER_FAIL) {
		x->sa_rx_filter_fail++;
		ret = discard_frame;
	}

	if (rdes2 & RDES2_L3_FILTER_MATCH)
		x->l3_filter_match++;

	if (rdes2 & RDES2_L4_FILTER_MATCH)
		x->l4_filter_match++;

	if ((rdes2 & RDES2_L3_L4_FILT_NB_MATCH_MASK) >> RDES2_L3_L4_FILT_NB_MATCH_SHIFT)
		x->l3_l4_filter_no_match++;

//	printk("%s,and return ret:%d\n",__func__,ret);
	return ret;
}

static int x2_rx_check_timestamp(void *desc)
{
	struct dma_desc *p = (struct dma_desc *)desc;
	unsigned int rdes0 = le32_to_cpu(p->des0);
	unsigned int rdes1 = le32_to_cpu(p->des1);
	unsigned int rdes3 = le32_to_cpu(p->des3);

	u32 own, ctxt;
	int ret = 1;

	own = rdes3 & RDES3_OWN;
	ctxt = ((rdes3 & RDES3_CONTEXT_DESCRIPTOR) >> RDES3_CONTEXT_DESCRIPTOR_SHIFT);

	if (likely(!own && ctxt)) {
		if ((rdes0 == 0xffffffff) && (rdes1 == 0xffffffff))
			ret = -EINVAL;
		else
			ret = 0;
	}

	return ret;
}

static int x2_get_rx_timestamp_status(void *desc, void *next_desc, u32 ats)
{
	struct dma_desc *p = (struct dma_desc *)desc;
	int ret = -EINVAL;

	if (likely(le32_to_cpu(p->des3) & RDES3_RDES1_VALID)) {
		if (likely(le32_to_cpu(p->des1) & RDES1_TIMESTAMP_AVAILABLE)) {
			int i = 0;

			do {
				ret = x2_rx_check_timestamp(next_desc);
				if (ret < 0)
					goto exit;

				i++;
			}while ((ret == 1) && (i < 10));

			if (i == 10)
				ret = -EBUSY;
		}
	}

exit:
	if (likely(ret == 0))
		return 1;
	return 0;
}
static void x2_get_rx_hwtstamp(struct x2_priv *priv, struct dma_desc *p, struct dma_desc *np, struct sk_buff *skb)
{
	struct skb_shared_hwtstamps *shhwtstamp = NULL;
	struct dma_desc *desc = p;
	u64 ns;

	if (!priv->hwts_rx_en)
		return;

	desc = np;

	if (x2_get_rx_timestamp_status(p, np, priv->adv_ts)) {
	//	printk("get rx hwstamp: des0:0x%x, des1:0x%x\n",desc->des0,desc->des1);
		ns = le32_to_cpu(desc->des0);
		ns += le32_to_cpu(desc->des1) * 1000000000ULL; //very very very very be carefully & *
		shhwtstamp = skb_hwtstamps(skb);
		memset(shhwtstamp, 0, sizeof(struct skb_shared_hwtstamps));
		shhwtstamp->hwtstamp = ns_to_ktime(ns);
//		printk("%s:  second:%d, and ns:%d, hwtstamp:0x%x\n", __func__,le32_to_cpu(desc->des1), ns, shhwtstamp->hwtstamp);
	} else {
		//printk("cannot get RX hw timestamp\n");
	}

}

static void x2_rx_vlan(struct net_device *ndev, struct sk_buff *skb)
{
	struct ethhdr *ehdr;
	u16 vlanid;

	if ((ndev->features & NETIF_F_HW_VLAN_CTAG_RX) == NETIF_F_HW_VLAN_CTAG_RX && !__vlan_get_tag(skb, &vlanid)) {
		ehdr = (struct ethhdr *)skb->data;
		memmove(skb->data + VLAN_HLEN, ehdr, ETH_ALEN*2);
		skb_pull(skb, VLAN_HLEN);
		printk("%s, vlanid:0x%x\n", __func__,vlanid);
		__vlan_hwaccel_put_tag(skb, htons(ETH_P_8021Q),vlanid);
	}
}

static inline u32 x2_rx_dirty(struct x2_priv *priv, u32 queue)
{
	struct x2_rx_queue *rx_q = &priv->rx_queue[queue];
	u32 dirty;

	if (rx_q->dirty_rx <= rx_q->cur_rx)
		dirty = rx_q->cur_rx - rx_q->dirty_rx;
	else
		dirty = DMA_RX_SIZE - rx_q->dirty_rx + rx_q->cur_rx;

	return dirty;
}

static inline void x2_rx_refill(struct x2_priv *priv, u32 queue)
{
	struct x2_rx_queue *rx_q = &priv->rx_queue[queue];
	int dirty = x2_rx_dirty(priv, queue);
	unsigned int entry = rx_q->dirty_rx;


	int bfsize = priv->dma_buf_sz;
//	printk("%s, and bfsize:%d\n",__func__,bfsize);
	while (dirty-- > 0) {
		struct dma_desc *p;
		if (priv->extend_desc)
			p = (struct dma_desc *)(rx_q->dma_erx + entry);
		else
			p = rx_q->dma_rx + entry;

		if (!rx_q->rx_skbuff[entry]) {
			struct sk_buff *skb;
			skb = netdev_alloc_skb_ip_align(priv->dev, bfsize);
			if (!skb) {
				rx_q->rx_zeroc_thresh = STMMAC_RX_THRESH;
				if(net_ratelimit()) {
					printk("%s: fail to alloc skb entry%d\n",__func__, entry);
				}

				break;
			}

			rx_q->rx_skbuff[entry] = skb;
			rx_q->rx_skbuff_dma[entry] = dma_map_single(priv->device, skb->data, bfsize, DMA_FROM_DEVICE);
			if (dma_mapping_error(priv->device, rx_q->rx_skbuff_dma[entry])) {
				printk("%s: Rx DMA map failed\n",__func__);
				dev_kfree_skb(skb);
				break;
			}

			p->des0 = cpu_to_le32(rx_q->rx_skbuff_dma[entry]);
			p->des1 = 0;

			if (rx_q->rx_zeroc_thresh > 0)
				rx_q->rx_zeroc_thresh--;

			//printk("%s: refill entry: #%d\n",__func__,entry);
		}

		dma_wmb();

		p->des3 = cpu_to_le32(RDES3_OWN | RDES3_BUFFER1_VALID_ADDR);//x2_init_rx_desc(p, priv->use_riwt, 0,0);
		p->des3 |= cpu_to_le32(RDES3_INT_ON_COMPLETION_EN);

		dma_wmb();
		entry = X2_GET_ENTRY(entry, DMA_RX_SIZE);
	}

	rx_q->dirty_rx = entry;
}


static int x2_rx_packet(struct x2_priv *priv, int limit, u32 queue)
{
	struct x2_rx_queue *rx_q = &priv->rx_queue[queue];
	unsigned int entry = rx_q->cur_rx;
	int coe = priv->rx_csum;
	unsigned int next_entry;
	unsigned int count = 0;
//	const struct ethhdr *eth;
//	struct iphdr *iph;
	struct net_device *ndev = priv->dev;
//	int flag = 0;

	//struct arphdr *arph;
	//unsigned char *arp_ptr;

	while (count < limit) {
		int status;
		struct dma_desc *p, *np;

		if (priv->extend_desc)
			p = (struct dma_desc *)(rx_q->dma_erx + entry);
		else
			p = rx_q->dma_rx + entry;

		status = x2_get_rx_status(&priv->dev->stats, &priv->xstats, p);

		if (status & dma_own)
			break;


		count++;
		rx_q->cur_rx = X2_GET_ENTRY(rx_q->cur_rx, DMA_RX_SIZE);
		next_entry = rx_q->cur_rx;

//		printk("%s, cur_rx:%d\n",__func__,rx_q->cur_rx);
		if (priv->extend_desc)
			np = (struct dma_desc *)(rx_q->dma_erx + next_entry);
		else
			np = rx_q->dma_rx + next_entry;

		prefetch(np);

		if (priv->extend_desc) {
			//so we need add code here
		}

		if (status == discard_frame) {
			//printk("%s, and discard frame\n",__func__);
			priv->dev->stats.rx_errors++;
			if (priv->hwts_rx_en && !priv->extend_desc) {
				dev_kfree_skb_any(rx_q->rx_skbuff[entry]);
				rx_q->rx_skbuff[entry] = NULL;
				dma_unmap_single(priv->device, rx_q->rx_skbuff_dma[entry], priv->dma_buf_sz, DMA_FROM_DEVICE);
			}
		}else {
			struct sk_buff *skb;
			int frame_len;
			unsigned int des;

			des = le32_to_cpu(p->des0);
			frame_len = (le32_to_cpu(p->des3) & RDES3_PACKET_SIZE_MASK);
			if (frame_len > priv->dma_buf_sz) {
				printk("len %d larger than size (%d)\n",frame_len, priv->dma_buf_sz);
				priv->dev->stats.rx_length_errors++;
				break;
			}

			if (!(ndev->features & NETIF_F_RXFCS)) {
				frame_len -= 4;
			}
		////	printk("%s,and first get frame-len:%d\n",__func__,frame_len);
//			if (status != llc_snap)
//				frame_len -= ETH_FCS_LEN;

//			printk("%s,and second get frame-len:%d\n",__func__,frame_len);
			skb = rx_q->rx_skbuff[entry];
			if (!skb) {
				printk("%s: inconsistent Rx chain\n",priv->dev->name);
				priv->dev->stats.rx_dropped++;
				break;
			}

			prefetch(skb->data - NET_IP_ALIGN);
			rx_q->rx_skbuff[entry] = NULL;
			rx_q->rx_zeroc_thresh++;
			skb_put(skb, frame_len);
			dma_unmap_single(priv->device, rx_q->rx_skbuff_dma[entry], priv->dma_buf_sz, DMA_FROM_DEVICE);


			x2_get_rx_hwtstamp(priv, p, np, skb);
			x2_rx_vlan(priv->dev, skb);

//			eth = (struct ethhdr*)skb->data;
			//printk("%s,and h_proto:0x%x\n",__func__,eth->h_proto);


#if 0
			if (eth->h_proto == 0x608) {
				flag = 1;

			}
			printk("%s, queue:%d, dst mac: %x:%x:%x:%x:%x:%x \n",__func__,queue,eth->h_dest[0],eth->h_dest[1],eth->h_dest[2],eth->h_dest[3],eth->h_dest[4],eth->h_dest[5]);
			printk("%s, src mac: %x:%x:%x:%x:%x:%x \n",__func__,eth->h_source[0],eth->h_source[1],eth->h_source[2],eth->h_source[3],eth->h_source[4],eth->h_source[5]);
	//		printk("%s, and h_proto: 0x%x\n",__func__,eth->h_proto);

#endif

			skb->protocol = eth_type_trans(skb, priv->dev);


		//	printk("%s, and protocol:0x%x, and len:%d\n",__func__,skb->protocol, frame_len);

		#if 0
			if (flag) {
				arph = (struct arphdr *)skb->data;
				printk("%s, arp: %s, \n", arph->ar_op ? "ARP request": "ARP response");
				arp_ptr = (unsigned char *)(arph + 1);

				arp_ptr += 6;
				printk("%s, arp src ip:%d.%d.%d.%d.\n",__func__, arp_ptr[0],arp_ptr[1],arp_ptr[2],arp_ptr[3]);
				arp_ptr += 10;
				printk("%s, arp dst ip:%d.%d.%d.%d.\n",__func__, arp_ptr[0],arp_ptr[1],arp_ptr[2],arp_ptr[3]);

			} else {
				iph = (struct iphdr *)skb->data;
				printk("%s,and iph_proto:0x%x, saddr:0x%x, daddr:0x%x\n",__func__,iph->protocol, iph->saddr, iph->daddr);
			}

			flag = 0;

#endif
			if (!coe)
				skb_checksum_none_assert(skb);
			else
				skb->ip_summed = CHECKSUM_UNNECESSARY;

			napi_gro_receive(&rx_q->napi, skb);
			priv->dev->stats.rx_packets++;
			priv->dev->stats.rx_bytes += frame_len;
		}

		entry = next_entry;
	}

	x2_rx_refill(priv, queue);
	priv->xstats.rx_pkt_n += count;

	return count;
}


static inline void x2_enable_dma_irq(struct x2_priv *priv, u32 chan)
{
	writel(DMA_CHAN_INTR_DEFAULT_MASK, priv->ioaddr + DMA_CHAN_INTR_ENA(chan));
}

static int x2_poll(struct napi_struct *napi, int budget)
{
	struct x2_rx_queue *rx_q = container_of(napi, struct x2_rx_queue, napi);
	struct x2_priv *priv = rx_q->priv_data;

	u32 tx_count = priv->plat->tx_queues_to_use;
	u32 chan = rx_q->queue_index;
	int work_done = 0;
	u32 queue;

	priv->xstats.napi_poll++;

	for (queue = 0; queue < tx_count; queue++) {
		x2_tx_clean(priv, queue);
	}

	work_done = x2_rx_packet(priv, budget, rx_q->queue_index);

	if (work_done < budget) {
		napi_complete_done(napi, work_done);
		x2_enable_dma_irq(priv, chan);
	}

	return work_done;

}


static void x2_clk_csr_set(struct x2_priv *priv)
{
	u32 clk_rate;

	clk_rate = clk_get_rate(priv->plat->x2_clk);

	if (!(priv->clk_csr & MAC_CSR_H_FRQ_MASK)) {
		if (clk_rate < CSR_F_35M)
			priv->clk_csr = STMMAC_CSR_20_35M;
		else if ((clk_rate >= CSR_F_35M) && (clk_rate < CSR_F_60M))
			priv->clk_csr = STMMAC_CSR_35_60M;
		else if ((clk_rate >= CSR_F_60M) && (clk_rate < CSR_F_100M))
			priv->clk_csr = STMMAC_CSR_60_100M;
		else if ((clk_rate >= CSR_F_100M) && (clk_rate < CSR_F_150M))
			priv->clk_csr = STMMAC_CSR_100_150M;
		else if ((clk_rate >= CSR_F_150M) && (clk_rate < CSR_F_250M))
			priv->clk_csr = STMMAC_CSR_150_250M;
		else if ((clk_rate >= CSR_F_250M) && (clk_rate < CSR_F_300M))
			priv->clk_csr = STMMAC_CSR_250_300M;
	}


	priv->clk_csr = STMMAC_CSR_20_35M;

}


static void x2_reset_subtask(struct x2_priv *priv)
{

#if 0
	if (!test_and_clear_bit(STMMAC_RESET_REQUESTED, &priv->state))
		return;

	if (test_bit(STMMAC_DOWN, &priv->state))
		return;


#endif
	printk("%s, Reset adapter\n", __func__);
}

static void x2_service_task(struct work_struct *work)
{
	struct x2_priv *priv  = container_of(work, struct x2_priv, service_task);

	x2_reset_subtask(priv);
	//clear_bit(STMMAC_SERVICE_SCHED, &priv->state);
}

static int x2_dvr_probe(struct device *device, struct plat_config_data *plat_dat, struct x2_resource *x2_res)
{
	struct net_device *ndev = NULL;
	struct x2_priv *priv;
	int ret = 0;
	u32 queue;

	ndev = alloc_etherdev_mqs(sizeof(struct x2_priv), MTL_MAX_TX_QUEUES, MTL_MAX_RX_QUEUES);
	if (!ndev)
		return -ENOMEM;

	SET_NETDEV_DEV(ndev, device);
	priv = netdev_priv(ndev);
	priv->device = device;
	priv->dev = ndev;

	ndev->ethtool_ops = &x2_ethtool_ops;
	priv->plat = plat_dat;
	priv->ioaddr = x2_res->addr;
	priv->dev->base_addr = (unsigned long)x2_res->addr;

	priv->dev->irq = x2_res->irq;

	if (x2_res->mac) {
		ether_addr_copy(ndev->dev_addr, x2_res->mac);
		dev_info(device,"set mac address %02x:%02x:%02x:%02x:%02x:%02x (using dtb adress)\n",ndev->dev_addr[0],ndev->dev_addr[1], ndev->dev_addr[2],ndev->dev_addr[3],ndev->dev_addr[4],ndev->dev_addr[5]);
	} else {
		get_random_bytes(ndev->dev_addr, ndev->addr_len);
		ndev->dev_addr[0] = 0x00;
		ndev->dev_addr[1] = 0x11;
		ndev->dev_addr[2] = 0x22;
		dev_info(device,"set mac address %02x:%02x:%02x:%02x:%02x:%02x (using random mac adress)\n",ndev->dev_addr[0],ndev->dev_addr[1], ndev->dev_addr[2],ndev->dev_addr[3],ndev->dev_addr[4],ndev->dev_addr[5]);
	}

	x2_set_umac_addr(priv->ioaddr, ndev->dev_addr, 0);

	dev_set_drvdata(device, priv->dev);
#if 0
	priv->wq = create_singlethread_workqueue("x2_wq");
	if (!priv->wq) {
		printk("failed to create workqueue\n");
		goto error_wq;
	}

	INIT_WORK(&priv->service_task, x2_service_task);

#endif

	ret = x2_hw_init(priv);
	if (ret)
		goto err_hw_init;

	if (priv->dma_cap.tsn && priv->plat->tx_queues_to_use > 3) {

		x2_configure_tsn(priv);
	}

	netif_set_real_num_rx_queues(ndev, priv->plat->rx_queues_to_use);
	netif_set_real_num_tx_queues(ndev, priv->plat->tx_queues_to_use);

	ndev->netdev_ops = &x2_netdev_ops;

	ndev->hw_features = NETIF_F_SG;

#if 1
	if((priv->plat->tso_en) ) {
		ndev->hw_features |= NETIF_F_TSO | NETIF_F_TSO6;
		priv->tso = true;
		printk("TSO feature enabled\n");
	}
	if (priv->dma_cap.tx_coe)
		ndev->hw_features |= NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM;

	if (priv->dma_cap.rx_coe)
		ndev->hw_features |= NETIF_F_RXCSUM;
#endif
	ndev->hw_features |= NETIF_F_RXFCS;

	ndev->features = ndev->hw_features | NETIF_F_HIGHDMA;

	ndev->watchdog_timeo = DWCEQOS_TX_TIMEOUT * HZ;

#ifdef X2_VLAN_TAG_USED
	ndev->features |= NETIF_F_HW_VLAN_CTAG_RX;
#endif

	x2_mdio_set_csr(priv);

	ndev->min_mtu = ETH_ZLEN - ETH_HLEN;
	ndev->max_mtu = JUMBO_LEN;

	if ((priv->plat->maxmtu < ndev->max_mtu) && (priv->plat->maxmtu >= ndev->min_mtu))
		ndev->max_mtu = priv->plat->maxmtu;
	else if (priv->plat->maxmtu < ndev->min_mtu)
		printk("waring: maxmtu having invalid value by Network\n");



	for (queue = 0; queue < priv->plat->rx_queues_to_use; queue++) {
		struct x2_rx_queue *rx_q = &priv->rx_queue[queue];

		netif_napi_add(ndev, &rx_q->napi, x2_poll, (8 * priv->plat->rx_queues_to_use));
	}

	spin_lock_init(&priv->lock);

	if (!priv->plat->clk_csr)
		x2_clk_csr_set(priv);
	else
		priv->clk_csr = priv->plat->clk_csr;

	ret = x2_mdio_register(priv);
	if (ret < 0) {
		printk("MDIO bus error register\n");
		goto err_mdio_reg;
	}

	ret = devm_request_irq(priv->device, ndev->irq, &x2_interrupt, 0, ndev->name, ndev);
	if (ret < 0) {
		printk("%s: error allocating the IRQ\n",__func__);
		goto irq_error;
	}

	ret = register_netdev(ndev);
	if (ret) {
		printk("error register network device\n");
		goto err_netdev_reg;
	}
	return 0;

irq_error:
err_netdev_reg:
	mdiobus_unregister(priv->mii);
	priv->mii->priv = NULL;
	mdiobus_free(priv->mii);
	priv->mii = NULL;
err_mdio_reg:
	for (queue = 0; queue < priv->plat->rx_queues_to_use; queue++) {
		struct x2_rx_queue *rx_q = &priv->rx_queue[queue];
		netif_napi_del(&rx_q->napi);
	}
err_hw_init:
	free_netdev(ndev);
	return ret;
}


static int x2_eth_dwmac_config_dt(struct platform_device *pdev, struct plat_config_data *plat_dat)
{
	struct device_node *np = pdev->dev.of_node;
	u32 burst_map = 0;
	u32 bit_index = 0;
	u32 a_index = 0;

	if (!plat_dat->axi) {
		plat_dat->axi = kzalloc(sizeof(struct x2_axi), GFP_KERNEL);
		if (!plat_dat->axi)
			return -ENOMEM;
	}

	plat_dat->axi->axi_lpi_en = of_property_read_bool(np,"snps,en-lpi");
	if (of_property_read_u32(np,"snps,write-requests",&plat_dat->axi->axi_wr_osr_lmt))
		plat_dat->axi->axi_wr_osr_lmt = 1;
	else
		plat_dat->axi->axi_wr_osr_lmt--;

	if (of_property_read_u32(np,"snps,read-requests",&plat_dat->axi->axi_rd_osr_lmt))
		plat_dat->axi->axi_rd_osr_lmt = 1;
	else
		plat_dat->axi->axi_rd_osr_lmt--;


	of_property_read_u32(np,"snps,burst-map",&burst_map);


	for (bit_index = 0; bit_index < 7; bit_index++) {
		if (burst_map & (1 << bit_index)) {
			switch (bit_index) {
			case 0:
			plat_dat->axi->axi_blen[a_index] = 4;break;
			case 1:
			plat_dat->axi->axi_blen[a_index] = 8; break;
			case 2:
			plat_dat->axi->axi_blen[a_index] = 16; break;
			case 3:
			plat_dat->axi->axi_blen[a_index] = 32; break;
			case 4:
			plat_dat->axi->axi_blen[a_index] = 64; break;
			case 5:
			plat_dat->axi->axi_blen[a_index] = 128; break;
			case 6:
			plat_dat->axi->axi_blen[a_index] = 256; break;
			default:
			break;
			}
			a_index++;
		}
	}


	plat_dat->has_gmac4 = 1;
	plat_dat->dma_cfg->aal = 1;
	plat_dat->tso_en = 1;
	plat_dat->pmt = 1;
	return 0;
}


static int hobot_eth_probe(struct platform_device *pdev)
{
	struct plat_config_data *plat_dat;

	struct resource *res;
	struct x2_resource x2_res;
	int ret = -ENXIO;






	memset(&x2_res, 0, sizeof(struct x2_resource));

	x2_res.irq = platform_get_irq(pdev, 0);


	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		printk("%s, get plat resouce failed\n",__func__);
		ret = -ENXIO;
		goto err_get_res;
	}
	x2_res.addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(x2_res.addr)) {
		printk("%s, error ioremap\n",__func__);
		ret = PTR_ERR(x2_res.addr);
		goto err_get_res;
	}


	plat_dat = x2_probe_config_dt(pdev, &x2_res.mac);
	if (IS_ERR(plat_dat)) {
		return PTR_ERR(plat_dat);
	}


	ret = x2_eth_dwmac_config_dt(pdev, plat_dat);
	if (ret)
		goto remove;

	ret = x2_dvr_probe(&pdev->dev, plat_dat, &x2_res);
	if (ret)
		goto remove;

	printk("%s: probe sucessfully\n",__func__);
	return ret;


remove:

err_get_res:

	return ret;


}

static int hgb_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct x2_priv *priv = netdev_priv(ndev);

	printk("%s\n",__func__);
	x2_stop_all_dma(priv);
	x2_set_mac(priv->ioaddr, false);
	netif_carrier_off(ndev);
	unregister_netdev(ndev);
	clk_disable_unprepare(priv->plat->x2_phy_ref_clk);
	clk_disable_unprepare(priv->plat->x2_mac_div_clk);
	clk_disable_unprepare(priv->plat->x2_mac_pre_div_clk);

	if (priv->mii) {
		mdiobus_unregister(priv->mii);
		priv->mii->priv = NULL;
		mdiobus_free(priv->mii);
		priv->mii = NULL;
	}
	free_netdev(ndev);
	of_node_put(priv->plat->phy_node);
	of_node_put(priv->plat->mdio_node);
	printk("%s, successufully exit\n",__func__);
	return 0;
}

static const struct of_device_id hgb_of_match[] = {
	{ .compatible = "snps,dwc-qos-ethernet-4.10a", },
	{}
};

MODULE_DEVICE_TABLE(of, hgb_of_match);

static struct platform_driver hgb_driver = {
	.probe = hobot_eth_probe,
	.remove = hgb_remove,
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = hgb_of_match,
	},

};

module_platform_driver(hgb_driver);
MODULE_LICENSE("GPL v2");
