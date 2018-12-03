/*
 * X2 SPI controller driver (master mode only)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/io.h>
#include <linux/delay.h>

#define X2_SPI_MAX_CS          3
#define X2_SPI_NAME            "x2_spi"
/* X2 SPI register offsets */
#define X2_SPI_TXD_REG         0x00 /* data transmit register */
#define X2_SPI_RXD_REG         0x04 /* data receive register */
#define X2_SPI_CTRL_REG        0x08 /* SPI control register */
#define X2_SPI_SSC_REG         0x0C /* SSN limit configure register */
#define X2_SPI_SFSR_REG        0x10 /* SPI status register */
#define X2_SPI_RFTO_REG        0x14 /* Threshold of receive time out */
#define X2_SPI_TLEN_REG        0x18 /* transfer length */
#define X2_SPI_INST_REG        0x1C /* instruction should be transmitted */
#define X2_SPI_INST_MASK_REG   0x20 /* instruction mask register */
#define X2_SPI_SRCPND_REG      0x24 /* spi interrupt source pending register */
#define X2_SPI_INTMASK_REG     0x28 /* interrupt mask register */
#define X2_SPI_INTSETMASK_REG  0x2C /* interrupt set mask register */
#define X2_SPI_INTUNMASK_REG   0x30 /* interrupt unset mask register */
#define X2_SPI_DMA_CTRL0_REG   0x34 /* DMA control register_0 */
#define X2_SPI_DMA_CTRL1_REG   0x38 /* DMA control register_1 */
#define X2_SPI_TDMA_ADDR0_REG  0x3C /* transmit dma addr_0 */
#define X2_SPI_TDMA_SIZE0_REG  0x40 /* transmit dma size_0 */
#define X2_SPI_TDMA_ADDR1_REG  0x44 /* transmit dma addr_1 */
#define X2_SPI_TDMA_SIZE1_REG  0x48 /* transmit dma size_1 */
#define X2_SPI_RDMA_ADDR0_REG  0x4C /* receive  dma addr_0 */
#define X2_SPI_RDMA_SIZE0_REG  0x50 /* receive  dma size_0 */
#define X2_SPI_RDMA_ADDR1_REG  0x54 /* receive  dma addr_1 */
#define X2_SPI_RDMA_SIZE1_REG  0x58 /* receive  dma size_1 */
#define X2_SPI_TSIZE_REG       0x5C /* current tx_size */
#define X2_SPI_RSIZE_REG       0x60 /* current rx_size */
#define X2_SPI_FIFO_RESET_REG  0x64 /* FIFO reset register */
#define X2_SPI_TDMA_SIZE_REG   0x68 /* tx_dma size for ping-pong */
#define X2_SPI_RDMA_SIZE_REG   0x6C /* rx_dma size for ping-pong */
/* X2 SPI register bit Masks */
#define X2_SPI_CORE_EN         BIT(16)
#define X2_SPI_SS_OFFSET       30
#define X2_SPI_SS_MASK         0xC0000000
#define X2_SPI_SLAVE_MODE      BIT(21)
#define X2_SPI_INT_ALL         0x000007FF
#define X2_SPI_TIP_MASK        BIT(8)
#define X2_SPI_DATA_RDY        BIT(4)
#define X2_SPI_TF_EMPTY        BIT(5)
#define X2_SPI_TX_DIS          BIT(23)
#define X2_SPI_RX_DIS          BIT(22)
#define X2_SPI_SAMP_SEL        BIT(27)


/* X2 SPI operation Macro */
#define X2_SPI_OP_CORE_EN      1
#define X2_SPI_OP_CORE_DIS     0


#define x2_spi_rd(dev, reg)       ioread32((dev)->regs_base + (reg))
#define x2_spi_wr(dev, reg, val)  iowrite32((val), (dev)->regs_base + (reg))

struct x2_spi {
    void __iomem *regs_base; /* virt. address of the control registers */
    int irq;                 /* spi interrupt */
    const u8 *txbuf;         /* pointer in the Tx buffer */
    u8 *rxbuf;               /* pointer in the Rx buffer */
    int len;                 /* Number of bytes left to transfer */
    int word_width;          /* Bits per word:16bit or 8bit */
};

static int x2_spi_en_ctrl(struct x2_spi *x2spi, int en_flag)
{
    u32 val = 0;

    val = x2_spi_rd(x2spi, X2_SPI_CTRL_REG);
    if (en_flag)
        val |= X2_SPI_CORE_EN;
    else
        val &= (~X2_SPI_CORE_EN);
    x2_spi_wr(x2spi, X2_SPI_CTRL_REG, val);

    return 0;
}

static irqreturn_t x2_spi_int_handle(int irq, void *data)
{
    struct x2_spi *x2spi = (struct x2_spi *)data;
    pr_err("enter %s_%d!\n", __func__, __LINE__);
    return IRQ_NONE;
}

static void x2_spi_init_hw(struct x2_spi *x2spi)
{
    u32 val = 0;

    /* First, should reset the whole controller */
    //x2_spi_reset_func()

    x2_spi_en_ctrl(x2spi, X2_SPI_OP_CORE_DIS);

    x2_spi_wr(x2spi, X2_SPI_INTSETMASK_REG, X2_SPI_INT_ALL); /* disable all interrupt */
    x2_spi_wr(x2spi, X2_SPI_SRCPND_REG, X2_SPI_INT_ALL); /* clear all interrupt pending */

    val = x2_spi_rd(x2spi, X2_SPI_CTRL_REG);
    val &= (~X2_SPI_SLAVE_MODE); /* spi master mode */
    val &= (~X2_SPI_RX_DIS);
    val &= (~X2_SPI_TX_DIS);
    val &= (~X2_SPI_SAMP_SEL);
    x2_spi_wr(x2spi, X2_SPI_CTRL_REG, val);

    x2_spi_en_ctrl(x2spi, X2_SPI_OP_CORE_EN);

    return;
}

/* This function enables SPI master controller */
static int x2_spi_prepare_xfer_hardware(struct spi_master *master)
{
    struct x2_spi *x2spi = spi_master_get_devdata(master);

    x2_spi_en_ctrl(x2spi, X2_SPI_OP_CORE_EN);

    return 0;
}

static int x2_spi_unprepare_xfer_hardware(struct spi_master *master)
{
    struct x2_spi *x2spi = spi_master_get_devdata(master);

    x2_spi_en_ctrl(x2spi, X2_SPI_OP_CORE_DIS);

    return 0;
}

/**
 * x2_spi_chipselect - Select or deselect the chip select line
 * @spi:     Pointer to the spi_device structure
 * @is_high: Select(0) or deselect (1) the chip select line
 */
static void x2_spi_chipselect(struct spi_device *spi, bool is_high)
{
    u32 val = 0;
    struct x2_spi *x2spi = spi_master_get_devdata(spi->master);

    val = x2_spi_rd(x2spi, X2_SPI_CTRL_REG);
    if (is_high) {
        val |= X2_SPI_SS_MASK;
    } else {
        val &= (~X2_SPI_SS_MASK);
        val |= (spi->chip_select<<X2_SPI_SS_OFFSET);
    }
    x2_spi_wr(x2spi, X2_SPI_CTRL_REG, val);

    return;
}

static int x2_spi_transfer_one(struct spi_master *master, struct spi_device *spi,
                                       struct spi_transfer *xfer)
{
    u32 val, i, j;
    unsigned int txd = 0;
    struct x2_spi *x2spi = spi_master_get_devdata(master);

    x2spi->txbuf = (u8 *)xfer->tx_buf;
    x2spi->rxbuf = (u8 *)xfer->rx_buf;
    x2spi->len   = xfer->len;

    j = 0;
    do {
        j++;
        udelay(10);
        val = x2_spi_rd(x2spi, X2_SPI_SFSR_REG);
    }while((val & X2_SPI_TIP_MASK) && (j < 10));
    if (j >= 10) return -EAGAIN;

    for (i=0; i<x2spi->len; i++) {
        j = 0;
        do {
            j++;
            udelay(10);
            val = x2_spi_rd(x2spi, X2_SPI_SFSR_REG);
        } while((!(val & X2_SPI_TF_EMPTY)) && (j<10));
        if (j >= 10) return -EAGAIN;
        txd = x2spi->txbuf ? x2spi->txbuf[i] : 0;
        x2_spi_wr(x2spi, X2_SPI_TLEN_REG, 8);
        x2_spi_wr(x2spi, X2_SPI_TXD_REG, txd&0xFF);
        if (x2spi->rxbuf) {
            j = 0;
            do {
                j++;
                udelay(10);
                val = x2_spi_rd(x2spi, X2_SPI_SFSR_REG);
            } while((!(val & X2_SPI_DATA_RDY)) && j<10);
            if (j >= 10) return -EAGAIN;
            x2spi->rxbuf[i] = (u8)x2_spi_rd(x2spi, X2_SPI_RXD_REG);
        }
        spi_finalize_current_transfer(master);
    }

    j = 0;
    do {
        j++;
        udelay(10);
        val = x2_spi_rd(x2spi, X2_SPI_SFSR_REG);
    } while((val & X2_SPI_TIP_MASK) && (j<10));
    if (j >= 10) return -EAGAIN;

    return xfer->len;
}

static int x2_spi_probe(struct platform_device *pdev)
{
    int err;
    struct x2_spi *x2spi;
    struct spi_master *master;
    struct resource *res;

    dev_info(&pdev->dev, "enter x2_spi_probe!\n");
    x2spi = devm_kzalloc(&pdev->dev, sizeof(*x2spi), GFP_KERNEL);
    if (!x2spi)
        return -ENOMEM;

    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    x2spi->regs_base = devm_ioremap_resource(&pdev->dev, res);
    if (IS_ERR(x2spi->regs_base))
        return PTR_ERR(x2spi->regs_base);
    x2spi->word_width = 8;

    x2_spi_init_hw(x2spi);

    x2spi->irq = platform_get_irq(pdev, 0);
    if (x2spi->irq < 0) {
        dev_warn(&pdev->dev, "Can't get interrupt resource!\n");
        return x2spi->irq;
    }

#if 0
    ret = devm_request_irq(&pdev->dev, x2spi->irq, x2_spi_int_handle, 0, pdev->name, x2spi);
    if (ret < 0) {
        dev_warn(&pdev->dev, "can't request interrupt\n");
        return ret;
    }
#endif
    master = spi_alloc_master(&pdev->dev, sizeof(*x2spi));
    if (master == NULL) {
        dev_err(&pdev->dev, "Unable to allocate SPI Master!\n");
        return -ENOMEM;
    }
    spi_master_set_devdata(master, x2spi);

    master->bus_num = pdev->id;
    master->num_chipselect = X2_SPI_MAX_CS;
    master->mode_bits = SPI_CPOL | SPI_CPHA;
    master->prepare_transfer_hardware = x2_spi_prepare_xfer_hardware;
    master->transfer_one = x2_spi_transfer_one;
    master->unprepare_transfer_hardware = x2_spi_unprepare_xfer_hardware;
    master->set_cs = x2_spi_chipselect;
    master->dev.of_node = pdev->dev.of_node;
    platform_set_drvdata(pdev, master);

    /* register spi controller */
    err = devm_spi_register_master(&pdev->dev, master);
    if (err) {
        dev_err(&pdev->dev, "spi register master failed!\n");
        spi_master_put(master);
        return err;
    }

    return 0;
}

static int x2_spi_remove(struct platform_device *pdev)
{
    struct spi_master *master = platform_get_drvdata(pdev);
    struct x2_spi *x2spi = spi_master_get_devdata(master);

    x2_spi_en_ctrl(x2spi, X2_SPI_OP_CORE_DIS);
    spi_unregister_master(master);

    return 0;
}

static const struct of_device_id x2_spi_of_match[] = {
    { .compatible = "hobot,x2-spi" },
    { /* end of table */ }
};
MODULE_DEVICE_TABLE(of, x2_spi_of_match);

static struct platform_driver xilinx_spi_driver = {
    .probe = x2_spi_probe,
    .remove = x2_spi_remove,
    .driver = {
        .name = X2_SPI_NAME,
        .of_match_table = x2_spi_of_match,
    },
};
module_platform_driver(xilinx_spi_driver);

MODULE_AUTHOR("hobot, Inc.");
MODULE_DESCRIPTION("X2 SPI driver");
MODULE_LICENSE("GPL v2");
