#ifndef ELPPCI_H_
#define ELPPCI_H_

/* PCI FPGA control bits for Register 0x40 */
#define ELPPCI_CTRL_REG           0x40
#define ELPPCI_RST_BIT            (1ul<<0)
#define ELPPCI_DPA_EN_BIT         (1ul<<2)
#define ELPPCI_SECURE_BIT         (1ul<<3)
#define ELPPCI_IRQ_EN_BIT         (1ul<<4)
#define ELPPCI_LITTLE_ENDIAN_BIT  (1ul<<5)
#define ELPPCI_CLR_MSTR_ABORT_BIT (1ul<<6)
#define ELPPCI_TRNG_RINGS_EN_BIT  (1ul<<7)

#endif
