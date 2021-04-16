/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/  
/**
 * @author   Jesse.Huang (Jesse.Huang@hobot.cc)
 * @date     2019/11/27
 * @version  V1.0
 * @par      Horizon Robotics
 */ 

#include <linux/io.h>
#define SCRIPT_NAME_MAX 	1000
#define SCRIPT_MAX 			200
#define NELEMS(x)  (sizeof(x) / sizeof((x)[0]))
    typedef struct usrdrv_regs_s {
	uint32_t offset;
	uint32_t value;
} usrdrv_regs_t;

typedef struct usrdrv_reg_script_s {
	char name[SCRIPT_NAME_MAX];
	usrdrv_regs_t script[SCRIPT_MAX];
} usrdrv_reg_script_t;

static usrdrv_reg_script_t g_script[] = {
	{
	 "script/test_bif",
	 {
	  {0x01006000, 0x12345678},
	  {0x01006000, 0x87654321},
	  {0, 0},
	  },
	 },
	{
	 "clk/default",
	 {
	  {0x01006000, 0x12345678},
	  {0x01006000, 0x87654321},
	  {0, 0},
	  },
	 },
	{
	 "clk/mipi 720p",
	 {
	  {0x01006000, 0x12345678},
	  {0x01006000, 0x87654321},
	  {0, 0},
	  },
	 },
};

#define BASE 0xA0000000
uint32_t usrdrv_run_script(volatile unsigned char *base, char *name)
{
	int i, j;
	for (i = 0; i < NELEMS(g_script); i++) {
		//use strlen(g_script[i].name) to ignore '/n' from user space
		if (strncmp(g_script[i].name, name, SCRIPT_NAME_MAX) == 0) {
			printk("%s run %s\n", __FUNCTION__, name);
			for (j = 0; j < SCRIPT_MAX; j++) {
				uint32_t off = g_script[i].script[j].offset;
				uint32_t val = g_script[i].script[j].value;
				volatile void __iomem *vaddr = base + off;
				if (off == 0)
					break;
				printk("write [%8x] to %8x; result: %8x -> ",
				       BASE + off, val, readl(vaddr));
				writel(val, vaddr);
				printk(KERN_CONT "%8x\n", readl(vaddr));
			}
			return 1;
		}
	}
	printk("%s ERROR: fail to run %s\n", __FUNCTION__, name);
	return 0;
}

void usrdrv_list_script(void)
{
	int i;
	for (i = 0; i < NELEMS(g_script); i++) {
		printk("[%4d]: %s\n", i, g_script[i].name);
	}
}
