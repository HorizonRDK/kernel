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
/*==============================================================
modify include\configs\x2.h
	CONFIG_SPL_MAX_SIZE    		0x8000 to 0xA000 
	CONFIG_SPL_BSS_MAX_SIZE     0x600  to 0x700
  ============================================================*/
#define MAX_TRACE_LEN		1000
#define MAX_CLK_ELEMENT 	300
#define MAX_CLK_NAME 		200
#define MAX_CLK_NEXT 		100
#define INV 				0xffff	/*invalid value */
#define INV_CLK				9999	/*invalid value */
#define XS					""	/*don't care string */
#define XV				    0xfffe	/*don't care value */
#define NO_CLK				"---"
#define MHZ24				"24Z"
#define OCX24				24
//#define clk_printf(format, ...)   printf("CLK: " format , ##__VA_ARGS__)
#define clk_printf(format, ...)   printk(format , ##__VA_ARGS__)
#define ERR() clk_printf("error at %s %u\n", __FUNCTION__, __LINE__);
#define LINE() clk_printf("%s %u\n", __FUNCTION__, __LINE__);

#define wreg32(offset, value)	\
	*((volatile unsigned int*)(g_reg_base + offset)) = value

#define rreg32(offset)	        \
	readl(((volatile unsigned int*)(g_reg_base + offset)))

#define debugf(format, ...)	/*clk_printf(format, ##__VA_ARGS__) */
#define infof(format, ...) 		clk_printf(format, ##__VA_ARGS__)

unsigned int caculate_clk_by_name(char *name);

volatile unsigned char *g_reg_base;

//char clk_trace_str[MAX_TRACE_LEN];

typedef struct _register_t {
	unsigned int offset;	//remove 0xA000 0000
	unsigned char msb;
	unsigned char lsb;
	unsigned int val;
} register_t;

typedef struct _next_t {
	uint16_t val_key;
	uint16_t factor;
	char str[MAX_CLK_NAME];
} next_t;

typedef struct _clk_element_t {
	char name[MAX_CLK_NAME];
	unsigned char type;
	register_t reg;
	uint16_t mhz;
	next_t next[MAX_CLK_NEXT];
} clk_element_t;

#include "inc/clock_dump_layout.h"

unsigned int get_reg(unsigned int i)
{
	unsigned int mask = 0;
	unsigned int j;
	for (j = g_clk_element[i].reg.lsb; j <= g_clk_element[i].reg.msb; j++) {
		mask |= 1 << j;
	}
	g_clk_element[i].reg.val =
	    (rreg32(g_clk_element[i].reg.offset) & mask) >> g_clk_element[i].
	    reg.lsb;
	debugf("%s reg=%x, msb=%u, lsb=%u, val=%x, mask=%x\n",
	       g_clk_element[i].name, g_clk_element[i].reg.offset,
	       g_clk_element[i].reg.msb, g_clk_element[i].reg.lsb,
	       g_clk_element[i]
	       .reg.val, mask);
	return g_clk_element[i].reg.val;
}

unsigned int caculate_clk(unsigned int i)
{
	unsigned int j = 0, val;
	if (g_clk_element[i].type == TYPE_CST) {
		val = 0;
	} else {
		val = get_reg(i);
	}

	switch (g_clk_element[i].type) {
	case TYPE_MP:
	case TYPE_MV:
	case TYPE_CST:
		for (j = 0; j < MAX_CLK_NEXT; j++) {
			if (g_clk_element[i].next[j].val_key == INV) {
				ERR();
				break;
			}
			if (g_clk_element[i].next[j].val_key == val) {
				debugf
				    ("found next,type=%u, name = %s, factor=%u\n",
				     g_clk_element[i].type,
				     g_clk_element[i].next[j].str,
				     g_clk_element[i].next[j].factor);
				break;
			}
		}
		break;
	case TYPE_MPD:
		j = 0;
		break;
	case TYPE_PLL:
		debugf("found PLL %s\n", g_clk_element[i].name);
		break;
	}

	switch (g_clk_element[i].type) {
	case TYPE_MP:
		if (strcmp(g_clk_element[i].next[j].str, NO_CLK) == 0) {
			g_clk_element[i].mhz = 0;
		} else if (strcmp(g_clk_element[i].next[j].str, MHZ24) == 0) {
			g_clk_element[i].mhz = 24;
		} else {
			g_clk_element[i].mhz =
			    caculate_clk_by_name(g_clk_element[i].next[j].str);
		}
		debugf("mhz of i,j(%u,%u)= %u\n", i, j, g_clk_element[i].mhz);
		return g_clk_element[i].mhz;
	case TYPE_MPD:
		g_clk_element[i].mhz =
		    caculate_clk_by_name(g_clk_element[i].next[0].str);
		debugf("mhz of i,j(%u,%u)= %u (before div)\n", i, 0,
		       g_clk_element[i].mhz);
		g_clk_element[i].mhz /= (val + 1);
		debugf("mhz of i,j(%u,%u)= %u (after div)\n", i, j,
		       g_clk_element[i].mhz);
		return g_clk_element[i].mhz;
	case TYPE_MV:
		(j < MAX_CLK_NEXT) ? g_clk_element[i].mhz =
		    g_clk_element[i].next[j].factor : 0;
		debugf("mhz of i,j(%u,%u)= %u\n", i, j, g_clk_element[i].mhz);
		return g_clk_element[i].mhz;
	case TYPE_CST:
		g_clk_element[i].mhz = g_clk_element[i].next[j].factor;
		return g_clk_element[i].mhz;
	case TYPE_PLL:
		 {
			unsigned int fbdiv11_00, refdiv17_12, postdiv122_20,
			    postdiv226_24;
			unsigned int val = g_clk_element[i].reg.val;
			fbdiv11_00 = (val & 0xfff);	//bit 11~0
			val = val >> (12 - 0);	//remove bit
			refdiv17_12 = (val & 0x1f);	//bit 5~0
			val = val >> (20 - 12);	//remove bit
			postdiv122_20 = (val & 0x3);	//bit 2~0
			val = val >> (24 - 20);	//remove bit
			postdiv226_24 = (val & 0x3);	//bit 2~0
			g_clk_element[i].mhz =
			    (((OCX24 / refdiv17_12) * fbdiv11_00) /
			     postdiv122_20) / postdiv226_24;
			infof("        found PLL %s=(((OCX24/refdiv17_12)*fbdiv11_00)/"
			"postdiv122_20)/postdiv226_24=%u=((%u/%u)*%u)/%u/%u\n\n",
			     g_clk_element[i].name, g_clk_element[i].mhz, OCX24,
			     refdiv17_12, fbdiv11_00, postdiv122_20,
			     postdiv226_24);
		}
		return g_clk_element[i].mhz;
	default:
		ERR();
		break;
	}
	return 0;
}

void put_value(char *str, unsigned int val)
{
	char buf[10];
	int i, len;
	snprintf(buf, sizeof(buf), "%u", val);
	debugf("PUT_VALUE: %s %u\n", buf, val);
	len = strlen(buf);
	if (len > 10)
		len = 10;
	for (i = 0; i < len; i++) {
		str[i] = buf[i];
	}
}

unsigned int get_index_by_name(char *name)
{
	unsigned int i;
	for (i = 0; i < MAX_CLK_ELEMENT; i++) {
		if (strcmp(g_clk_element[i].name, "") == 0) {
			ERR();
			break;
		}
		if (strcmp(g_clk_element[i].name, name) == 0) {
			return i;
		}
	}
	return INV;
}

unsigned int caculate_clk_by_name(char *name)
{
	unsigned int i;
	//unsigned int trace_val_offset;
	//char buf[100];
	for (i = 0; i < MAX_CLK_ELEMENT; i++) {
		debugf("search %s, %u\n", name, i);
		if (strcmp(g_clk_element[i].name, "") == 0) {
			ERR();
			break;
		}
		if (strcmp(g_clk_element[i].name, name) == 0) {
			debugf("found, index = %u\n", i);

			g_clk_element[i].mhz = caculate_clk(i);

			debugf("mhz of i(%u) = %u\n", i, g_clk_element[i].mhz);
			infof("\"%s \\n %x[%u:%u]=(%x,%u)\"->",
			      g_clk_element[i].name,
			      g_clk_element[i].reg.offset,
			      g_clk_element[i].reg.msb,
			      g_clk_element[i].reg.lsb,
			      g_clk_element[i].reg.val, g_clk_element[i].mhz);
			return g_clk_element[i].mhz;
		}
	}
	return 0;
}

void dump_clk(char clk_name[][MAX_CLK_NAME])
{
	unsigned int i;
	for (i = 0; strcmp(clk_name[i], "") != 0; i++) {
		//clk_trace_str[0]=0;
		infof
		    ("=================%s clock trace:====================== \n",
		     clk_name[i]);
		caculate_clk_by_name(clk_name[i]);
		infof("\n");
		//infof("%s %u Mhz\n",clk_name[i],mhz);
		//infof("TRACE: %s\n",clk_trace_str);
	}
}

void dump_clklist(char clk_name[][MAX_CLK_NAME])
{
	unsigned int i;
	for (i = 0; strcmp(clk_name[i], "") != 0; i++) {
		infof("[%4.4d] %s\n", i, clk_name[i]);
	}
	infof("[%4.4d] all\n", i);
}

void init_base(volatile unsigned char *base)
{
	g_reg_base = base;
}

//==========================================================
void clk_dump_list(void)
{
	dump_clklist(clk_endpoint);
}

uint32_t clk_dump(volatile unsigned char *base, char *clk_name)
{
	init_base(base);
	debugf("%s: sizeof(int)=%u, should be 4\n", __FUNCTION__,
	       (unsigned int)sizeof(int));
	if (strcmp(clk_name, "all") == 0) {
		dump_clk(clk_endpoint);
		return 1;
	}
	caculate_clk_by_name(clk_name);
	return 1;
}
