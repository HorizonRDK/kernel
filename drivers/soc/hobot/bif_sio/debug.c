#include <linux/kernel.h>
#define _DEBUG_PRINTF_
#include "debug.h"
#define DBG_LOG_HEX_WIDTH 16
#define PACKET_ONCE	128

static char prinf_buf[4096];
static int addr_base;

static int _print_hex(unsigned char *bbuf, int blen, char *pbuf, int plen)
{
	int i;
	int off = 0;

	for (i = 0; i < DBG_LOG_HEX_WIDTH; ++i) {

		if (i < blen)
			off +=
			    snprintf(pbuf + off, plen - off, "%02X ", bbuf[i]);
		else
			off += snprintf(pbuf + off, plen - off, "   ");

	}
	return off;

}

static int _print_ascii(unsigned char *bbuf, int blen, char *pbuf, int plen)
{
	int i;
	int off = 0;

	for (i = 0; i < blen; ++i) {
		if (bbuf[i] >= 32 && bbuf[i] <= 126)
			off += snprintf(pbuf + off, plen - off, "%c", bbuf[i]);
		else
			off += snprintf(pbuf + off, plen - off, ".");
	}

	return off;

}

static void _print2buf(unsigned char *bbuf, int blen, char *pbuf, int plen)
{
	int i;
	int curlen;
	int off;

	off = 0;
	for (i = 0; i < blen; i += DBG_LOG_HEX_WIDTH) {

		curlen = blen - i;
		if (curlen > DBG_LOG_HEX_WIDTH)
			curlen = DBG_LOG_HEX_WIDTH;
		off += snprintf(pbuf + off, plen - off, "0x%08X  ",
				i + addr_base);
		off += _print_hex(bbuf + i, curlen, pbuf + off, plen - off);
		off += snprintf(pbuf + off, plen - off, "  ");
		off += _print_ascii(bbuf + i, curlen, pbuf + off, plen - off);
		off += snprintf(pbuf + off, plen - off, "\n");

	}

	if (off >= plen)
		pbuf[plen - 1] = 0;
	else
		pbuf[off] = 0;
	return;

}

void _dbg_printhex(unsigned char *buf, int len)
{
	int i;
	int curlen;

	addr_base = 0;
	/*
	 * printk have limit can't print longer than 128 maybe or more
	 */
	for (i = 0; i < len; i += PACKET_ONCE) {
		curlen = len - i;
		if (curlen > PACKET_ONCE)
			curlen = PACKET_ONCE;
		_print2buf(buf + i, curlen, prinf_buf, sizeof(prinf_buf));
		addr_base += PACKET_ONCE;
		printk(prinf_buf);
	}
}
