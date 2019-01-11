#include < linux / kernel.h >
#define _DEBUG_PRINTF_
#include "debug.h"
#define DBG_LOG_HEX_WIDTH 16
static char prinf_buf[4096];

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
		off += snprintf(pbuf + off, plen - off, "0x%08X  ", i);
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
	_print2buf(buf, len, prinf_buf, sizeof(prinf_buf));
	printk(prinf_buf);
}
