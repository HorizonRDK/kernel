/*
 * 1-Wire implementation for the ds23el15 chip
 *
 * Copyright (C) 2013 maximintergrated
 *
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/idr.h>
#include <linux/delay.h>
#include <linux/random.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/gpio.h>

#include <linux/input.h>

#include "../w1.h"
#include "../w1_int.h"
#include "../w1_family.h"
#include "w1_ds28e1x_sha256.h"

#define SANITY_DBGCHK 0

//!TBD: add this definition in w1_family.h will be better
#define W1_FAMILY_DS28E15   0x17
#define W1_FAMILY_DS28E11   0x4B
#define W1_FAMILY_DS28E1X   W1_FAMILY_DS28E15

// 1-Wire commands
#define CMD_WRITE_MEMORY         0x55
#define CMD_READ_MEMORY          0xF0
#define CMD_LOAD_LOCK_SECRET     0x33
#define CMD_COMPUTE_LOCK_SECRET  0x3C
#define CMD_SELECT_SECRET        0x0F
#define CMD_COMPUTE_PAGEMAC      0xA5
#define CMD_READ_STATUS          0xAA
#define CMD_WRITE_BLOCK_PROTECT  0xC3
#define CMD_WRITE_AUTH_MEMORY    0x5A
#define CMD_WRITE_AUTH_PROTECT   0xCC
#define CMD_PIO_READ             0xDD
#define CMD_PIO_WRITE            0x96
#define CMD_RELEASE		0xAA

#define BLOCK_READ_PROTECT       0x80
#define BLOCK_WRITE_PROTECT      0x40
#define BLOCK_EPROM_PROTECT      0x20
#define BLOCK_WRITE_AUTH_PROTECT 0x10

#define ROM_CMD_SKIP             0x3C
#define ROM_CMD_RESUME           0xA5

#define SELECT_SKIP     0
#define SELECT_RESUME   1
#define SELECT_MATCH    2
#define SELECT_ODMATCH  3
#define SELECT_SEARCH   4
#define SELECT_READROM  5
#define SELECT_ODSKIP	6

#define PROT_BIT_AUTHWRITE	0x10
#define PROT_BIT_EPROM		0x20
#define PROT_BIT_WRITE		0x40
#define PROT_BIT_READ		0x80

#define DS28E15_FAMILY		0x17

#define DS28E15_PAGES		2	// 2 pages, 4 blocks, 7 segment in each page. total 64 bytes
#define PAGE_TO_BYTE		32	// 1 page = 32 bytes
#define BLOCK_TO_BYTE		16	// 1 block = 16 bytes
#define SEGMENT_TO_BYTE		4	// 1 segment = 4 bytes

//#define LOW_VOLTAGE
#ifdef LOW_VOLTAGE
#define SHA_COMPUTATION_DELAY    4
#define EEPROM_WRITE_DELAY       15
#define SECRET_EEPROM_DELAY      200
#else   //use this in 8909
#define SHA_COMPUTATION_DELAY    4
#define EEPROM_WRITE_DELAY       10
#define SECRET_EEPROM_DELAY      100//<-90
#endif

#define ID_MIN		0
#define ID_MAX		3
#define CO_MIN		0
#define CO_MAX		10
#define ID_DEFAULT	1
#define CO_DEFAULT	1
#define RETRY_LIMIT	10
#define RETRY_DELAY 2 //unit:ms




#define AUTH_PAGE_NO    0

#define BUILD_STAGE_DEV 0
#define BUILD_STAGE_PRD 1
#define BUILD_STAGE         BUILD_STAGE_DEV 

#if     (BUILD_STAGE==BUILD_STAGE_DEV)
#define MODE_STRING "dev"
#elif   (BUILD_STAGE==BUILD_STAGE_PRD)
#define MODE_STRING "prd"
#endif


#define ATTR_SIZE_MODE          4096
#define ATTR_SIZE_DUID          8
#define ATTR_SIZE_SEED          32
#define ATTR_SIZE_MNID          2
#define ATTR_SIZE_DMAC          32
#define ATTR_SIZE_AUTH          16
#define ATTR_SIZE_EXT_SID       8
#define ATTR_SIZE_DBG_PAGE      32
#define ATTR_SIZE_DBG_SECRET    32

///
/// Following data is only for test purpose. serect data will be managed in user space!! 
///
//!binding page data for development. 
//!for production case. binding data should be different and keep confidental.
static u8 g_data_binding_page[32]={ 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
				                    0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10,
				                    0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
				                    0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20};
static u8 g_data_writing_page[32]={ 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
				                    0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10,
				                    0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
				                    0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20};

//!secret data for development. 
static u8 g_data_secret[32]={0x34,0x45,0x92,0x4a,0xcb,0x03,0x9a,0xbd,
								 0x72,0x14,0x56,0x78,0x8b,0x12,0x34,0xbc,
								 0x75,0x34,0x56,0x78,0x3a,0x12,0x41,0x4a,
								 0x32,0x3c,0x56,0x78,0x9b,0x12,0xb2,0xf2};
//!for production case. secret data will be calculated dynamiclly.
static u8 g_data_prd_secbase[32]={0x34,0x45,0x92,0x4a,0xcb,0x03,0x9a,0xbd,
								 0x72,0x14,0x56,0x78,0x8b,0x12,0x34,0xbc,
								 0x75,0x34,0x56,0x78,0x3a,0x12,0x41,0x4a,
								 0x32,0x3c,0x56,0x78,0x9b,0x12,0xb2,0xf2};
static u8 g_data_prd_secreal[32];//calculate from g_data_prd_secbase


//!partial base&challenge data for development. 
//!for production g_data_partial_base should be different and keep confidental.
//!partial_ch data can be dynamic by seed and RNG
static u8 g_data_partial_base[32]={0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
								0x88,0x99,0xAA,0xBB,0xCC,0xDD,0xEE,0xFF,
								0x5F,0x37,0xF1,0x25,0x38,0x52,0x83,0x9E,
								0x7B,0xC9,0xD4,0x00,0xBC,0x47,0x27,0x5B};
static u8 g_data_partial_ch[32]={0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
								0x88,0x99,0xAA,0xBB,0xCC,0xDD,0xEE,0xFF,
								0x5F,0x37,0xF1,0x25,0x38,0x52,0x83,0x9E,
								0x7B,0xC9,0xD4,0x00,0xBC,0x47,0x27,0x5B};

// misc state
static unsigned short slave_crc16;

static int special_mode = 0;
static char special_values[2];
static char rom_no[8];

int verification = -1, id = 2, color, model, detect;
#ifdef CONFIG_W1_SN
char g_sn[14];
#endif

#define READ_EOP_BYTE(seg) (32-seg*4)

/*
static char w1_array[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
				0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10,
				0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
				0x19, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26 };
 */
static u8 skip_setup  = 1;	// for test if the chip did not have secret code, we would need to write temp secret value
static u8 init_verify = 0;	// for inital verifying


//module_param_array(g_data_secret, char, NULL, 0);
//-----------------------------------------------------------------------------
// ------ DS28E15 Functions
//-----------------------------------------------------------------------------

//----------------------------------------------------------------------
// Set or clear special mode flag
//
// 'enable'      - '1' to enable special mode or '0' to clear
//
void set_special_mode(int enable, uchar *values)
{
   special_mode= enable;
   special_values[0] = values[0];
   special_values[1] = values[1];
}

//--------------------------------------------------------------------------
// Calculate a new CRC16 from the input data shorteger.  Return the current
// CRC16 and also update the global variable CRC16.
//
static short oddparity[16] = { 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };

static unsigned short docrc16(unsigned short data)
{
	data = (data ^ (slave_crc16 & 0xff)) & 0xff;
	slave_crc16 >>= 8;

	if (oddparity[data & 0xf] ^ oddparity[data >> 4])
        slave_crc16 ^= 0xc001;

	data <<= 6;
	slave_crc16  ^= data;
	data <<= 1;
	slave_crc16   ^= data;

	return slave_crc16;
}

//--------------------------------------------------------------------------
//  Compute MAC to write a 4 byte memory block using an authenticated
//  write.
//
//  Parameters
//     page - page number where the block to write is located (0 to 15)
//     segment - segment number in page (0 to 7)
//     new_data - 4 byte buffer containing the data to write
//     old_data - 4 byte buffer containing the data to write
//     manid - 2 byte buffer containing the manufacturer ID (general device: 00h,00h)
//     mac - buffer to put the calculated mac into
//
//  Returns: TRUE - mac calculated
//           FALSE - Failed to calculate
//
int calculate_write_authMAC256(int page, int segment, char *new_data, char *old_data, char *manid, char *mac)
{
	char mt[64];

	// calculate MAC
	// clear
	memset(mt,0,64);

	// insert ROM number
	memcpy(&mt[32],rom_no,8);

	mt[43] = segment;
	mt[42] = page;
	mt[41] = manid[0];
	mt[40] = manid[1];

	// insert old data
	memcpy(&mt[44],old_data,4);

	// insert new data
	memcpy(&mt[48],new_data,4);

	// compute the mac
	return compute_mac256(mt, 55, &mac[0]);
}


/* Performs a Compute Next SHA-256 calculation given the provided 32-bytes
   of binding data and 8 byte partial secret. The first 8 bytes of the
   resulting MAC is set as the new secret.
//  Input Parameter:  
    1. unsigned char *binding: 32 byte buffer containing the binding data
    2. unsigned char *partial: 8 byte buffer with new partial secret
    3. int page_nim: page number that the compute is calculated on 
    4. unsigned char *manid: manufacturer ID
   Globals used : SECRET used in calculation and set to new secret
   Returns: TRUE if compute successful
            FALSE failed to do compute
*/
void  calculate_next_secret(unsigned char* binding, unsigned char* partial, int page_num, unsigned char* manid)
{
   unsigned char MT[128];
   unsigned char MAC[64];
   set_secret(g_data_prd_secbase);

   // clear 
   memset(MT,0,128);

   // insert page data
   memcpy(&MT[0],binding,32);

   // insert challenge
   memcpy(&MT[32],partial,32);

   // insert ROM number or FF
   memcpy(&MT[96],rom_no,8);

   MT[106] = page_num;
   MT[105] = manid[0];
   MT[104] = manid[1];

   compute_mac256(MT, 119, MAC);

   // set the new secret to the first 32 bytes of MAC
   memcpy(g_data_prd_secreal, MAC, 32);
   set_secret(MAC);

}


//-----------------------------------------------------------------------------
// ------ DS28E15 Functions - 1 wire command
//-----------------------------------------------------------------------------
//--------------------------------------------------------------------------
//  Write a 4 byte memory block. The block location is selected by the
//  page number and offset blcok within the page. Multiple blocks can
//  be programmed without re-selecting the device using the continue flag.
//  This function does not use the Authenticated Write operation.
//
//  Parameters
//     page - page number where the block to write is located (0, 1)
//     segment - segment number in page (0 to 7)
//     data - 4 byte buffer containing the data to write
//     contflag - Flag to indicate the write is continued from the last (=1)
//
//  Returns: 0 - block written
//               else - Failed to write block (no presence or invalid CRC16)
//
int w1_ds28e1x_write_seg(struct w1_slave *sl, int page, int seg, uchar *data, int contflag)
{
	uchar buf[256],cs;
	int cnt, i, offset;
	int length =4;

	cnt = 0;
	offset = 0;

	if (!sl)
		return -ENODEV;

	if (!contflag)
	{
		if (w1_reset_select_slave(sl))
			return -1;

		buf[cnt++] = CMD_WRITE_MEMORY;
		buf[cnt++] = (seg << 5) | page;   // address

		// Send command
		w1_write_block(sl->master, &buf[0], 2);

		// Read CRC
		w1_read_block(sl->master, &buf[cnt], 2);

		cnt += 2;

		offset = cnt;
	}

	// add the data
	for (i = 0; i < length; i++)
		buf[cnt++] = data[i];

	// Send data
	w1_write_block(sl->master, data, length);

	// Read CRC
	w1_read_block(sl->master, &buf[cnt], 2);
	cnt += 2;

	// check the first CRC16
	if (!contflag)
	{
		slave_crc16 = 0;
		for (i = 0; i < offset; i++)
			docrc16(buf[i]);

		if (slave_crc16 != 0xB001)
			return -1;
	}

	// check the second CRC16
	slave_crc16 = 0;
	for (i = offset; i < cnt; i++)
		docrc16(buf[i]);

	if (slave_crc16 != 0xB001)
		return -1;

	// send release and strong pull-up
	buf[0] = CMD_RELEASE;
	w1_write_block(sl->master, &buf[0], 1);

	// now wait for EEPROM writing.
	msleep(EEPROM_WRITE_DELAY);

	// disable strong pullup

	// read the CS byte
	cs = w1_read_8(sl->master);

	if (cs == 0xAA)
		return 0;
	else
		return cs;

}

//--------------------------------------------------------------------------
//  Write a memory segment. The segment location is selected by the
//  page number and offset segment within the page. Multiple segments can
//  be programmed without re-selecting the device.
//  This function does not use the Authenticated Write operation.
//
//  Parameters
//     page - page number where the block to write is located (0, 1)
//     seg - segment number in page (0 to 7)
//     data - 4 byte multiple buffer containing the data to write
//     length - length to write (4 multiple number, 4, 8, ..., 32)
//
//  Returns: 0 - block written
//               else - Failed to write block (no presence or invalid CRC16)
//
int w1_ds28e1x_write_memory(struct w1_slave *sl, int seg, int page, uchar *data, int length)
{
	uchar buf[256];
	uchar cs=0;
	int i;

	if (!sl)
		return -ENODEV;

	// program one or more contiguous 4 byte segments of a memory block.
	if (length%4)
		return -EINVAL;


	memcpy(buf,data,length);

	for (i=0;i<length/4;i++) {
		cs = w1_ds28e1x_write_seg(sl, page, i, &buf[i*4], (i==0)? 0 : 1);
	}

	return cs;
}

//--------------------------------------------------------------------------
//  Write a 4 byte memory block using an authenticated write (with MAC).
//  The block location is selected by the
//  page number and offset blcok within the page. Multiple blocks can
//  be programmed without re-selecting the device using the continue flag.
//  This function does not use the Authenticated Write operation.
//
//  Parameters
//     page - page number where the block to write is located (0, 1)
//     segment - segment number in page (0 to 7)
//     data - SHA input data for the Authenticated Write Memory including new_data, old_data, and manid
//	new_data - 4 byte buffer containing the data to write
//	old_data - 4 byte buffer containing the data to write
//	manid - 2 byte buffer containing the manufacturer ID (general device: 00h,00h)
//
//  Restrictions
//     The memory block containing the targeted 4-byte segment must not be write protected.
//     The Read/Write Scratchpad command(w1_ds28e1x_write_scratchpad) must have been issued once
//     in write mode after power-on reset to ensure proper setup of the SHA-256 engine.
//
//  Returns: 0 - block written
//               else - Failed to write block (no presence or invalid CRC16)
//
int w1_ds28e1x_write_authblock(struct w1_slave *sl, int page, int segment, uchar *data, int contflag)
{
	uchar buf[256],cs;
	uchar new_data[4], old_data[4], manid[2];
	int cnt, i, offset;

	cnt = 0;
	offset = 0;

	if (!sl)
		return -ENODEV;

	memcpy(new_data, &data[0], 4);
	memcpy(old_data, &data[4], 4);
	memcpy(manid, &data[8], 2);

	if (!contflag) {
		if (w1_reset_select_slave(sl))
			return -1;

		buf[cnt++] = CMD_WRITE_AUTH_MEMORY;
		buf[cnt++] = (segment << 5) | page;   // address

		// Send command
		w1_write_block(sl->master, &buf[0], 2);

		// Read the first CRC
		w1_read_block(sl->master, &buf[cnt], 2);
		cnt += 2;

		offset = cnt;
	}

	// add the data
	for (i = 0; i < 4; i++)
		buf[cnt++] = new_data[i];

	// Send data - first 4bytes
	w1_write_block(sl->master, new_data, 4);

	// read the second CRC
	w1_read_block(sl->master, &buf[cnt], 2);
	cnt += 2;

	// now wait for the MAC computation.
	msleep(SHA_COMPUTATION_DELAY);

	if (!contflag) {
		// check the first CRC16
		slave_crc16 = 0;
		for (i = 0; i < offset; i++)
			docrc16(buf[i]);

		if (slave_crc16 != 0xB001)
			return -1;
	}

	// check the second CRC16
	slave_crc16 = 0;

	for (i = offset; i < cnt; i++)
		docrc16(buf[i]);

	if (slave_crc16 != 0xB001)
		return -1;

	// compute the mac
	if (special_mode)
	{
		if (!calculate_write_authMAC256(page, segment, new_data, old_data, special_values, &buf[0]))
			return -1;
	}
	else
	{
		if (!calculate_write_authMAC256(page, segment, new_data, old_data, manid, &buf[0]))
		return -1;
	}

	// transmit MAC as a block - send the second 32bytes
	cnt=0;
	w1_write_block(sl->master, buf, 32);

	// calculate CRC on MAC
	slave_crc16 = 0;
	for (i = 0; i < 32; i++)
		docrc16(buf[i]);

	// append read of CRC16 and CS byte
	w1_read_block(sl->master, &buf[0], 3);
	cnt = 3;

	// ckeck CRC16
	for (i = 0; i < (cnt-1); i++)
		docrc16(buf[i]);

	if (slave_crc16 != 0xB001)
		return -1;

	// check CS
	if (buf[cnt-1] != 0xAA)
		return -1;

	// send release and strong pull-up
	buf[0] = CMD_RELEASE;
	w1_write_block(sl->master, &buf[0], 1);

	// now wait for the MAC computation.
	msleep(EEPROM_WRITE_DELAY);

	// disable strong pullup


	// read the CS byte
	cs = w1_read_8(sl->master);

	if (cs == 0xAA)
		return 0;
	else
		return cs;

}

//--------------------------------------------------------------------------
//  Write a 4 byte memory block using an authenticated write (with MAC).
//  The MAC must be pre-calculated.
//
//  Parameters
//     page - page number where the block to write is located (0 to 15)
//     segment - segment number in page (0 to 7)
//     new_data - 4 byte buffer containing the data to write
//     mac - mac to use for the write
//
//  Returns: 0 - block written
//               else - Failed to write block (no presence or invalid CRC16)
//
int w1_ds28e1x_write_authblockMAC(struct w1_slave *sl, int page, int segment, uchar *new_data, uchar *mac)
{
	uchar buf[256],cs;
	int cnt, i, offset;

	cnt = 0;
	offset = 0;

	if (!sl)
		return -ENODEV;

	// check if not continuing a previous block write
	if (w1_reset_select_slave(sl))
		return -1;

	buf[cnt++] = CMD_WRITE_AUTH_MEMORY;
	buf[cnt++] = (segment << 5) | page;   // address

	// Send command
	w1_write_block(sl->master, &buf[0], 2);

	// Read CRC
	w1_read_block(sl->master, &buf[cnt], 2);
	cnt += 2;

	offset = cnt;

	// add the data
	for (i = 0; i < 4; i++)
		buf[cnt++] = new_data[i];

	// Send data
	w1_write_block(sl->master, new_data, 4);

	// read first CRC
	w1_read_block(sl->master, &buf[cnt], 2);
	cnt += 2;

	// now wait for the MAC computation.
	msleep(SHA_COMPUTATION_DELAY);

	// disable strong pullup

	// check the first CRC16
	slave_crc16 = 0;
	for (i = 0; i < offset; i++)
		docrc16(buf[i]);

	if (slave_crc16 != 0xB001)
		return -1;

	// check the second CRC16
	slave_crc16 = 0;

	for (i = offset; i < cnt; i++)
		docrc16(buf[i]);

	if (slave_crc16 != 0xB001)
		return -1;

	// transmit MAC as a block
	w1_write_block(sl->master, mac, 32);

	// calculate CRC on MAC
	slave_crc16 = 0;
	for (i = 0; i < 32; i++)
		docrc16(mac[i]);

	// append read of CRC16 and CS byte
	w1_read_block(sl->master, &buf[0], 3);
	cnt = 3;

	// ckeck CRC16
	for (i = 0; i < (cnt-1); i++)
		docrc16(buf[i]);

	if (slave_crc16 != 0xB001)
		return -1;

	// check CS
	if (buf[cnt-1] != 0xAA)
		return -1;

	// send release and strong pull-up
	buf[0] = CMD_RELEASE;
	w1_write_block(sl->master, &buf[0], 1);

	// now wait for the MAC computation.
	msleep(EEPROM_WRITE_DELAY);

	// disable strong pullup

	// read the CS byte
	cs = w1_read_8(sl->master);

	if (cs == 0xAA)
		return 0;
	else
		return cs;
}

//--------------------------------------------------------------------------
//  Read memory. Multiple pages can
//  be read without re-selecting the device using the continue flag.
//
//  Parameters
//	 seg - segment number(0~7) in page
//     page - page number where the block to read is located (0 to 15)
//     rdbuf - 32 byte buffer to contain the data to read
//     length - length to read (allow jsut segment(4bytes) unit) (4, 8, 16, ... , 64)
//
//  Returns: 0 - block read and verified CRC
//               else - Failed to write block (no presence or invalid CRC16)
//
int w1_ds28e1x_read_memory(struct w1_slave *sl, int seg, int page, uchar *rdbuf, int length)
{
	uchar buf[256];
	int cnt, i, offset;

	cnt = 0;
	offset = 0;

	if (!sl)
		return -ENODEV;

	// Check presence detect
	if (w1_reset_select_slave(sl))
		return -1;

	buf[cnt++] = CMD_READ_MEMORY;
	buf[cnt++] = (seg<<5) | page;	 // address

	// Send command
	w1_write_block(sl->master, &buf[0], 2);

	// Read CRC
	w1_read_block(sl->master, &buf[cnt], 2);
	cnt += 2;

	offset = cnt;

	// read data and CRC16
	w1_read_block(sl->master, &buf[cnt], length+2);
	cnt+=length+2;

	// check the first CRC16
	slave_crc16 = 0;
	for (i = 0; i < offset; i++)
		docrc16(buf[i]);

	if (slave_crc16 != 0xB001)
		return -1;

	if (READ_EOP_BYTE(seg) == length) {
		// check the second CRC16
		slave_crc16 = 0;


		for (i = offset; i < cnt; i++)
			docrc16(buf[i]);

		if (slave_crc16 != 0xB001)
			return -1;
	}

	// copy the data to the read buffer
	memcpy(rdbuf,&buf[offset],length);

	return 0;
}


//--------------------------------------------------------------------------
//  Read page and verify CRC. Multiple pages can
//  be read without re-selecting the device using the continue flag.
//
//  Parameters
//     page - page number where the block to write is located (0 to 15)
//     rdbuf - 32 byte buffer to contain the data to read
//
//  Returns: 0 - block read and verified CRC
//               else - Failed to write block (no presence or invalid CRC16)
//
int w1_ds28e1x_read_page(struct w1_slave *sl, int page, uchar *rdbuf)
{
	return w1_ds28e1x_read_memory(sl, 0, page, rdbuf, 32);
}

//--------------------------------------------------------------------------
//  Write page and verify CRC. Multiple pages can
//  be read without re-selecting the device using the continue flag.
//
//  Parameters
//     page - page number where the block to write is located (0 to 1)
//     wtbuf - 32 byte buffer to contain the data to write
//
//  Returns: 0 - block read and verified CRC
//               else - Failed to write block (no presence or invalid CRC16)
//
int w1_ds28e1x_write_page(struct w1_slave *sl, int page, uchar *wtbuf)
{
	return w1_ds28e1x_write_memory(sl, 0, page, wtbuf, 32);
}


//----------------------------------------------------------------------
// Read the scratchpad (challenge or secret)
//
// 'rbbuf'      - 32 byte buffer to contain the data to read
//
// Return: 0 - select complete
//             else - error during select, device not present
//
int w1_ds28e1x_read_scratchpad(struct w1_slave *sl, uchar *rdbuf)
{
	uchar buf[256];
	int cnt=0, i, offset;

	if (!sl)
		return -ENODEV;

	// select device for write
	if (w1_reset_select_slave(sl))
		return -1;

	buf[cnt++] = CMD_SELECT_SECRET;
	buf[cnt++] = 0x0F;

	// Send command
	w1_write_block(sl->master, &buf[0], 2);

	// Read CRC
	w1_read_block(sl->master, &buf[cnt], 2);
	cnt += 2;

	offset = cnt;

	// read data and CRC16
	w1_read_block(sl->master, &buf[cnt], 34);
	cnt+=34;

	// check first CRC16
	slave_crc16 = 0;
	for (i = 0; i < offset; i++)
		docrc16(buf[i]);

	if (slave_crc16 != 0xB001)
		return -1;

	// check the second CRC16
	slave_crc16 = 0;
	for (i = offset; i < cnt; i++)
		docrc16(buf[i]);

	if (slave_crc16 != 0xB001)
		return -1;

	// copy the data to the read buffer
	memcpy(rdbuf,&buf[offset],32);

	return 0;
}

//----------------------------------------------------------------------
// Write the scratchpad (challenge or secret)
//
// 'data'      - data to write to the scratchpad (32 bytes)
//
// Return: 0 - select complete
//             else - error during select, device not present
//
int w1_ds28e1x_write_scratchpad(struct w1_slave *sl, uchar *data)
{
	uchar buf[256];
	int cnt=0, i, offset;

	if (!sl)
		return -ENODEV;


	// select device for write
	if (w1_reset_select_slave(sl))
		return -1;

	buf[cnt++] = CMD_SELECT_SECRET;
	buf[cnt++] = 0x00;

	// Send command
	w1_write_block(sl->master, &buf[0], 2);

	// Read CRC
	w1_read_block(sl->master, &buf[cnt], 2);
	cnt += 2;

	offset = cnt;

	// add the data
	memcpy(&buf[cnt], data, 32);
	cnt+=32;

	// Send the data
	w1_write_block(sl->master, data, 32);

	// Read CRC
	w1_read_block(sl->master, &buf[cnt], 2);
	cnt += 2;

	// check first CRC16
	slave_crc16 = 0;
	for (i = 0; i < offset; i++)
		docrc16(buf[i]);

#if SANITY_DBGCHK
        printk(KERN_ERR "write_scratchpad crc16 chkpoint_I %02X%02X%02X%02X\n",buf[0],buf[1],buf[2],buf[3]);
        if(slave_crc16 != 0xB001)
            printk(KERN_ERR "eRR:crc16==%04X\n", slave_crc16);
#endif

	if (slave_crc16 != 0xB001)
		return -2;

	// check the second CRC16
	slave_crc16 = 0;
	for (i = offset; i < cnt; i++)
		docrc16(buf[i]);

	if (slave_crc16 != 0xB001)
		return -3;

	return 0;
}

//----------------------------------------------------------------------
// Load first secret operation on the DS28E25/DS28E22/DS28E15.
//
// 'lock'      - option to lock the secret after the load (lock = 1)
//
// Restrictions
//             The Read/Write Scratchpad command(w1_ds28e1x_write_scratchpad) must have been issued
//             in write mode prior to Load and Lock Secret to define the secret's value.
//
// Return: 0 - load complete
//             else - error during load, device not present
//
int w1_ds28e1x_load_secret(struct w1_slave *sl, int lock)
{
	uchar buf[256],cs;
	int cnt=0, i;

	if (!sl)
		return -ENODEV;

	// select device for write
	if (w1_reset_select_slave(sl))
		return -1;

	buf[cnt++] = CMD_LOAD_LOCK_SECRET;
	buf[cnt++] = (lock) ? 0xE0 : 0x00;  // lock flag

	// Send command
	w1_write_block(sl->master, &buf[0], 2);

	// Read CRC
	w1_read_block(sl->master, &buf[cnt], 2);
	cnt += 2;

	// check CRC16
	slave_crc16 = 0;
	for (i = 0; i < cnt; i++)
		docrc16(buf[i]);

#if SANITY_DBGCHK
    printk(KERN_ERR "load_secret crc16 chkpoint_I %02X%02X%02X%02X\n",buf[0],buf[1],buf[2],buf[3]);
    if(slave_crc16 != 0xB001)
        printk(KERN_ERR "eRR:crc16==%04X\n", slave_crc16);
#endif

	if (slave_crc16 != 0xB001)
		return -1;

	// send release and strong pull-up
	buf[0] = CMD_RELEASE;
	w1_write_block(sl->master, &buf[0], 1);

	// now wait for the MAC computation.
	// use 100ms for both E11&E15
	//TBD !!from data sheet. E11 is 100ms, E15 200ms(reference code using 100ms, and it works)
	msleep(SECRET_EEPROM_DELAY);
    //msleep(SECRET_EEPROM_DELAY+SECRET_EEPROM_DELAY);

	// disable strong pullup

	// read the CS byte
	cs = w1_read_8(sl->master);

	if (cs == 0xAA)
		return 0;
	else
		return cs;
}


//----------------------------------------------------------------------
// Compute secret operation on the DS28E25/DS28E22/DS28E15.
//
// 'partial'    - partial secret to load (32 bytes)
// 'page_num'   - page number to read 0 - 1
//  'lock'      - option to lock the secret after the load (lock = 1)
//
//  Restrictions
//     The Read/Write Scratchpad command(w1_ds28e1x_write_scratchpad) must have been issued
//     in write mode prior to Compute and Lock Secret to define the partial secret.
//
// Return: 0 - compute complete
//		  else - error during compute, device not present
//
int w1_ds28e1x_compute_secret(struct w1_slave *sl, int page_num, int lock)
{
	uchar buf[256],cs;
	int cnt=0, i;

	if (!sl)
		return -ENODEV;

	page_num = page_num & 0x01;
	// select device for write
	if (w1_reset_select_slave(sl))
		return -1;

	buf[cnt++] = CMD_COMPUTE_LOCK_SECRET;
	buf[cnt++] = (lock) ? (0xE0 | page_num) : page_num;  // lock flag

	// Send command
	w1_write_block(sl->master, &buf[0], 2);

	// Read CRC
	w1_read_block(sl->master, &buf[cnt], 2);
	cnt += 2;

	// check CRC16
	slave_crc16 = 0;
	for (i = 0; i < cnt; i++)
		docrc16(buf[i]);

	if (slave_crc16 != 0xB001)
		return -1;

	// send release and strong pull-up
	buf[0] = CMD_RELEASE;
	w1_write_block(sl->master, &buf[0], 1);

	// now wait for the MAC computation.
	msleep(SHA_COMPUTATION_DELAY * 2 + SECRET_EEPROM_DELAY);

	// disable strong pullup

	// read the CS byte
	cs = w1_read_8(sl->master);

	if (cs == 0xAA)
		return 0;
	else
		return cs;
}


//--------------------------------------------------------------------------
//  Do Compute Page MAC command and return MAC. Optionally do
//  annonymous mode (anon != 0).
//
//  Parameters
//     pbyte - parameter byte including page_num and anon
//	page_num - page number to read 0, 1
//	anon - Flag to indicate Annonymous mode if (anon != 0)
//     mac - 32 byte buffer for page data read
//
//  Restrictions
//     The Read/Write Scratchpad command(w1_ds28e1x_write_scratchpad) must have been issued
//     in write mode prior to Compute and Read Page MAC to define the challenge.
//
//  Returns: 0 - page read has correct MAC
//               else - Failed to read page or incorrect MAC
//
int w1_ds28e1x_compute_read_pageMAC(struct w1_slave *sl, int pbyte, uchar *mac)
{
	uchar buf[256],cs;
	int cnt=0, i;

	if (!sl)
		return -ENODEV;

	// select device for write
	if (w1_reset_select_slave(sl))
		return -1;

	buf[cnt++] = CMD_COMPUTE_PAGEMAC;
	buf[cnt++] = pbyte;

	// Send command
	w1_write_block(sl->master, &buf[0], 2);

	// Read CRC
	w1_read_block(sl->master, &buf[cnt], 2);
	cnt += 2;

	// check CRC16
	slave_crc16 = 0;
	for (i = 0; i < cnt; i++)
		docrc16(buf[i]);

	if (slave_crc16 != 0xB001)
		return -2;

	// now wait for the MAC computation.
	//msleep(SHA_COMPUTATION_DELAY * 2);
	mdelay(SHA_COMPUTATION_DELAY * 2);

	// disable strong pullup

	// read the CS byte
	cs = w1_read_8(sl->master);
	if (cs != 0xAA)
		return -3;

	// read the MAC and CRC
	w1_read_block(sl->master, &buf[0], 34);

	// check CRC16
	slave_crc16 = 0;
	for (i = 0; i < 34; i++)
		docrc16(buf[i]);

	if (slave_crc16 != 0xB001)
		return -4;

	// copy MAC to return buffer
	memcpy(mac, buf, 32);

	return 0;
}

//--------------------------------------------------------------------------
//  Do Read Athenticated Page command and verify MAC. Optionally do
//  annonymous mode (anon != 0).
//
//  Parameters
//     page_num - page number to read 0, 1
//     challange - 32 byte buffer containing the challenge
//     mac - 32 byte buffer for mac read
//     page_data - 32 byte buffer to contain the data to read
//     manid - 2 byte buffer containing the manufacturer ID (general device: 00h,00h)
//     skipread - Skip the read page and use the provided data in the 'page_data' buffer
//     anon - Flag to indicate Annonymous mode if (anon != 0)
//
//  Returns: 0 - page read has correct MAC
//               else - Failed to read page or incorrect MAC
//
int w1_ds28e1x_read_authverify(struct w1_slave *sl, int page_num, uchar *challenge, uchar *page_data, uchar *manid, int skipread, int anon)
{
	uchar mac[32];
	uchar mt[128];
	int pbyte;
	int i = 0, rslt;

	if (!sl)
		return -ENODEV;

	// check to see if we skip the read (use page_data)
	if (!skipread)
	{
		// read the page to get data
		while (i < RETRY_LIMIT) {
			rslt = w1_ds28e1x_read_page(sl, page_num, page_data);
			if (rslt == 0)
				break;

			mdelay(RETRY_DELAY); /* wait 10ms */
			i++;
		}
		if (i >= RETRY_LIMIT) {
			printk(KERN_ERR "w1_ds28e1x_read_authverify err :  -1F\n");
			return -1;
		}
		i = 0;
	}
    else
    {
        //!TBD. for two page devices.
        //memcpy(page_data, g_data_binding_page, 32);
    }
	// The Read/Write Scratch pad command must have been issued in write mode prior
	// to Compute and Read Page MAC to define the challenge
	while (i < RETRY_LIMIT) {
		rslt = w1_ds28e1x_write_scratchpad(sl, challenge);
		if (rslt == 0)
			break;

		mdelay(RETRY_DELAY); /* wait 10ms */
		i++;
	}
	if (i >= RETRY_LIMIT) {
		printk(KERN_ERR "w1_ds28e1x_read_authverify err :  -2F\n");
		return -2;
	}
	i = 0;

	// have device compute mac
	pbyte = anon ? 0xE0 : 0x00;
	pbyte = pbyte | page_num;
	while (i < RETRY_LIMIT) {
		rslt = w1_ds28e1x_compute_read_pageMAC(sl, pbyte, mac);
		if (rslt == 0)
			break;

		mdelay(RETRY_DELAY); /* wait 10ms */
		i++;
	}
	if (i >= RETRY_LIMIT) {
		printk(KERN_ERR "w1_ds28e1x_read_authverify err :  -3\n");
		return -3;
	}
	// create buffer to compute and verify mac

	// clear
	memset(mt,0,128);

	// insert page data
	memcpy(&mt[0],page_data,32);

	// insert challenge
	memcpy(&mt[32],challenge,32);

	// insert ROM number or FF
	if (anon)
		memset(&mt[96],0xFF,8);
	else
		memcpy(&mt[96],rom_no,8);

	mt[106] = page_num;

	if (special_mode)
	{
		mt[105] = special_values[0];
		mt[104] = special_values[1];
	}
	else
	{
		mt[105] = manid[0];
		mt[104] = manid[1];
	}

	if (verify_mac256(mt, 119, mac) ==0) {
		printk(KERN_ERR "w1_ds28e1x_compute_read_pageMAC err :  -4\n");
		return -4;
	} else
		return 0;
}


//--------------------------------------------------------------------------
//  Verify provided MAC and page data. Optionally do
//  annonymous mode (anon != 0).
//
//  Parameters
//     page_num - page number to read 0 - 16
//     challange - 32 byte buffer containing the challenge
//     page_data - 32 byte buffer to contain the page data
//     manid - 2 byte buffer containing the manufacturer ID (general device: 00h,00h)
//     mac - 32 byte buffer of mac read
//     anon - Flag to indicate Annonymous mode if (anon != 0)
//
//  Returns: 0 - page read has correct MAC
//               else - Failed to read page or incorrect MAC
//
int w1_ds28e1x_authverify(struct w1_slave *sl, int page_num, uchar *challenge, uchar *page_data, uchar *manid, uchar *mac, int anon)
{
	uchar mt[128];

	// create buffer to compute and verify mac

	// clear
	memset(mt,0,128);

	// insert page data
	memcpy(&mt[0],page_data,32);

	// insert challenge
	memcpy(&mt[32],challenge,32);

	// insert ROM number or FF
	if (anon)
		memset(&mt[96],0xFF,8);
	else
		memcpy(&mt[96],rom_no,8);

	mt[106] = page_num;

	if (special_mode)
	{
		mt[105] = special_values[0];
		mt[104] = special_values[1];
	}
	else
	{
		mt[105] = manid[0];
		mt[104] = manid[1];
	}

	if (verify_mac256(mt, 119, mac) == 0)
		return -1;
	else
		return 0;
}

//--------------------------------------------------------------------------
//  Read status bytes, either personality or page protection.
//
//  Parameters
//     pbyte - include personality, allpages, and page_num
//	personality - flag to indicate the read is the 4 personality bytes (1)
//		      or page page protection (0)
//	allpages - flag to indicate if just one page (0) or all (1) page protection
//		      bytes.
//	block_num - block number if reading protection 0 to 3
//     rdbuf - 16 byte buffer personality bytes (length 4) or page protection
//            (length 1 or 16)
//
//  Returns: 0 - status read
//               else - Failed to read status
//
int w1_ds28e1x_read_status(struct w1_slave *sl, int pbyte, uchar *rdbuf)
{
	uchar buf[256];
	int cnt, i, offset,rdnum;
	int personality, allpages, block_num;

	if (!sl)
		return -ENODEV;

	personality = (pbyte & 0xE0) ? 1 : 0;
	allpages = (pbyte == 0) ? 1 : 0;
	block_num = pbyte & 0x03;

	cnt = 0;
	offset = 0;

	if (w1_reset_select_slave(sl)){
		pr_info("%s reset_select_slave error", __func__);
		return -1;
	}

	buf[cnt++] = CMD_READ_STATUS;
	if (personality)
		buf[cnt++] = 0xE0;
	else if (!allpages)
		buf[cnt++] = block_num;
	else
		buf[cnt++] = 0;

	// send the command
	w1_write_block(sl->master, &buf[0], 2);
	offset = cnt + 2;

	// adjust data length
	if ((personality) || (allpages))
		rdnum = 8;
	else
		rdnum = 5;

	// Read the bytes
	w1_read_block(sl->master, &buf[cnt], rdnum);
	cnt += rdnum;

	// check the first CRC16
	slave_crc16 = 0;
	for (i = 0; i < offset; i++)
		docrc16(buf[i]);

	if (slave_crc16 != 0xB001){
		pr_info("%s slave_crc16 is error 1.\n", __func__);
		return -1;
	}

	if ((personality || allpages || (block_num == 1)))
	{
		// check the second CRC16
		slave_crc16 = 0;
		for (i = offset; i < cnt; i++)
			docrc16(buf[i]);

		if (slave_crc16 != 0xB001){
			pr_info("%s slave_crc16 is error 2.\n", __func__);
			return -1;
		}
	}

	// copy the data to the read buffer
	memcpy(rdbuf,&buf[offset],rdnum-4);
	return 0;
}

//--------------------------------------------------------------------------
//  Write page protection byte.
//
//  Parameters
//     block - block number (0 to 3) which covers two pages each
//     prot - protection byte
//
//  Returns: 0 - protection written
//               else - Failed to set protection
//
int w1_ds28e1x_write_blockprotection(struct w1_slave *sl, uchar block, uchar prot)
{
	uchar buf[256],cs;
	int cnt=0, i;

	if (!sl)
		return -ENODEV;

	// select device for write
	if (w1_reset_select_slave(sl))
		return -1;

	buf[cnt++] = CMD_WRITE_BLOCK_PROTECT;

	// compute parameter byte
	buf[cnt++] = prot|block;

	w1_write_block(sl->master, &buf[0], cnt);

	// Read CRC
	w1_read_block(sl->master, &buf[cnt], 2);
	cnt += 2;

	// check CRC16
	slave_crc16 = 0;
	for (i = 0; i < cnt; i++)
		docrc16(buf[i]);

	if (slave_crc16 != 0xB001)
		return -1;

	// sent release

	// now wait for programming
	msleep(EEPROM_WRITE_DELAY);

	// disable strong pullup

	// read the CS byte
	cs = w1_read_8(sl->master);

	if (cs == 0xAA)
		return 0;
	else
		return cs;
}

//--------------------------------------------------------------------------
//  Write page protection byte.
//
//  Parameters
//     data - input data for Authenticated Write Block Protection
//	new_value - new protection byte(parameter byte) including block num
//	old_value - old protection byte(parameter byte)
//	manid - manufacturer ID
//
//  Returns: 0 - protection written
//               else - Failed to set protection
//
int w1_ds28e1x_write_authblockprotection(struct w1_slave *sl, uchar *data)
{
	uchar buf[256],cs,mt[64];
	int cnt=0, i;
	int new_value, old_value;
	uchar manid[2];

	if (!sl)
		return -ENODEV;

	new_value = data[0];
	old_value = data[1];
	manid[0] = data[2];
	manid[1] = data[3];

	// select device for write
	if (w1_reset_select_slave(sl))
		return -1;

	buf[cnt++] = CMD_WRITE_AUTH_PROTECT;
	buf[cnt++] = new_value;

	// Send command
	w1_write_block(sl->master, &buf[0], 2);

	// read first CRC
	w1_read_block(sl->master, &buf[cnt], 2);

	// now wait for the MAC computation.
	msleep(SHA_COMPUTATION_DELAY);

	// disable strong pullup

	// check CRC16
	slave_crc16 = 0;
	for (i = 0; i < cnt; i++)
		docrc16(buf[i]);

	if (slave_crc16 != 0xB001)
		return -1;

	// calculate MAC
	// clear
	memset(mt,0,64);

	// insert ROM number
	memcpy(&mt[32],rom_no,8);

	// instert block and page
	mt[43] = 0;
	mt[42] = new_value & 0x0F;

	// check on special mode
	if (special_mode)
	{
		mt[41] = special_values[0];
		mt[40] = special_values[1];
	}
	else
	{
		mt[41] = manid[0];
		mt[40] = manid[1];
	}

	// old data
	mt[44] = (old_value & PROT_BIT_AUTHWRITE) ? 0x01 : 0x00;
	mt[45] = (old_value & PROT_BIT_EPROM) ? 0x01 : 0x00;
	mt[46] = (old_value & PROT_BIT_WRITE) ? 0x01 : 0x00;
	mt[47] = (old_value & PROT_BIT_READ) ? 0x01 : 0x00;
	// new data
	mt[48] = (new_value & PROT_BIT_AUTHWRITE) ? 0x01 : 0x00;
	mt[49] = (new_value & PROT_BIT_EPROM) ? 0x01 : 0x00;
	mt[50] = (new_value & PROT_BIT_WRITE) ? 0x01 : 0x00;
	mt[51] = (new_value & PROT_BIT_READ) ? 0x01 : 0x00;

	// compute the mac
	compute_mac256(mt, 55, &buf[0]);
	cnt = 32;

	// send the MAC
	w1_write_block(sl->master, &buf[0], 32);

	// Read CRC and CS byte
	w1_read_block(sl->master, &buf[cnt], 3);
	cnt += 3;

	// ckeck CRC16
	slave_crc16 = 0;
	for (i = 0; i < (cnt-1); i++)
		docrc16(buf[i]);

	if (slave_crc16 != 0xB001)
		return -1;

	// check CS
	if (buf[cnt-1] != 0xAA)
		return -1;

	// send release and strong pull-up
	// DATASHEET_CORRECTION - last bit in release is a read-zero so don't check echo of write byte
	w1_write_8(sl->master, 0xAA);

	// now wait for the MAC computation.
	msleep(EEPROM_WRITE_DELAY);

	// disable strong pullup

	// read the CS byte
	cs = w1_read_8(sl->master);

	if (cs == 0xAA)
		return 0;
	else
		return cs;
}

//-------------------------------------------------------------------------
// get_array_value
//
#ifdef CONFIG_OF_SUBCMDLINE_PARSE
static int get_array_value(void)
{
	int i, ret;
	char str[20], *buf,  data;

	pr_info("%s: W1 Read Array.\n", __func__);
	ret = of_parse_args_on_subcmdline("array=",(char *)str);
	if (ret) {
		pr_err("%s: W1 Read Array Error\n", __func__);
		return -1;
	}
	buf = str;
	for(i=0;i<5;i++){
		strncpy(&data,&buf[i*2],2);
		sscanf(&data, "%x", (unsigned int *)&ret);
		g_data_secret[i]=ret;
	}
	return 0;
}
#else
static int __init get_array_value(char *str)
{
	int i, get[6];

	pr_info("%s: W1 Read Array\n", __func__);

	for(i=0;i<5;i++){
		sscanf(str, "%x", &get[i]);
		g_data_secret[i]=get[i];
		str=str+3;
	}
	return 0;
}
__setup("array=", get_array_value);
#endif


//--------------------------------------------------------------------------
//  w1_ds28e1x_verifymac
//
//  compare two mac data, Device mac and calculated mac and
//  returned the result
//  Returns: 0 - Success, have the same mac data in both of Device and AP.
//               else - Failed, have the different mac data or verifying sequnce failed.
//
int w1_ds28e1x_verifymac(struct w1_slave *sl)
{
	int rslt,rt;
	uchar buf[256], challenge[32], manid[2];
	uchar memimage[512];
	uchar master_secret[32];
#if 1
	int i = 0;
#endif

	// copy the secret code
	#if (BUILD_STAGE==BUILD_STAGE_DEV)
	memcpy(master_secret, g_data_secret, 32); // Set temp_secret to the master secret
	#else //BUILD_STAGE==BUILD_STAGE_PRD
	memcpy(master_secret, g_data_prd_secreal, 32); // Set temp_secret to the master secret    
    #endif
	rt = 0;

	// Store the master secret and romid.
	set_secret(master_secret);
	set_romid(rom_no);

	// read personality bytes to get manufacturer ID
#if 1
	while (i < RETRY_LIMIT) {
		rslt = w1_ds28e1x_read_status(sl, 0xE0, buf);
		if (rslt == 0)
			break;

		mdelay(RETRY_DELAY); /* wait 10ms */
		i++;
	}
	i = 0;
#else
	rslt = w1_ds28e1x_read_status(sl, 0xE0, buf);
#endif
	if (rslt == 0)
	{
		manid[0] = buf[3];
		manid[1] = buf[2];
	} else {
		pr_info("%s : read_status error\n", __func__);
		rt = -1;
		goto success;
	}

	// if you want to use random value, insert code here.
	//!!![XXX]get_random_bytes(challenge, 32);  // random challenge
	memcpy(challenge, g_data_partial_ch,32);

	// Enhance verification status
	while (i < RETRY_LIMIT) {
		rslt = w1_ds28e1x_read_authverify(sl, 0, challenge,
					g_data_binding_page, manid, 1, 0);
		if (rslt == 0)
			break;

        printk(KERN_ERR "retry %d/%d\n", i+1, RETRY_LIMIT);
		mdelay(RETRY_DELAY); /* wait 10ms */
		i++;
	}
	printk(KERN_ERR "%s  result : %d\n",__func__, rslt);

	if (rslt){
		pr_info("%s: read_authverify error\n", __func__);
		rt = -1;
	}

success:
	return rt;
}

static int w1_ds28e1x_setup_device(struct w1_slave *sl)
{
	int rslt,rt;
	uchar buf[256], challenge[32], manid[2];
	uchar memimage[512];
	uchar master_secret[32];

	// hard code master secret
	memcpy(master_secret, g_data_secret, 32); // Set temp_secret to the master secret

	// ----- DS28EL25/DS28EL22/DS28E15/DS28E11 Setup
	printk(KERN_ERR "-------- DS28E1X Setup Example\n");
	rt = 0;

	printk(KERN_ERR "Write scratchpad - master secret\n");
	rslt = w1_ds28e1x_write_scratchpad(sl, master_secret);
	printk(KERN_ERR "result : %d\n", rslt);
	if (rslt) rt = -1;

	printk(KERN_ERR "Load the master secret\n");
	rslt = w1_ds28e1x_load_secret(sl, 0);
	printk(KERN_ERR "result : %d\n",rslt);
	if (rslt) rt = -1;

    printk(KERN_ERR "Load the binding page\n");
    rslt = w1_ds28e1x_write_page(sl, 0, g_data_binding_page);
    printk(KERN_ERR "result : %d\n",rslt);
	if (rslt) rt = -1;

	printk(KERN_ERR "Set the master secret in the Software SHA-256\n");
	set_secret(master_secret);
	set_romid(rom_no);

	// fill image with random data to write
	// read personality bytes to get manufacturer ID
	printk(KERN_ERR " w1_ds28e1x_read_status(\n");
	rslt = w1_ds28e1x_read_status(sl, 0xE0, buf);
	if (rslt == 0) {
		manid[0] = buf[3];
		manid[1] = buf[2];
	}
	else{
		rt = -1;
		goto end;
	}
	printk(KERN_ERR "result : %d\n",rslt);

	printk(KERN_ERR "Read-Authenticate with unique secret\n");
	get_random_bytes(challenge, 32);  // random challenge
	rslt = w1_ds28e1x_read_authverify(sl, 0, challenge, g_data_binding_page, manid, 1, 0);
	printk(KERN_ERR "result : %d\n",rslt);
	if (rslt) rt = -1;

	printk(KERN_ERR "DS28E1X Setup Example: %s\n",(rt) ? "FAIL" : "SUCCESS");
	printk(KERN_ERR "--------------------------------------------------\n");
end:
	return rt;

}


static int w1_ds28e1x_setup_for_production(struct w1_slave *sl)
{
    int i;
	int rslt,rt;
	uchar buf[256], challenge[32], manid[2];
	uchar memimage[512];
	uchar master_secret[32];

	// hard code master secret
	memcpy(master_secret, g_data_prd_secbase, 32); // Set temp_secret to the master secret

	// ----- DS28EL25/DS28EL22/DS28E15 Setup
	printk(KERN_ERR "-------- DS28E15 Setup Example\n");
	rt = 0;

	printk(KERN_ERR "Write scratchpad - master secret\n");
	rslt = w1_ds28e1x_write_scratchpad(sl, master_secret);
	printk(KERN_ERR "result : %d\n", rslt);
	if (rslt) rt = -1;

	printk(KERN_ERR "Load the master secret\n");
	rslt = w1_ds28e1x_load_secret(sl, 0);
	printk(KERN_ERR "result : %d\n",rslt);
	if (rslt) rt = -1;

    printk(KERN_ERR "Load the binding page\n");
    rslt = w1_ds28e1x_write_page(sl, 0, g_data_binding_page);
    printk(KERN_ERR "result : %d\n",rslt);
	if (rslt) rt = -1;

	printk(KERN_ERR "Set the master secret in the Software SHA-256\n");
	set_secret(master_secret);
	set_romid(rom_no);

	// fill image with random data to write
	// read personality bytes to get manufacturer ID
	printk(KERN_ERR " w1_ds28e1x_read_status(\n");
    // read personality bytes to get manufacturer ID
    i = 0;
    while (i < RETRY_LIMIT) {
        rslt = w1_ds28e1x_read_status(sl, 0xE0, buf);
        if (rslt == 0)
            break;

        mdelay(RETRY_DELAY); /* wait 10ms */
        i++;
    }

    if (rslt == 0){
        manid[0] = buf[3];
        manid[1] = buf[2];
    } else {
        printk(KERN_ERR "%s : read_status error\n", __func__);
        rt = -1;
        goto end;
    }
	printk(KERN_ERR "result : %d\n",rslt);

	printk(KERN_ERR "1st Read-Authenticate with unique secret\n");
	get_random_bytes(challenge, 32);  // random challenge
	rslt = w1_ds28e1x_read_authverify(sl, 0, challenge, g_data_binding_page, manid, 1, 0);
	printk(KERN_ERR "1st result : %d\n",rslt);
	if (rslt) rt = -1;

    //generate 
	// The Read/Write Scratch pad command must have been issued in write mode prior
	// to Compute and Read Page MAC to define the challenge
	i = 0;
	while (i < RETRY_LIMIT) {
		rslt = w1_ds28e1x_write_scratchpad(sl, g_data_partial_base);
		if (rslt == 0)
			break;

        printk(KERN_ERR "retry write scratchpad %d/%d\n", i+1, RETRY_LIMIT);
		mdelay(RETRY_DELAY); /* wait 10ms */
		i++;
	}
	if (i >= RETRY_LIMIT) {
		printk(KERN_ERR "w1_ds28e1x_setup_for_production err :  -2F\n");
		rt = -2;
        goto end;
	}

    rslt = w1_ds28e1x_compute_secret(sl, 0, 0);
    printk(KERN_ERR "compute_secret result:%d\n",rslt);
   	if (rslt) rt = -1;

    calculate_next_secret(g_data_binding_page, g_data_partial_base, 0, manid);

    printk(KERN_ERR "2nd Read-Authenticate with unique secret\n");
    rslt = w1_ds28e1x_read_authverify(sl, 0, challenge, g_data_binding_page, manid, 1, 0);
    printk(KERN_ERR "2nd result : %d\n",rslt);
	if (rslt) rt = -1;

	printk(KERN_ERR "DS28E15 Setup Example: %s\n",(rt) ? "FAIL" : "SUCCESS");
	printk(KERN_ERR "--------------------------------------------------\n");
end:
	return rt;

}

//--------------------------------------------------------------------------
/*Read the 64-bit ROM ID of DS28E15
// Input parameter: 
   1.RomID  :64Bits RomID Receiving Buffer 
                    
// Returns: 0     = success ,RomID CRC check is right ,RomID Sotred in RomID Buffer
            other = failure ,maybe on_reset() Error ,or CRC check Error;
*/
int w1_ds28e1x_read_romid(struct w1_slave *sl, unsigned char *RomID)
{
    uchar buf[256];
    int cnt, i, offset;
    u8 crc8;

    //reset
    if (w1_reset_bus(sl->master))
    	return -1;

    if (sl->master->slave_count == 1)
        w1_write_8(sl->master, W1_READ_ROM);
    else
        return -1;

    udelay(10);
    for(i = 0;i < 8;i++)
    {
        RomID[i] = w1_read_8(sl->master);
    }

    crc8 = w1_calc_crc8(RomID, 8);
  
    //if Receiving No  Error ,CRC =0;
    if(crc8!=0)
        return -2;
    else
        return 0;
}

/*
 w1 slave device node attribute directory layout is as follows:
 -->/sys/bus/w1/devices/
  devname_dir/          #device name. format: 17-xxxxxxxxxxxx (for E11 case)
    ������ mode         #dev|prd. development<->debug. prd:production<->release.        {string}{RO}
    ������ duid         #device unique id.(serial num of maxim chip).==devname_dir      {binary}{RO}
    ������ seed         #seed for secure processing. (e.g. partial data / auth result)  {binary}{RW}
    ������ mnid         #read device manufacture id                                     {binary}{RO}
    ������ dmac         #read computed double SHA-256 MAC Result Data                   {binary}{RO}
    ������ auth         #Authenticator MAC Verify result. in dev mode: 0:OK 1:NG        {binary}{RO}
    ������ dbg_page     #page(0) data inject (mainly for debug in develoment)           {binary}{WO}
    ������ dbg_secret   #secert key data inject (mainly for debug in develoment)        {binary}{WO}

    PS: following attribute can be added for support for 2-level vendor lic case using E15
    ������ ext_sid      #vendor specific sid if exist, default:0x00000000               {binary}{RW} suggest write-once.
    
 */

static ssize_t w1_ds28e1x_if_mode_read(struct device *device,
	struct device_attribute *attr, char *buf)
{
	// read cover model
	return sprintf(buf, "%s\n", MODE_STRING);
}

static ssize_t w1_ds28e1x_if_duid_read(struct device *device,
	struct device_attribute *attr, char *buf)
{
    struct w1_slave *sl = dev_to_w1_slave(device);

    #if 0 //! it seems no need to read on-the-fly
    int ret = w1_ds28e1x_read_romid(sl, buf);
    if(0==ret)
        return ATTR_SIZE_DUID;
    else
        return -EFAULT;
    #else
    memcpy(buf, rom_no, ATTR_SIZE_DUID);
    return ATTR_SIZE_DUID;
    #endif
}

static ssize_t w1_ds28e1x_if_mnid_read(struct device *device,
	struct device_attribute *attr, char *buf)
{
    int i = 0;
    int ret;
    uchar data[8];
    struct w1_slave *sl = dev_to_w1_slave(device);

    while (i < RETRY_LIMIT) {
        ret = w1_ds28e1x_read_status(sl, 0xE0, data);
        if (ret == 0)
            break;

        mdelay(RETRY_DELAY);
        i++;
    }

    if (ret == 0){
        buf[0] = data[3];
        buf[1] = data[2];
        return ATTR_SIZE_MNID;
    } 
    else 
    {
        printk(KERN_ERR "%s : read manid error\n", __func__);
        return -1;
    }

}


static ssize_t w1_ds28e1x_if_seed_read(struct device *device,
	struct device_attribute *attr, char *buf)
{
    //memcpy(buf, g_data_partial_ch, ATTR_SIZE_SEED);
    //do real read for debug
    struct w1_slave *sl = dev_to_w1_slave(device);
    if(0==w1_ds28e1x_read_scratchpad(sl, buf))
        return ATTR_SIZE_SEED;
    else
        return -1;
}

static ssize_t w1_ds28e1x_if_seed_write(struct device *device,
	struct device_attribute *attr, const char *buf, size_t count)
{
	if (count < ATTR_SIZE_SEED)
    {
        printk(KERN_ERR "%s: not enough seed data - %d\n", __func__, count);
		return -EFAULT;
    }

    memcpy(g_data_partial_ch, buf, ATTR_SIZE_SEED);
    return ATTR_SIZE_SEED;
}


static ssize_t w1_ds28e1x_if_dmac_read(struct device *device,
	struct device_attribute *attr, char *buf)
{
    int i = 0;
    int ret = 0;
    struct w1_slave *sl = dev_to_w1_slave(device);

	// The Read/Write Scratch pad command must have been issued in write mode prior
	// to Compute and Read Page MAC to define the challenge
	while (i < RETRY_LIMIT) {
        ret |= w1_ds28e1x_write_scratchpad(sl, g_data_partial_ch);
        if(ret)
        {
            printk(KERN_ERR "%s: scratchpad{NG)&compute_read_pageMAC retry %d/%d [ret=%d]\n", __FUNCTION__, i+1, RETRY_LIMIT, ret);
            i++;
            ret = 0;
            mdelay(RETRY_DELAY);
            continue;            
        }
        ret |= w1_ds28e1x_compute_read_pageMAC(sl, 0, buf);
        if (ret == 0)
            break;

        printk(KERN_ERR "%s: scratchpad&compute_read_pageMAC(NG) retry %d/%d [ret=%d]\n", __FUNCTION__, i+1, RETRY_LIMIT, ret);
        mdelay(RETRY_DELAY); /* wait 10ms */
        ret = 0;
        i++;
	}

	if (i >= RETRY_LIMIT) {
		printk(KERN_ERR "%s Failed to Get Expected MAC\n", __FUNCTION__);
		return 0;
	}

    return ATTR_SIZE_DMAC;
}

static ssize_t w1_ds28e1x_if_auth_read(struct device *device,
	struct device_attribute *attr, char *buf)
{
    struct w1_slave *sl = dev_to_w1_slave(device);
    int ret = w1_ds28e1x_verifymac(sl);

    //!for develop, use 1st byte only
    buf[0] = ret?1:0;
    //!from 2nd byte, used for human reading.
    sprintf(&buf[1], "result is %d {0:OK ; 1:NG}\n\0", ret?1:0);
    return ATTR_SIZE_AUTH;
}

// verified mac value between the device and AP
static ssize_t w1_ds28e1x_if_page_read(struct device *device,
	struct device_attribute *attr, char *buf)
{
    #if 0000//!!only for internal debug usage!!
    memcpy(buf, g_data_binding_page, ATTR_SIZE_DBG_PAGE);
    return ATTR_SIZE_DBG_PAGE;
    #endif

    /*read nothing*/
    memset(buf, 0, ATTR_SIZE_DBG_PAGE);
    return ATTR_SIZE_DBG_PAGE;

}

static ssize_t w1_ds28e1x_if_page_write(struct device *device,
	struct device_attribute *attr, const char *buf, size_t count)
{
    int ret;
    int i = 0;
    char pageread[ATTR_SIZE_DBG_PAGE];
    struct w1_slave *sl = dev_to_w1_slave(device);
	if (count < ATTR_SIZE_DBG_PAGE)
    {
    	printk(KERN_ERR "%s: not enough binding data for page - %d\n", __func__, count);
		return -EFAULT;
    }

    while (i < RETRY_LIMIT) {
        ret = w1_ds28e1x_write_page(sl, 0, buf);
        if (ret == 0)
            break;

        if(ret==0x33 || ret==0x55 || ret==0x53)
        {
            printk(KERN_ERR "WARNING-%s: page already LOCKED!! - %d\n", __func__,ret);
            memcpy(g_data_writing_page, buf, ATTR_SIZE_DBG_PAGE);
            return ATTR_SIZE_DBG_PAGE;
        }


        printk(KERN_ERR "%s: writing binding data failed retry %d/%d\n", __func__, i+1, RETRY_LIMIT);
        mdelay(RETRY_DELAY); /* wait 10ms */
        i++;
    }

    if(0!=ret)
    {
        printk(KERN_ERR "%s: writing binding data failed \n", __func__);
        return 0;
    }

    #if ENABLE_KEYDATA_LOCK
    //doubel checking by data reading back
    while (i < RETRY_LIMIT) {
        ret = w1_ds28e1x_read_page(sl, 0, pageread);
        if (ret == 0)
        {
            if(0==memcmp(buf, pageread, ATTR_SIZE_DBG_PAGE))
                break;
            else
                printk(KERN_ERR "%s: double checking binding data(readback) diff FAILED\n", __func__);
        }

        printk(KERN_ERR "%s: double checking binding data retry %d/%d\n", __func__, i+1, RETRY_LIMIT);
        mdelay(RETRY_DELAY); /* wait 10ms */
        i++;
    }

    //page lock
    if(0==ret)
    {
        printk(KERN_ERR "%s: page write verified, lock the page\n", __func__);
        ret = w1_ds28e1x_write_blockprotection(sl, 1, PROT_BIT_READ|PROT_BIT_AUTHWRITE);
    }
    #endif
    
    if(0==ret)
    {
        //sync data with running code for further checking. 
        memcpy(g_data_writing_page, buf, ATTR_SIZE_DBG_PAGE);
        return ATTR_SIZE_DBG_PAGE;
    }
    else
        return 0;
}

// verified mac value between the device and AP
static ssize_t w1_ds28e1x_if_secret_read(struct device *device,
	struct device_attribute *attr, char *buf)
{
    #if 0000//!!only for internal debug usage!!
    memcpy(buf, g_data_secret, ATTR_SIZE_DBG_SECRET);
    return ATTR_SIZE_DBG_SECRET;
    #endif

    /*read nothing*/
    memset(buf, 0, ATTR_SIZE_DBG_SECRET);
    return ATTR_SIZE_DBG_SECRET;

}

static ssize_t w1_ds28e1x_if_secret_write(struct device *device,
	struct device_attribute *attr, const char *buf, size_t count)
{
    int i = 0;
    int ret = 0;
    uchar challenge[32], manid[2];
    uchar readback[32];
    struct w1_slave *sl = dev_to_w1_slave(device);
	if (count < ATTR_SIZE_DBG_SECRET)
    {
    	printk(KERN_ERR "%s: not enough secret data - %d\n", __func__, count);
		return 0;
    }

    //for manid
    while (i < RETRY_LIMIT) {
        ret = w1_ds28e1x_read_status(sl, 0xE0, challenge);
        if (ret == 0)
            break;

        mdelay(RETRY_DELAY); /* wait 10ms */
        i++;
    }

    if (ret == 0){
        manid[0] = challenge[3];
        manid[1] = challenge[2];
    } else {
        printk(KERN_ERR "%s : read_status error\n", __func__);
        ret = -1;
        goto end;
    }

    //for writing scratchpad
	i = 0;
	while (i < RETRY_LIMIT) {
		ret |= w1_ds28e1x_write_scratchpad(sl, buf);
        ret |= w1_ds28e1x_load_secret(sl, 0);

		if (ret == 0)
			break;

        printk(KERN_ERR "retry write scratchpad & load secret %d/%d\n", i+1, RETRY_LIMIT);
		mdelay(RETRY_DELAY);
		i++;
        ret = 0;
	}
	if (i >= RETRY_LIMIT) {
		printk(KERN_ERR "%s: load secret failed\n", __func__);
		ret = -2;
        goto end;
	}

	set_secret(buf);
	set_romid(rom_no);

    //verify MAC for checking whether secret is correct or not.
    get_random_bytes(challenge, 32);  // random challenge
    ret = w1_ds28e1x_read_authverify(sl, 0, challenge, g_data_writing_page, manid, 1, 0);

    #if ENABLE_KEYDATA_LOCK
    if(0!=ret)
        goto end;

    //Do Lock
    i = 0;
    while (i < RETRY_LIMIT) {
        ret |= w1_ds28e1x_write_scratchpad(sl, buf);
        ret |= w1_ds28e1x_read_scratchpad(sl, readback);
        ret |=memcmp(buf, readback, ATTR_SIZE_DBG_SECRET);
        if (ret != 0)
        {
            printk(KERN_ERR "[lock loop]%s: scratchpad write&readback checkfailed%d/%d\n", i+1, RETRY_LIMIT);
            continue;
        }
        ret |= w1_ds28e1x_load_secret(sl, 1);

        if (ret == 0)
            break;

        printk(KERN_ERR "[lock_loop]retry write scratchpad & load secret %d/%d\n", i+1, RETRY_LIMIT);
        mdelay(RETRY_DELAY);
        i++;
        ret = 0;
    }
    if (i >= RETRY_LIMIT) {
        printk(KERN_ERR "[lock loop]%s: load secret failed\n", __func__);
        ret = -2;
        goto end;
    }

    set_secret(buf);
    set_romid(rom_no);

    //verify MAC for checking whether secret is correct or not.
    get_random_bytes(challenge, 32);  // random challenge
    ret = w1_ds28e1x_read_authverify(sl, 0, challenge, g_data_writing_page, manid, 1, 0);
    printk(KERN_ERR "[lock loop]%s: load secret with lock OK!\n", __func__);
    #endif

end:
    if(0==ret)
    {
        printk(KERN_ERR "SUCCESS Do Secret Loading!!\n");
        return ATTR_SIZE_DBG_SECRET;
    }
    else
    {
        printk(KERN_ERR "FAILED Do Secret Loading!!\n");
        return 0;
    }

}


static struct device_attribute w1_ds28e1x_if_lists[] =
{
	__ATTR(mode, S_IRUGO, w1_ds28e1x_if_mode_read, NULL),
	__ATTR(duid, S_IRUGO, w1_ds28e1x_if_duid_read, NULL),
    __ATTR(mnid, S_IRUGO, w1_ds28e1x_if_mnid_read, NULL),
    __ATTR(dmac, S_IRUGO, w1_ds28e1x_if_dmac_read, NULL),
	__ATTR(auth, S_IRUGO, w1_ds28e1x_if_auth_read, NULL),
	__ATTR(seed, S_IRUGO|S_IWUGO, w1_ds28e1x_if_seed_read, w1_ds28e1x_if_seed_write),	
	__ATTR(dbg_page,   S_IRUGO|S_IWUGO, w1_ds28e1x_if_page_read, w1_ds28e1x_if_page_write),
	__ATTR(dbg_secret, S_IRUGO|S_IWUGO, w1_ds28e1x_if_secret_read, w1_ds28e1x_if_secret_write),
};

static int w1_ds28e1x_get_buffer(struct w1_slave *sl, uchar *rdbuf, int retry_limit)
{
	int ret = -1, retry = 0;

	while((ret != 0) && (retry < retry_limit)) {
		ret = w1_ds28e1x_read_page(sl, 0, &rdbuf[0]);
		if(ret != 0 )
			pr_info("%s : error %d\n", __func__, ret);

		retry++;
	}

	return ret;
}

#ifdef CONFIG_W1_SN
static const int sn_cdigit[19] = {
	0x0e, 0x0d, 0x1f, 0x0b, 0x1c,
	0x12, 0x0f, 0x1e, 0x0a, 0x13,
	0x14, 0x15, 0x19, 0x16, 0x17,
	0x20, 0x1b, 0x1d, 0x11};

static bool w1_ds28e1x_check_digit(const uchar *sn)
{
	int i, tmp1 = 0, tmp2 = 0;
	int cdigit = sn[3];

	for (i=4;i<10;i++)
		tmp1 += sn[i];

	tmp1 += sn[4]*5;
	tmp2 = (tmp1 * sn[9] * sn[13]) % 19;

	tmp1 = (sn[10] + sn[12]) * 3 + (sn[11] + sn[13]) * 6 + 14;

	if (cdigit == sn_cdigit[((tmp1 + tmp2) % 19)])
		return true;
	else
		return false;
}

static uchar w1_ds28e1x_char_convert(uchar c)
{
	char ctable[36] = {
	'0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
	'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'J', 'K',
	'L', 'M', 'N', 'P', 'Q', 'R', 'S', 'T', 'V', 'W',
	'X', 'Y', 'Z', 'I', 'O', 'U'};

	return ctable[c];
}

static void w1_ds28e1x_slave_sn(const uchar *rdbuf)
{
	int i;
	u8 sn[15];

	sn[14] = 0;

	if (w1_ds28e1x_check_digit(&rdbuf[4])) {
		for (i = 0 ; i < 14 ; i++)
			sn[i] = w1_ds28e1x_char_convert(rdbuf[i+4]);

		for (i = 0 ; i < 14 ; i++)
			g_sn[i] = sn[13 - i];
	} else {
		pr_info("%s: sn is not good %s\n", __func__, sn);
	}
}
#endif


static int w1_ds28e1x_add_slave(struct w1_slave *sl)
{
	int err = 0;
    int ii;

	printk(KERN_ERR "\nw1_ds28e1x_add_slave start\n");

#ifdef CONFIG_OF_SUBCMDLINE_PARSE
	err = get_array_value();
	if (err) {
		printk(KERN_ERR "%s: w1_get_array_value error\n", __func__);
	}
#endif

	for (ii = 0; ii < ARRAY_SIZE(w1_ds28e1x_if_lists) && !err; ++ii)
    {   
		err = device_create_file(&sl->dev, &w1_ds28e1x_if_lists[ii]);
    	if (err) 
        {
    		device_remove_file(&sl->dev, &w1_ds28e1x_if_lists[ii]);
    		printk(KERN_ERR "%s: create device attribute file error\n", __func__);
            //!TBD: previous successfully created file cleanup can be removed also.
    		return err;
	    }
    }

	// copy rom id to use mac calculation
	memcpy(rom_no, (u8 *)&sl->reg_num, sizeof(sl->reg_num));

	if (init_verify) {
		if (skip_setup == 0) {
            #if 1
            //in some race condition ofdevice search, setup may fail. so gurantee setup using re-try.
            ii = 0;
            while (ii < RETRY_LIMIT) {
        		err = w1_ds28e1x_setup_device(sl);
        		//err = w1_ds28e1x_setup_for_production(sl);
        		if (err == 0)
        			break;

        		mdelay(10); /* wait 10ms */
        		ii++;
	        }
            #else
			err = w1_ds28e1x_setup_device(sl);
            #endif
			printk(KERN_ERR "w1_ds28e1x_setup_device result=%d\n", err);
            if(0 == err)
			    skip_setup = 1;
			err = w1_ds28e1x_verifymac(sl);
			verification = err;
		} else {
			err = w1_ds28e1x_verifymac(sl);
			verification = err;
			printk(KERN_ERR "w1_ds28e1x_verifymac\n");
		}
	}

	if(!verification)
	{
	    #if 0000//TBD
		pr_info("%s:uevent send 1\n", __func__);
		input_report_switch(sl->master->bus_master->input, SW_W1, 1);
		input_sync(sl->master->bus_master->input);
        #endif
	}

	printk(KERN_ERR "w1_ds28e1x_add_slave end, skip_setup=%d, err=%d\n", skip_setup, err);
	return err;
}

static void w1_ds28e1x_remove_slave(struct w1_slave *sl)
{
	int ii;
	for (ii = ARRAY_SIZE(w1_ds28e1x_if_lists) - 1; ii >= 0; --ii)
		device_remove_file(&sl->dev, &w1_ds28e1x_if_lists[ii]);

	verification = -1;
	printk(KERN_ERR "\nw1_ds28e1x_remove_slave\n");
}

static struct w1_family_ops w1_ds28e1x_fops = {
	.add_slave      = w1_ds28e1x_add_slave,
	.remove_slave   = w1_ds28e1x_remove_slave,
};

static struct w1_family w1_ds28e15_family = {
	.fid = W1_FAMILY_DS28E15,
	.fops = &w1_ds28e1x_fops,
};

static struct w1_family w1_ds28e11_family = {
	.fid = W1_FAMILY_DS28E11,
	.fops = &w1_ds28e1x_fops,
};

static int __init w1_ds28e1x_init(void)
{
	int ret = 0;
    //!E15 is only for debug in development stage.
    ret |= w1_register_family(&w1_ds28e15_family);
    ret |= w1_register_family(&w1_ds28e11_family);
    return ret;
}

static void __exit w1_ds28e1x_exit(void)
{
    //!E15 is only for debug in development stage.
	w1_unregister_family(&w1_ds28e15_family);
    w1_unregister_family(&w1_ds28e11_family);
}

module_init(w1_ds28e1x_init);
module_exit(w1_ds28e1x_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Clark Kim <clark.kim@maximintegrated.com>");
MODULE_DESCRIPTION("1-wire Driver for Maxim/Dallas DS23EL15 DeepCover Secure Authenticator IC");
