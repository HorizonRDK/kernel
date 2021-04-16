/*
 * 1-Wire SHA256 software implementation for the ds23el15 chip
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

#ifndef _W1_DS28E15_SHA256_H
#define _W1_DS28E15_SHA256_H

#ifndef ushort
typedef unsigned short ushort;
#endif

#ifndef ulong
typedef unsigned long ulong;
#endif

#ifndef uchar
typedef unsigned char uchar;
#endif

// SHA-256 Functions
int compute_sha256(uchar *message, short length, ushort skipconst,
		   ushort reverse, uchar *digest);
int compute_mac256(uchar *message, short length, uchar *MAC);
int verify_mac256(uchar *message, short length, uchar *compare_MAC);
int calculate_nextsecret256(uchar *binding, uchar *partial, int page_num,
			    uchar *manid);
void set_secret(const uchar *secret_data);
void set_romid(uchar *romid_data);

// Utility Functions
ulong sha_ch(ulong x, ulong y, ulong z);
ulong sha_maj(ulong x, ulong y, ulong z);
ulong sha_rotr_32(ulong val, ushort r);
ulong sha_shr_32(ulong val, ushort r);
ulong sha_bigsigma256_0(ulong x);
ulong sha_littlesigma256_0(ulong x);
ulong sha_littlesigma256_1(ulong x);
void sha_copy32(ulong *p1, ulong *p2, ushort length);
void sha_copyWordsToBytes32(ulong *input, uchar *output, ushort numwords);
void sha_writeResult(ushort reverse, uchar *outpointer);
ulong sha_getW(int index);
void sha_prepareSchedule(uchar *message);
void sha256_hashblock(uchar *message, ushort lastblock);

#endif
