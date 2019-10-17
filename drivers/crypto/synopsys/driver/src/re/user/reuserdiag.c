/*
 * This Synopsys software and associated documentation (hereinafter the
 * "Software") is an unsupported proprietary work of Synopsys, Inc. unless
 * otherwise expressly agreed to in writing between Synopsys and you. The
 * Software IS NOT an item of Licensed Software or a Licensed Product under
 * any End User Software License Agreement or Agreement for Licensed Products
 * with Synopsys or any supplement thereto. Synopsys is a registered trademark
 * of Synopsys, Inc. Other names included in the SOFTWARE may be the
 * trademarks of their respective owners.
 *
 * The contents of this file are dual-licensed; you may select either version
 * 2 of the GNU General Public License ("GPL") or the BSD-3-Clause license
 * ("BSD-3-Clause"). The GPL is included in the COPYING file accompanying the
 * SOFTWARE. The BSD License is copied below.
 *
 * BSD-3-Clause License:
 * Copyright (c) 2015 Synopsys, Inc. and/or its affiliates.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions, and the following disclaimer, without
 *    modification.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. The names of the above-listed copyright holders may not be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>
#include "elpreuser.h"

typedef struct  {
    uint32_t length[2];
    uint32_t state[4], curlen;
    unsigned char buf[64];
} md5_state;

static const uint32_t md5vals[64] = {
   0xd76aa478, 0xe8c7b756, 0x242070db, 0xc1bdceee, 0xf57c0faf, 0x4787c62a, 0xa8304613, 0xfd469501,
   0x698098d8, 0x8b44f7af, 0xffff5bb1, 0x895cd7be, 0x6b901122, 0xfd987193, 0xa679438e, 0x49b40821,
   0xf61e2562, 0xc040b340, 0x265e5a51, 0xe9b6c7aa, 0xd62f105d, 0x02441453, 0xd8a1e681, 0xe7d3fbc8,
   0x21e1cde6, 0xc33707d6, 0xf4d50d87, 0x455a14ed, 0xa9e3e905, 0xfcefa3f8, 0x676f02d9, 0x8d2a4c8a,
   0xfffa3942, 0x8771f681, 0x6d9d6122, 0xfde5380c, 0xa4beea44, 0x4bdecfa9, 0xf6bb4b60, 0xbebfbc70,
   0x289b7ec6, 0xeaa127fa, 0xd4ef3085, 0x04881d05, 0xd9d4d039, 0xe6db99e5, 0x1fa27cf8, 0xc4ac5665,
   0xf4292244, 0x432aff97, 0xab9423a7, 0xfc93a039, 0x655b59c3, 0x8f0ccc92, 0xffeff47d, 0x85845dd1,
   0x6fa87e4f, 0xfe2ce6e0, 0xa3014314, 0x4e0811a1, 0xf7537e82, 0xbd3af235, 0x2ad7d2bb, 0xeb86d391,
};

static const unsigned char rots[4][4] = {
    { 7,12,17,22 },
    { 5,9,14,20  },
    { 4,11,16,23 },
    { 6,10,15,21 }
};

static const unsigned char words[4][16] = {
    {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15 },
    {1,6,11,0,5,10,15,4,9,14,3,8,13,2,7,12 },
    {5,8,11,14,1,4,7,10,13,0,3,6,9,12,15,2 },
    {0,7,14,5,12,3,10,1,8,15,6,13,4,11,2,9 }
};

#define F(x,y,z)  (z ^ (x & (y ^ z)))
#define G(x,y,z)  (y ^ (z & (y ^ x)))
#define H(x,y,z)  (x^y^z)
#define I(x,y,z)  (y^(x|(~z)))

#define ROL(x, y) ( (((uint32_t)(x)<<(uint32_t)((y)&31)) | (((uint32_t)(x)&0xFFFFFFFFUL)>>(uint32_t)(32-((y)&31)))) & 0xFFFFFFFFUL)

#define STORE32L(x, y)                                                                     \
     { (y)[3] = (unsigned char)(((x)>>24)&255); (y)[2] = (unsigned char)(((x)>>16)&255);   \
       (y)[1] = (unsigned char)(((x)>>8)&255); (y)[0] = (unsigned char)((x)&255); }

#define LOAD32L(x, y)                            \
     { x = ((uint32_t)((y)[3] & 255)<<24) | \
           ((uint32_t)((y)[2] & 255)<<16) | \
           ((uint32_t)((y)[1] & 255)<<8)  | \
           ((uint32_t)((y)[0] & 255)); }

#ifndef MIN
   #define MIN(x, y) ( ((x)<(y))?(x):(y) )
#endif

/* this version is for 32-bit only platforms */
#define HASH_PROCESS32(func_name, compress_name, block_size)                     \
int func_name (const unsigned char *in, unsigned long inlen, md5_state *md)             \
{                                                                                           \
    unsigned long n;                                                                        \
    int           err;                                                                      \
    uint32_t       tmp;                                                                      \
    if (in == NULL || md == NULL) {                                                         \
       return -1;                                                                      \
    }                                                                                       \
    if (md->curlen > sizeof(md->buf)) {                 \
       return -1;                                                                      \
    }                                                                                       \
    while (inlen > 0) {                                                                     \
        if (md->curlen == 0 && inlen >= block_size) {                     \
           if ((err = compress_name (md, (unsigned char *)in)) != 0) {                 \
              return err;                                                                   \
           }                                                                                \
           tmp = md->length[0];                                           \
           md->length[0] += block_size * 8;                               \
           if (md->length[0] < tmp) { md->length[1] += 1; } \
           in             += block_size;                                                    \
           inlen          -= block_size;                                                    \
        } else {                                                                            \
           n = MIN(inlen, (block_size - md->curlen));                     \
           memcpy(md->buf + md->curlen, in, (size_t)n); \
           md->curlen += n;                                               \
           in             += n;                                                             \
           inlen          -= n;                                                             \
           if (md->curlen == block_size) {                                \
              if ((err = compress_name (md, md->buf)) != 0) {        \
                 return err;                                                                \
              }                                                                             \
              tmp = md->length[0];                                        \
              md->length[0] += block_size * 8;                            \
              if (md->length[0] < tmp) { md->length[1] += 1; } \
              md->curlen = 0;                                             \
           }                                                                                \
       }                                                                                    \
    }                                                                                       \
    return 0;                                                                          \
}

static md5_compress(md5_state *md, unsigned char *buf)
{
    uint32_t i, W[16], a, b, c, d;
    uint32_t t, xx;

    /* copy the state into 512-bits into W[0..15] */
    for (i = 0; i < 16; i++) {
        LOAD32L(W[i], buf + (4*i));
    }

    /* copy state */
    a = md->state[0];
    b = md->state[1];
    c = md->state[2];
    d = md->state[3];

    for (i = 0; i < 64; i++) {
        xx = i>>4;
        a += W[words[xx][i&15]] + md5vals[i];
        switch (xx) {
            case 0: a += F(b,c,d); break;
            case 1: a += G(b,c,d); break;
            case 2: a += H(b,c,d); break;
            case 3: a += I(b,c,d); break;
        }
        a = ROL(a, rots[xx][i&3]) + b;
        t = d; d = c; c = b; b = a; a = t;
    }
    md->state[0] = md->state[0] + a;
    md->state[1] = md->state[1] + b;
    md->state[2] = md->state[2] + c;
    md->state[3] = md->state[3] + d;

    return 0;
}

HASH_PROCESS32(md5_process, md5_compress, 64)

static int md5_init(int flags, md5_state * md)
{
   if (md == NULL) {
      return -1;
   }
   md->state[0] = 0x67452301UL;
   md->state[1] = 0xefcdab89UL;
   md->state[2] = 0x98badcfeUL;
   md->state[3] = 0x10325476UL;
   md->curlen = 0;
   md->length[0] = 0;
   md->length[1] = 0;
   return 0;
}

static int md5_done(unsigned char *out, unsigned long *outlen, md5_state *md)
{
    int i;
    uint32_t tmp;

    if (md->curlen >= sizeof(md->buf)) {
       return -1;
    }

    if (*outlen < 16) {
       *outlen = 16;
       return -1;
    }
    *outlen = 16;

    /* increase the length of the message */
    tmp = md->length[0];
    md->length[0] += md->curlen * 8;
    if (md->length[0] < tmp) { md->length[1] += 1; }

    /* append the '1' bit */
    md->buf[md->curlen++] = (unsigned char)0x80;

    /* if the length is currently above 56 bytes we append zeros
     * then compress.  Then we can fall back to padding zeros and length
     * encoding like normal.
     */
    if (md->curlen > 56) {
        while (md->curlen < 64) {
            md->buf[md->curlen++] = (unsigned char)0;
        }
        md5_compress(md, md->buf);
        md->curlen = 0;
    }

    /* pad upto 56 bytes of zeroes */
    while (md->curlen < 56) {
        md->buf[md->curlen++] = (unsigned char)0;
    }

    /* store length */
    STORE32L(md->length[0], md->buf+56);
    STORE32L(md->length[1], md->buf+60);
    md5_compress(md, md->buf);

    /* copy output */
    for (i = 0; i < 4; i++) {
        STORE32L(md->state[i], out+(4*i));
    }
    return 0;
}

static void md5_buf(const unsigned char *in, uint32_t len, unsigned char *out)
{
   unsigned long outlen = 16;
   md5_state md;

   md5_init(0, &md);
   md5_process(in, len, &md);
   md5_done(out, &outlen, &md);
}

static int actions=0, noverify=0;

static uint32_t getnum(int vfd, int b)
{
   uint32_t t;
   unsigned char buf[1];
   int x;

   t = 0;
   while (b--) {
      x = read(vfd, buf, 1);
      if (x != 1) {
         printf("%d actions...PASSED\n", actions);
         exit(0);
      }
      t = (t << 8) | buf[0];
   }
   return t;
}

static struct elp_spacc_re_usr fd, fd2, fd3, *fds[2];
static int fd_cnt=0;

static int work_vector(int vfd)
{
   unsigned char in[RE_MAX_SIZE], out[RE_MAX_SIZE], out2[RE_MAX_SIZE],
                 md[16], md2[16],
                 key[256],
                 iv[256],
                 hmackey[256],
                 cipher_params[256],
                 sequence_number[8],
                 error_msg[255];

   unsigned int action,fail_code;
   int id, errnum, errnum2, cmd, ver;

   int inlen, outlen;

   uint32_t in_packet_length,
            keylength, ivlength, mackeylength,
            position,
            filesize,
            outctlength, count;

   filesize = getnum(vfd, 4);
   action   = getnum(vfd, 1);

   ++actions;

   if (action == 0) {
      // write a CCS record out
      in[0] = 1;
      outlen = re_dev_do_record(&fd, in, 1, out, sizeof out, 0, 0, RE_DEV_CCS);
      if (outlen < 0) {
         perror("Error performing CCS:");
         return -1;
      }

      // we receive the CCS in the other direction
      inlen = outlen;
      outlen = re_dev_do_record(&fd3, out, inlen, out, sizeof out, 0, 0, RE_DEV_READ_MODE);
      if (outlen < 0) {
         perror("Error performing CCS:");
         return -1;
      }
      return 0;
   } else if (action == 1 || action == 2 || action == 3 || action == 4) {
      // alert, handshake, data
      switch (action) {
         case 1: cmd = RE_DEV_ALERT; break;
         case 2: cmd = RE_DEV_HANDSHAKE; break;
         case 3: cmd = RE_DEV_WRITE_MODE; break;
         case 4: cmd = RE_DEV_READ_MODE; break;
      }
      if (action == 4) {
         // decrypt vectors have a fail type flag
         fail_code     = getnum(vfd, 1);
      }
      in_packet_length = getnum(vfd, 4);

      if (action == 4) {
         // read the plaintext md5sum
         read(vfd, md, sizeof md);
      }
      read(vfd, in, in_packet_length);

      outlen = re_dev_do_record(fds[fd_cnt ^= 1], in, in_packet_length, out, sizeof out, 0, 0, cmd);

      if (action == 4 && fail_code) {
         if (RE_USR_ERR(fds[fd_cnt]) == fail_code) {
            // we're good only if this is a decrypt packet *AND* the failure type matches
            printf("Correctly returned error code: %u ... ", fail_code);
            return 0;
         } else {
            printf("FAIL: Incorrectly missed error ... err=%d should be %d ...", RE_USR_ERR(fds[fd_cnt]), fail_code);
            return -1;
         }
      }

      if (outlen < 0) {
         printf("Error processing record, err=%d...", RE_USR_ERR(fds[fd_cnt]));
         return -1;
      }

      if (action == 4) {
         md5_buf(out, outlen, md2);
         if (memcmp(md, md2, 16)) {
            printf("Error comparing md5sum of payload...");
            return -1;
         }
      }

      if (cmd == RE_DEV_WRITE_MODE) {
         // lets try to decrypt the packet and compare
         int outlen2;
         outlen2 = re_dev_do_record(&fd3, out, outlen, out2, sizeof out2, 0, 0, RE_DEV_READ_MODE);
         if (outlen2 < 0) {
            printf("Error verifying record, err=%d...", RE_USR_ERR(fds[fd_cnt]));
            return -1;
         }
         if (outlen2 != in_packet_length) {
            printf("Packet lengths don't match expected %d got %d(%d)...", in_packet_length, outlen2, outlen);
            return -1;
         }
         if (memcmp(out2, in, in_packet_length)) {
            printf("Packet contents don't match(%d)...", noverify);
            return -1;
         }
         ++noverify;
      } else {
      }
      return 0;
   } else if (action == 255 || action == 254) {
      // next output cipher suite
      // next input cipher suite

      read(vfd, cipher_params, 2);
      keylength    = getnum(vfd, 1); read(vfd, key,     keylength);
      ivlength     = getnum(vfd, 1); read(vfd, iv,      ivlength);
      mackeylength = getnum(vfd, 1); read(vfd, hmackey, mackeylength);
      read(vfd, sequence_number, 8);

      // we initialize fd/fd3 in mirror fashion so they can talk with one another
      if (action == 255) {
         errnum  = re_dev_set_write_context(&fd, iv, ivlength, key, keylength, hmackey, mackeylength, cipher_params, 2, sequence_number, 8);
         errnum2 = re_dev_set_read_context(&fd3, iv, ivlength, key, keylength, hmackey, mackeylength, cipher_params, 2, sequence_number, 8);
      } else {
         errnum  = re_dev_set_read_context(&fd, iv, ivlength, key, keylength, hmackey, mackeylength, cipher_params, 2, sequence_number, 8);
         errnum2 = re_dev_set_write_context(&fd3, iv, ivlength, key, keylength, hmackey, mackeylength, cipher_params, 2, sequence_number, 8);
      }
      if (errnum < 0) {
         printf("Error writing context: %d\n", RE_USR_ERR(&fd));
         return -1;
      }
      if (errnum2 < 0) {
         printf("Error writing 2nd context: %d\n", RE_USR_ERR(&fd3));
         return -1;
      }
      return 0;
   } else if (action == 253) {
      // used to retrieve a new context
      if (fd.fd != -1) {
         re_dev_close(&fd);
         re_dev_close(&fd2);
         re_dev_close(&fd3);
         fd.fd = -1;
      }

      errnum = re_dev_open(&fd, ver=getnum(vfd, 1));
      if (errnum < 0) {
         perror("Cannot open RE device");
         return EXIT_FAILURE;
      }
      errnum = re_dev_open(&fd3, ver);
      if (errnum < 0) {
         perror("Cannot open RE device");
         return EXIT_FAILURE;
      }

      errnum = re_dev_register(&fd);
      if (errnum < 0) {
         perror("Cannot register RE device");
         return EXIT_FAILURE;
      }
      errnum = re_dev_open_bind(&fd, &fd2);
      if (errnum < 0) {
         perror("Cannot open_bind RE device");
         return EXIT_FAILURE;
      }
      return 0;
   }
   printf("invalid action: %d\n", action);
   exit(EXIT_FAILURE);
}

int main(int argc, char **argv)
{
   int vfd;

   fd.fd = -1;
   fds[0] = &fd;
   fds[1] = &fd2;

   printf("Running: %s...", argv[1]);
   vfd = open(argv[1], O_RDONLY);
   if (vfd < 0) {
      perror("Cannot open vector:");
      return -1;
   }

   for (;;) {
      if (work_vector(vfd)) {
         printf("%d actions... FAILED\n", actions);
         return EXIT_FAILURE;
      }
   }

   return EXIT_SUCCESS;
}
