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
#include "elpkepuser.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>

static int do_job(
         uint32_t      opcode,  uint32_t options,
   const unsigned char *label,  uint32_t labellen,
   const unsigned char *master, uint32_t masterlen,
   const unsigned char *server, uint32_t serverlen,
   const unsigned char *client, uint32_t clientlen,
         unsigned char *out,    uint32_t outlen)
{
   int fd;
   unsigned char buf[1024], *p;
   uint32_t x;
   
   if ((7 + labellen + masterlen + serverlen + clientlen) > sizeof(buf)) {
      printf("Invalid buffer lengths\n");
      return -1;
   }
   
   if ((labellen|masterlen|serverlen|clientlen)&~0xFF) {
      printf("No buffer may be bigger than 255 bytes\n");
      return -1;
   }
   
   buf[0] = opcode;
   buf[1] = options;
   buf[2] = masterlen;
   buf[3] = serverlen;
   buf[4] = clientlen;
   buf[5] = labellen;
   buf[6] = outlen>>8;
   buf[7] = outlen&0xFF;
   
   p = &buf[8];
   if (master) { memcpy(p, master, masterlen); p += masterlen; }
   if (server) { memcpy(p, server, serverlen); p += serverlen; }
   if (client) { memcpy(p, client, clientlen); p += clientlen; }
   if (label)  { memcpy(p, label,  labellen);  p += labellen; }
   
   fd = open("/dev/spacckep", O_RDWR);
   if (fd < 0) {
      perror("Cannot open KEP device\n");
      return -1;
   }
   if (write(fd, buf, (p - buf)) != (p - buf)) {
      perror("Cannot write to KEP device\n");
      return -1;
   }
   if ((x = read(fd, out, outlen)) != outlen) {
      printf("Read %zu instead of %zu bytes from KEP\n", x, outlen);
      perror("Cannot read from KEP device\n");
      return -1;
   }
#if 0
{
   int x;
   printf("Read from device\n");
   for (x = 0; x < outlen; ) { printf("%02x", out[x]); if (!(++x & 15)) printf("\n"); }
}
#endif      
   
   close(fd);
   return 0;   
}


int kep_ssl3_keygen(const unsigned char *master_secret,
                    const unsigned char *server_secret,
                    const unsigned char *client_secret,
                          unsigned char *key, uint32_t keylen)
{
   return do_job(KEP_SSL3_KEYGEN, 0, NULL, 0, master_secret, 48, server_secret, 32, client_secret, 32, key, keylen);
}   

int kep_ssl3_sign(const unsigned char *sign_data, uint32_t sign_len, uint32_t options,
                  const unsigned char *master_secret,
                        unsigned char *dgst, uint32_t dgst_len)
{
   return do_job(KEP_SSL3_SIGN, options, NULL, 0, master_secret, 48, sign_data, sign_len, NULL, 0, dgst, dgst_len);
}                
                        
int kep_tls_prf(const unsigned char *label, uint32_t label_len, uint32_t options,
                const unsigned char *master_secret,
                const unsigned char *server_secret, uint32_t server_len,
                      unsigned char *prf, uint32_t prf_len)
{                      
   return do_job(KEP_TLS_PRF, options, label, label_len, master_secret, 48, server_secret, server_len, NULL, 0, prf, prf_len);
}   
  

int kep_tls_sign(const unsigned char *sign_data, uint32_t sign_len, uint32_t options,
                 const unsigned char *master_secret,
                       unsigned char *dgst, uint32_t dgst_len)
{
   return do_job(KEP_TLS_SIGN, options, NULL, 0, master_secret, 48, sign_data, sign_len, NULL, 0, dgst, dgst_len);
}                          


int kep_tls2_prf(const unsigned char *label, uint32_t label_len, uint32_t options,
                 const unsigned char *master_secret, uint32_t master_len,
                 const unsigned char *server_secret, uint32_t server_len,
                       unsigned char *prf, uint32_t prf_len)
{
   return do_job(KEP_TLS2_PRF, options, label, label_len, master_secret, master_len, server_secret, server_len, NULL, 0, prf, prf_len);
}                       

int kep_tls2_sign(const unsigned char *sign_data, uint32_t sign_len, uint32_t options,
                  const unsigned char *master_secret, uint32_t master_len,
                        unsigned char *dgst, uint32_t dgst_len)
{
   return do_job(KEP_TLS2_SIGN, options, NULL, 0, master_secret, master_len, sign_data, sign_len, NULL, 0, dgst, dgst_len);                        
}


