#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <sys/resource.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>

enum ecipher
{
  C_NULL   = 0,
  C_DES    = 1,
  C_AES    = 2,
  C_RC4    = 3,
  C_MULTI2 = 4,
  C_KASUMI = 5,
  C_SNOW3G_UEA2 = 6,
  C_ZUC_UEA3 = 7,

  C_MAX
};

enum eciphermode
{
  CM_ECB = 0,
  CM_CBC = 1,
  CM_CTR = 2,
  CM_CCM = 3,
  CM_GCM = 5,
  CM_OFB = 7,
  CM_CFB = 8,
  CM_F8  = 9,
  CM_XTS = 10,

  CM_MAX
};

enum ehash
{
  H_NULL   = 0,
  H_MD5    = 1,
  H_SHA1   = 2,
  H_SHA224 = 3,
  H_SHA256 = 4,
  H_SHA384 = 5,
  H_SHA512 = 6,
  H_XCBC   = 7,
  H_CMAC   = 8,
  H_KF9    = 9,
  H_SNOW3G_UIA2 = 10,
  H_CRC32_I3E802_3 = 11,
  H_ZUC_UIA3 = 12,
  H_SHA512_224 = 13,
  H_SHA512_256 = 14,
H_MAX
};

enum ehashmode
{
  HM_RAW    = 0,
  HM_SSLMAC = 1,
  HM_HMAC   = 2,

  HM_MAX
};

struct list { char *name; int val; };

struct list ciphers[] = {
     { "null", C_NULL},
     { "des",  C_DES},
     { "aes",  C_AES},
     { "rc4",  C_RC4},
     { "multi2", C_MULTI2},
     { "kasumi", C_KASUMI},
     { "snow3g", C_SNOW3G_UEA2},
     { "zuc", C_ZUC_UEA3 },
     { NULL, 0 },
};

struct list ciphermodes[] = {
   { "ecb",  CM_ECB },
   { "cbc",  CM_CBC },
   { "ctr",  CM_CTR },
   { "ccm",  CM_CCM },
   { "gcm",  CM_GCM },
   { "ofb",  CM_OFB },
   { "cfb",  CM_CFB },
   { "f8",   CM_F8  },
   { "xts",  CM_XTS },
   { NULL, 0 },
};

struct list hashes[] = {
   { "null",   H_NULL },
   { "md5",    H_MD5  },
   { "sha1",   H_SHA1 },
   { "sha224", H_SHA224 },
   { "sha256", H_SHA256 },
   { "sha384", H_SHA384 },
   { "sha512", H_SHA512 },
   { "xcbc",   H_XCBC },
   { "cmac",   H_CMAC },
   { "kf9",    H_KF9 },
   { "snow3g", H_SNOW3G_UIA2 },
   { "crc32",  H_CRC32_I3E802_3 },
   { "zuc",    H_ZUC_UIA3 },
   { "sha512_224", H_SHA512_224 },
   { "sha512_256",  H_SHA512_256 },
   { NULL, 0 }
};

struct list hashmodes[] = {
   { "raw", HM_RAW },
   { "sslmac", HM_SSLMAC },
   { "hmac", HM_HMAC },
   { NULL, 0 },
};

uint32_t find_list(struct list *foo, char *str)
{
   uint32_t x;
   for (x = 0; foo[x].name != NULL; x++) {
      if (!strcmp(foo[x].name, str)) {
         fprintf(stderr, "Picked %s\n", str);
         return foo[x].val;
      }
   }
   fprintf(stderr, "Invalid option [%s]...\n", str);
   return 0;
}

int main(int argc, char **argv)
{
   int x;
   uint32_t cipher, ciphermode, hash, hashmode, payloadsize, cipherksize, hashksize, mpmchainsize, outsize, useea, usempm, epn, runs;
   unsigned char buf[16], fname[64];
   struct rusage usage;
   double stime, utime, wtime, jobs_per_second, percent_cpu, jobs_per_percent, throughput, npackets, bits_processed;
   struct timeval t1, t2;
   FILE *fd;

   cipher = ciphermode = hash = hashmode = 0;
   payloadsize = 64;
   cipherksize = 16;
   hashksize   = 16;
   strcpy(fname, "/dev/spaccmpm");
   outsize     =  16;
   useea = usempm = 0;
   epn         = 0x5604;
   runs        = 65536;

   for (x = 1; x < argc; x++) {
      if (!strcmp(argv[x], "--size"))       { payloadsize = atoi(argv[x+1]); ++x; }
      if (!strcmp(argv[x], "--cipher"))     { cipher      = find_list(ciphers, argv[x+1]); ++x; }
      if (!strcmp(argv[x], "--ciphermode")) { ciphermode  = find_list(ciphermodes, argv[x+1]); ++x; }
      if (!strcmp(argv[x], "--ciphersize")) { cipherksize = atoi(argv[x+1]); ++x; }
      if (!strcmp(argv[x], "--hash"))       { hash        = find_list(hashes,  argv[x+1]); ++x; }
      if (!strcmp(argv[x], "--hashmode"))   { hashmode    = find_list(hashmodes,  argv[x+1]); ++x; }
      if (!strcmp(argv[x], "--hashsize"))   { hashksize   = atoi(argv[x+1]); ++x; }
      if (!strcmp(argv[x], "--device"))     { strcpy(fname, argv[x+1]); ++x; }
      if (!strcmp(argv[x], "--epn"))        { epn   = strtoul(argv[x+1], NULL, 16); ++x; }
      if (!strcmp(argv[x], "--runs"))       { runs  = strtoul(argv[x+1], NULL, 10); ++x; }

// MPM specific
      if (!strcmp(argv[x], "--mpmchainsize"))   { mpmchainsize = atoi(argv[x+1]); ++x; ++outsize; usempm = 1; }

// EA specific

   }

   buf[0] = cipher;
   buf[1] = ciphermode;
   buf[2] = cipherksize;
   buf[3] = hash;
   buf[4] = hashmode;
   buf[5] = hashksize;
   buf[6] = (payloadsize >> 8);
   buf[7] = payloadsize & 0xFF;
   buf[8] = (epn>>8);
   buf[9] = epn&0xFF;
   buf[10] = (runs>>16)&0xFF;
   buf[11] = (runs>>8)&0xFF;
   buf[12] = runs&0xFF;

   if (usempm) {
      buf[13] = mpmchainsize;
   }

   gettimeofday(&t1, NULL);
   fd = fopen(fname, "wb");
   fwrite(buf, outsize, 1, fd);
   fclose(fd);
   getrusage(RUSAGE_SELF, &usage);
   gettimeofday(&t2, NULL);
   npackets         = runs;
   bits_processed   = npackets * 8.0 * payloadsize;
   wtime            = ((t2.tv_sec*1000000+t2.tv_usec) - (t1.tv_sec*1000000+t1.tv_usec))/1000000.0;  // times are in seconds
   stime            = (usage.ru_stime.tv_sec*1000000+usage.ru_stime.tv_usec)/1000000.0;
   utime            = (usage.ru_utime.tv_sec*1000000+usage.ru_utime.tv_usec)/1000000.0;
   jobs_per_second  = npackets / wtime;
   percent_cpu      = ((stime + utime)/wtime)*100.0;
   jobs_per_percent = jobs_per_second / percent_cpu;
   throughput       = bits_processed / (wtime * 1000000.0);
   printf("%3.5f,%3.5f,%3.5f,%2.3f,%4.2f,%.2f,%.2f\n",
      wtime,             // wall time
      utime,             // user time
      stime,             // sys time
      percent_cpu,       // % of wall time
      throughput,        // bits per microsecond (megabit/sec)
      jobs_per_second,   // jobs per second
      jobs_per_percent   // jobs per second per 1% of CPU
   );

   return 0;
};






