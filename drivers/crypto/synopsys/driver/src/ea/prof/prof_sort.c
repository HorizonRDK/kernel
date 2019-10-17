// this application sorts data from /dev/xfrm_stats into in-order inbound/outbound files
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

uint32_t records[65536];
uint32_t sorted[65536];
uint32_t hist[2][65536];
uint32_t tot[2];

int main (void)
{
   FILE *f;
   uint32_t x, y;

   memset(hist, 0, sizeof hist);
   memset(tot, 0, sizeof tot);

   f = fopen ("/dev/xfrm_stats", "rb");
   fread (records, 1, sizeof records, f);
   fclose (f);

   // sort inbound first ...
   if ((records[0] ^ records[65535]) & 0x7FFF0000UL) {
      // wrap-around occurred so lets find out where
      for (x = 1; x < 65536; x++) {
         if ((records[0] ^ records[x]) & 0x7FFF0000UL) {
            break;
         }
      }
      // from 0...x-1 should be placed after 65536-x logically

      // copy first records
      memcpy (sorted, &records[x], sizeof (sorted) - (x * sizeof (sorted[0])));

      // copy trailer
      memcpy (&sorted[65536 - x], &records[0], x * sizeof (sorted[0]));
   } else {
      // array is sorted, just copy it over
      memcpy (sorted, records, sizeof sorted);
   }

   // sorted, now skip empty records
   for (x = 65535; x && !sorted[x]; x--); // seek backwards until non-zero
   f = fopen ("xfrm.dat", "wb");
   fwrite (sorted, 1, x * sizeof (sorted[0]), f);
   fclose (f);

   // emit as csv as well
   f = fopen("xfrm.csv", "w");
   fprintf(f, "direction,packet size\n");
   for (y = 0; y < x; y++) {
       fprintf(f,"%d,%zu\n", sorted[y] & 0x80000000 ? 1 : 0, sorted[y] & 0xFFFF);
       ++hist[sorted[y] & 0x80000000 ? 1 : 0][sorted[y] & 0xFFFF];
       ++tot[sorted[y] & 0x80000000 ? 1 : 0];
   }
   fclose(f);

   // compute histogram
   f = fopen("xfrm_hist.csv", "w");
   fprintf(f, "packet size,occurrences,%%\n");
   for (y = 0; y < 65536; y++) {
        if (hist[0][y] + hist[1][y]) {
           fprintf(f, "%zu,%zu,=%zu/%zu\n", y, hist[0][y] + hist[1][y], hist[0][y] + hist[1][y], x);
       }
   }
   fclose(f);
   f = fopen("xfrm_hist_0.csv", "w");
   fprintf(f, "packet size,occurrences,%%\n");
   for (y = 0; y < 65536; y++) {
        if (hist[0][y]) {
           fprintf(f, "%zu,%zu,=%zu/%zu\n", y, hist[0][y], hist[0][y], tot[0]);
       }
   }
   fclose(f);
   f = fopen("xfrm_hist_1.csv", "w");
   fprintf(f, "packet size,occurrences,%%\n");
   for (y = 0; y < 65536; y++) {
        if (hist[1][y]) {
           fprintf(f, "%zu,%zu,=%zu/%zu\n", y, hist[1][y], hist[1][y], tot[1]);
       }
   }
   fclose(f);


   return 0;
}
