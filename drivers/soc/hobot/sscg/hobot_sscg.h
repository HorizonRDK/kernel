/*
+ * This program is free software; you can redistribute it and/or modify
+ * it under the terms of the GNU General Public License as published by
+ * the Free Software Foundation; either version 2 of the License, or
+ * (at your option) any later version.
+ *
+ * This program is distributed in the hope that it will be useful,
+ * but WITHOUT ANY WARRANTY; without even the implied warranty of
+ * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
+ * GNU General Public License for more details.
+ */

void switch_pll_source(int, int);
void do_ss(int, uint32_t, uint32_t, uint32_t);
ssize_t read_reg(volatile unsigned char *);
void write_reg(volatile unsigned char *, uint32_t);
void _do_ss(int, uint32_t, uint32_t, uint32_t);

