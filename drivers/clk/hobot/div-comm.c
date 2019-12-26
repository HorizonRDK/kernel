#include "div-comm.h"

unsigned int _get_div(const struct clk_div_table *table,
				unsigned int val, unsigned long flags, u8 width)
{
	if (flags & CLK_DIVIDER_ONE_BASED)
		return val;
	if (flags & CLK_DIVIDER_POWER_OF_TWO)
		return 1 << val;
	if (flags & CLK_DIVIDER_MAX_AT_ZERO)
		return val ? val : div_mask(width) + 1;
	if (table)
		return val;
	return val + 1;
}
EXPORT_SYMBOL(_get_div);
