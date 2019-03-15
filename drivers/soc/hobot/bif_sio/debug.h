#ifndef _DEBUG_H_
#define _DEBUG_H_

#if defined _DEBUG_PRINTF_
#define tty_debug_log(format, ...) pr_err("bif sio: %s, %d "format, \
	__func__, __LINE__, ##__VA_ARGS__)

#define tty_err_log(format, ...) pr_err("bif sio: %s, %d "format, \
	__func__, __LINE__, ##__VA_ARGS__)

#define tty_dump_log(buf, len) _dbg_printhex(buf, len)
#else
#define tty_debug_log(...)
#define tty_err_log(...) pr_err(__VA_ARGS__) /* err log always open */
#define tty_dump_log(buf, len)
#endif

void _dbg_printhex(unsigned char *buf, int len);

#endif
