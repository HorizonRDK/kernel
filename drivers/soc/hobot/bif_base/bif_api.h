
#ifndef _BIF_API_H_
#define _BIF_API_H_

#define BIFOK		(0)
#define BIFERR		(1)
#define BIFNOSD		(2)
#define BIFNOSPI	(3)
#define STR_SUPPORT_NO	"no"
#define STR_SUPPORT_YES	"yes"
#define STR_BIFBUS_NO	"bifno"
#define STR_BIFBUS_SPI	"bifspi"
#define STR_BIFBUS_SD	"bifsd"

#define BIFSD_BLOCK		(512)
#define BIFSPI_BLOCK	(16)
#define PLATFORM_SIZE	(16)

#define MULTI(a, b)	(((a)%(b)) > 0 ? ((a)/(b)+1)*(b) : (a))

enum PLAT_TYPE {
	PLAT_CP = 0,
	PLAT_AP,
	PLAT_MAX,
};

enum PARAM_TYPE {
	PARAM_DTS = 0,
	PARAM_MODULE,
	PARAM_MAX,
};

enum SUPPORT_BUS {
	SUPPORT_NO = 0,
	SUPPORT_YES,
	SUPPORT_MAX,
};

enum BIFBUS_TYPE {
	BIFBUS_NO = 0,
	BIFBUS_SPI,
	BIFBUS_SD,
	BIFBUS_MAX,
};

struct bifplat_info {
	char platform[PLATFORM_SIZE];
	uint kernel_ver;
	enum PLAT_TYPE plat_type;
	enum PARAM_TYPE param;
	enum SUPPORT_BUS bifspi;
	enum SUPPORT_BUS bifsd;
	ulong bifbase_phyaddr;
	uint bifbase_phyaddrsize;
	int irq_pin_absent;
	int irq_pin;
	int irq_num;
	int tri_pin;
	int tri_val;
};

/*
 *AP side export symbol functions
 *@addr: phy addr
 *@count: bytes, must be multi of 512, eg. 512,1024,1536...
 *@return: 0, ok; other ,err
 *int bifsd_read(void *addr, unsigned int count, unsigned char *buf);
 *int bifsd_write(void *addr, unsigned int count, unsigned char *buf);
 */
int bif_sd_read(void *addr, unsigned int count, unsigned char *buf);
int bif_sd_write(void *addr, unsigned int count, unsigned char *buf);
/*
 *@addr: phy addr
 *@count: bytes, must be multi of 16
 *@return: 0, ok; other ,err
 *int bifspi_read(void *addr, unsigned int count, unsigned char *buf);
 *int bifspi_write(void *addr, unsigned int count, unsigned char *buf);
 */
int bif_spi_read(void *addr, unsigned int count, unsigned char *buf);
int bif_spi_write(void *addr, unsigned int count, unsigned char *buf);
/*
 *@channle: BIFBUS_NO, BIFBUS_SPI, BIFBUS_SD,BIFBUS_MAX
 *@addr: phy addr
 *@count: bytes, must be multi of 16
 *@return: 0, ok; other ,err
 *int bifspi_read(void *addr, unsigned int count, unsigned char *buf);
 *int bifspi_write(void *addr, unsigned int count, unsigned char *buf);
 */
int bifread(int channel, void *addr, uint count, unchar *buf);
int bifwrite(int channel, void *addr, uint count, unchar *buf);
#endif
