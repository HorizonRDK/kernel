/*
 * Horizon Robotics
 *
 *  Copyright (C) 2020 Horizon Robotics Inc.
 *  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef NETLINK_DIAG_H
#define NETLINK_DIAG_H
#include <linux/list.h>
#include <linux/spinlock.h>

#define DIAG_MSG_HEAD 0xAA	// diag msg identifier.
#define DIAG_MSG_VER  0x01	// What version of the dig msg
#define FRAGMENT_SIZE 0x4000// netlink msg fragment max size, 0x4000 = 16kb, so we can use kmalloc(...)
#define DIAG_USE_NETLINK_BROADCAST
#define USER_GROUP 1

/*
 *  You can't exceed this number(bytes),if you want
 *  to send more, please send in other context,
 *   not irq contex.
 */
#define DIAG_SEND_LEN_MAX_IN_IRQ 50

/* Interval between consecutive transmissions of same id */
#define DIAG_ID_SEND_INTERVAL_MS 200

/* frame type */
#define DIAG_MSG_TYPE_REPORT_EVENT_STAT	1
#define DIAG_MSG_TYPE_SEND_ENV_DATA	2
#define DIAG_MSG_TYPE_READ_ENV_DATA	3

/*
 * module id
 */
enum diag_module_id {
	ModuleDiagDriver = 1,
	ModuleDiag_i2c,
	ModuleDiag_VIO,
	ModuleDiag_bpu,
	ModuleDiag_sound,
	ModuleDiag_bif,
	ModuleDiag_eth,
	ModuleDiag_spi,
	ModuleDiag_emmc,
	ModuleDiag_qspi,
	ModuleIdMax = 1000,
};

/*
 * event priority
 */
enum diag_msg_prio {
	DiagMsgPrioHigh = 1,
	DiagMsgPrioMid,
	DiagMsgPrioLow,
	DiagMsgPrioMax,
};

/*
 * event status.
 */
enum diag_event_sta {
	DiagEventStaUnknown = 1,
	DiagEventStaSuccess,
	DiagEventStaFail,
	DiagEventStaMax,
};

/*
 * env data generation timing.
 */
enum diag_gen_envdata_timing {
	DiagGenEnvdataWhenErr = 1,
	DiagGenEnvdataLastErr,
	DiagGenEnvdataWhenSuccess,
	DiagGenEnvdataMax,
};

/*
 * Define the event id for your own module below,
 * note:each item can not greater than EVENT_ID_MAX
 * and start form 1, end with EVENT_ID_MAX - 1
 */
#define EVENT_ID_MAX 100
/* diag driver module event id */
enum diag_driver_module_eventid {
	EventIdKernelToUserSelfTest = 1,
};

/* i2c module event id */
enum diag_i2c_driver_module_eventid {
	EventIdI2cController0Err = 1,
	EventIdI2cController1Err,
	EventIdI2cController2Err,
	EventIdI2cController3Err,
	EventIdI2cController4Err,
	EventIdI2cController5Err,
};

/* VIO module event id */
enum diag_vio_module_eventid {
	EventIdVioMipiHost0Err = 1,
	EventIdVioMipiHost1Err,
	EventIdVioMipiHost2Err,
	EventIdVioMipiHost3Err,
	EventIdVioMipiDevErr,
	EventIdVioSifErr,
	EventIdVioIpuErr,
	EventIdVioIspDropErr,
};

/* bpu module event id */
enum diag_bpu_module_eventid {
	EventIdBpu0Err = 1,
	EventIdBpu1Err = 2,
};

/* sound module event id */
enum diag_sound_module_eventid {
	EventIdSoundI2s0Err = 1,
	EventIdSoundI2s1Err,
};

/* bif module event id */
enum diag_bif_module_eventid {
	EventIdBifSpiErr = 1,
	EventIdBifSdErr,
	//EventIdBifEthernetErr,
	//EventIdBifSioErr,
};

/* eth module event id */
enum diag_eth_module_eventid {
	EventIdEthDmaBusErr = 1,
};

/* qspi module event id */
enum diag_qspi_module_eventid {
	EventIdqspiErr = 1,
};

/* spi module event id */
enum diag_spi_module_eventid {
	EventIdSpi0Err = 1,
	EventIdSpi1Err,
	EventIdSpi2Err,
	EventIdQspiErr,
};

/* emmc module event id */
enum diag_emmc_module_eventid {
	EventIdEmmcErr = 1,
};

#define DIAG_UNMASK_ID_MAX_NUM 100
struct diag_msg_id_unmask_struct {
	uint16_t module_id;
	uint16_t event_id;
	struct list_head mask_lst;
};

/* msg id: It consists of three parts.*/
struct diag_msg_id {
	uint16_t module_id;	// module id.
	uint16_t event_id;	// event id.
	uint8_t  msg_pri;	// msg priroty.

};

/*
 * diag msg packt header format
 */
struct diag_msg_hdr {
	uint8_t		packt_ident;
	uint8_t		version;	// diag msg format version.
	uint8_t		frame_type; // status pack or env pack.
	uint8_t		start;		// Is diag msg start fragment.
	uint8_t		end;		// Is diag msg end fragment.
	uint32_t	seq;		// diag msg fragment sequence.
	uint32_t	len;		// payload data len.
} __packed;

/*
 * diag msg packt format
 */
struct diag_msg {
	struct diag_msg_hdr head;
	uint8_t *data;		// payload

	/* From the beginning of the packet
	 * to the end of the data.
	 */
	uint32_t checksum;

} __packed;

/*
 * event id and it's sta package,put
 * this package in the payload of diag msg.
 */
struct report_event_sta_pack {
	struct diag_msg_id id;
	uint8_t event_sta;
} __packed;

/*
 * env data head
 */
struct env_data_head {
	struct report_event_sta_pack pack_info;
	uint8_t env_data_gen_timing;

} __packed;

/*
 * env data structure
 */
struct env_data_pack {
	struct env_data_head head;
	uint8_t *data;

} __packed;

#define DIAG_LIST_MAX_DEFAULT 128

struct id_info {
	struct diag_msg_id id;
	uint8_t event_sta;
	uint8_t env_data_gen_timing;
	size_t envlen;
	struct list_head idlst;
	unsigned long record_jiffies;
	uint8_t *pdata;
};

#define ENVDATA_BUFFER_NUM 3
#define ENVDATA_MAX_SIZE 0x500000
struct envdata_buffer_manage {
	spinlock_t env_data_buffer_lock;
	uint8_t flags;
	uint8_t *pdata;
};

struct id_register_struct {
	struct diag_msg_id id;
	size_t envdata_max_size;
	uint32_t min_snd_ms;
	uint8_t register_had_env_flag;
	uint8_t last_sta;
	uint8_t current_sta;
	unsigned long last_snd_time_ms;
	unsigned long max_time_out_snd_ms;
	spinlock_t stat_change_lock;
	void (*msg_rcvcallback)(void *p, size_t len);
	struct envdata_buffer_manage envdata_buff[ENVDATA_BUFFER_NUM];
	struct list_head id_register_lst;
};

extern struct net init_net;
extern struct list_head diag_id_unmask_list;
extern uint32_t diag_id_unmask_list_num;
//extern spinlock_t diag_id_unmask_list_spinlock;

struct id_register_struct *diag_id_in_register_list(struct diag_msg_id *id);
/*
 * send event sta and it's env data to the diag app.
 * @id module id, event id, msg priority
 * @event_sta event sta
 * @env_data_gen_timing When is the environmental data generated.
 * @env_data env data ptr
 * @env_len env data len.
 * @return -1:error, -2: diag not ready, >=0:OK
 */
extern int diag_send_event_stat_and_env_data(
			uint8_t msg_prio,
			uint16_t module_id,
			uint16_t event_id,
			uint8_t event_sta,
			uint8_t env_data_gen_timing,
			uint8_t *env_data,
			size_t env_len);

extern int diag_send_event_stat(
		uint8_t msg_prio,
		uint16_t module_id,
		uint16_t event_id,
		uint8_t event_sta);

/*
 * diag register func, you must register you module diag before call send diag msg API.
 * @module_id  module id, you will send msg.
 * @event_id event id
 * @envdata_max_size how many env data, you will send[max size].
 * @min_snd_ms minimum interval between messages sent.
 * @max_time_out_snd maximum timeout
 * @rcvcallback this module--event maybe rcv a msg form diag app[call back]
 */
extern int diag_register(uint16_t module_id, uint16_t event_id, size_t envdata_max_size,
		uint32_t min_snd_ms, uint32_t max_time_out_snd,
		void (*rcvcallback)(void *p, size_t len));

#endif
