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

#define ENV_PAYLOAD_SIZE	128
#define MSG_WITH_ENV		1
#define MSG_WITHOUT_ENV		0
#define EVENT_ID_MAX 32

/*
 * Modules can use those two parameters below when register
 * to diagnose module.Message will be filerted when they meet
 * the conditions listed below:
 * 1.message with the same event ID,and the time interval is
 *   less than DIAG_MSG_INTERVAL_MIN
 * 2.message with the same event ID,and the time interval is
 *   between DIAG_MSG_INTERVAL_MIN & DIAG_MSG_INTERVAL_MAX,but
 *   the event status has not changed*/
#define DIAG_MSG_INTERVAL_MIN 100
#define DIAG_MSG_INTERVAL_MAX 148

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
	ModuleDiag_cpu_cal,
	ModuleDiag_mpu,
	ModuleDiag_uart,
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

/* event ID
 *
 * Define the event id for your own module below,
 * note: the counts of event ID of each module shall
 * not go over EVENT_ID_MAX, and start from 1, end
 * with EVENT_ID_MAX - 1.
 */

/* diag driver module event id */
enum diag_driver_module_eventid {
	EventIdKernelToUserSelfTest = 1,
	EventIdKernelToUserSelfTest2 = 2,
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
	EventIdVioIspErr,
	EventIdVioGdc0Err,
	EventIdVioGdc1Err,
	EventIdVioLdcErr,
	EventIdVioPymErr,
	EventIdVioFrameLost
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

/* cpu cal module event id */
enum diag_cpu_cal_module_eventid {
	EventIdCpuCalTestInitErr = 1,
	EventIdCpuCalTestDivErr,
	EventIdCpuCalTestBitANDORRErr,
	EventIdCpuCalTestAddSubErr,
	EventIdCpuCalTestADRPErr,
	EventIdCpuCalTestAdd1Err,
	EventIdCpuCalTestAdd2rr,
	EventIdCpuCalTestBitBICErr,
	EventIdCpuCalTestASRVrr,
	EventIdCpuCalTestCRCErr,
	EventIdCpuCalTestBitOpErr,
	EventIdCpuCalTestCondSel1rr,
	EventIdCpuCalTestLoadErr,
	EventIdCpuCalTestJumpBranErr,
	EventIdCpuCalTestGenRegErr,
	EventIdCpuCalTestMultiErr,
	EventIdCpuCalTestStoreErr,
	EventIdCpuCalTestSBFMErr,
	EventIdCpuCalTestSUB1Err,
	EventIdCpuCalTestSUB2Err,
	EventIdCpuCalTestLogicErr,
	EventIdCpuCalTestConSel2Err,
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

/* mpu module event id */
enum diag_mpu_module_eventid {
	EventIdMpuCNN0FetchErr = 12,
	EventIdMpuCNN1FetchErr,
	EventIdMpuCNN0OtherErr,
	EventIdMpuCNN1OtherErr,
	EventIdMpuVioM0Err = 29,
	EventIdMpuVpuErr,
	EventIdMpuVioM1Err,
};

/* uart module event id */
enum diag_uart_module_eventid {
	EventIdUart0Err = 1,
	EventIdUart1Err,
	EventIdUart2Err,
	EventIdUart3Err,
};

/*
 * diagnose event struct defintion
 */
struct diag_event {
	uint16_t module_id;
	uint16_t event_id;
	uint8_t event_prio;		//event priority
	uint8_t event_sta;		//event status

	uint8_t payload[ENV_PAYLOAD_SIZE];
	uint8_t env_len;
	uint8_t when;
};

/* header part in struct diag_msg */
struct diag_msg_header {
	uint8_t msg_type;	// 1:with env, 0:without env
	uint16_t msg_len;	// message length
	uint32_t checksum;  //checksum of the data
};

/* data part in struct diag_msg */
struct diag_msg_data {
	uint16_t module_id;
	uint16_t event_id;
	uint8_t event_sta;		//event status
	uint8_t payload[ENV_PAYLOAD_SIZE];
	uint8_t env_len;
	uint8_t when;
};

/*
 * message send from kernel space to user space
 */
struct diag_msg {
	struct diag_msg_header header;
	struct diag_msg_data data;
};

/*
 * Each event_id may have an unique handling method.
 */
typedef void (*callback_t)(void *p, size_t len);
struct diag_event_id_handle {
	uint8_t event_id;		//event id
	uint32_t min_snd_ms;	//minimum interval between message send
	uint32_t max_snd_ms;	//maximum interval between message send
	callback_t cb;			//callback to process specific logic
};

/*
 * register struct, used by diagnose_register function to
 * provide register information.
 */
struct diag_register_info {
	uint16_t module_id;
	struct diag_event_id_handle event_handle[EVENT_ID_MAX];
	uint8_t event_cnt;
};

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
extern int diag_register(uint16_t module_id, uint16_t event_id,
		size_t envdata_max_size, uint32_t min_snd_ms,
		uint32_t max_time_out_snd, void (*rcvcallback)(void *p, size_t len));

/*
 * register to the diagnose driver, other modules should call
 * this function before send event to the diagnose driver.
 */
extern int32_t diagnose_register(const struct diag_register_info *register_info);

/*
 * Send event to the diagnose driver.
 *
 * @event: diagnose event
 * @return: 0 on success, -1 on failure
 */
extern int32_t diagnose_send_event(struct diag_event *event);

/*
 * unregister to the diagnose driver.
 */
extern int32_t diagnose_unregister(uint16_t module);

/*
 * unregister to the diagnose event.
 */
extern int32_t diag_event_unregister(uint16_t module, uint16_t event);

#endif
