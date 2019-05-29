/***************************************************************************
 *   Copyright (C) 2019 by horizon.                                        *
 *   bo01.chen@horizon.ai                                                  *
 *                                                                         *
 *   Diag netlink header file.                                             *
 *	 version: v1.0                                                     *
 *                                                                         *
 ***************************************************************************/
#ifndef NETLINK_DIAG_H
#define NETLINK_DIAG_H

#define DIAG_MSG_HEAD 0xAA	// diag msg identifier.
#define DIAG_MSG_VER  0x01	// What version of the dig msg
#define FRAGMENT_SIZE 0x4000	// netlink msg fragment max size.

extern struct net init_net;

/* frame type */
typedef enum {
	/*kernel module or app send event id
	 * and it's status to diag app.
	 */
	REPORT_EVENT_STAT = 1,

	/* kernel module or app send event
	 * corresponding env data to diag app.
	 */
	SEND_ENV_DATA = 2,

	/* diag app send this cmd to kernel or
	 * other app request to get env data.
	 */
	READ_ENV_DATA = 3,

} diag_msg_type;

/*
 * diag msg packt header format
 */
typedef struct {
	uint8_t		packt_ident;
	uint8_t		version;	// diag msg format version.
	diag_msg_type	type;
	uint8_t		start;		// Is diag msg start fragment.
	uint8_t		end;		// Is diag msg end fragment.
	uint32_t	seq;		// diag msg fragment sequence.
	uint32_t	len;		// payload data len.
} __packed diag_msg_hdr;

/*
 * diag msg packt format
 */
typedef struct {
	diag_msg_hdr head;
	uint8_t *data;		// payload

	/* From the beginning of the packet
	 * to the end of the data.
	 */
	uint32_t checksum;

} __packed diag_msg;

/*
 * summary of all kernel and userspace event id.
 */
typedef enum {
	IMAGE_FRAME	= 0, // range belong to kernel modules.
	DIAG_DEV_KERNEL_TO_USER_SELFTEST,
	EVENT_MAX	= 11,

	/* userspace app event below */

} diag_event_id;

/*
 * event status.
 */
typedef enum {
	DIAG_EVENT_UNKNOWN = 0,
	DIAG_EVENT_SUCCESS = 1,
	DIAG_EVENT_FAIL = 2,

} diag_event_stat;

/*
 * event id and it's sta package,put
 * this package in the payload of diag msg.
 */
typedef struct {
	diag_event_id event_id;
	diag_event_stat sta;
} __packed report_event_sta_pack;

/*
 * when is the environmental data generated.
 */
typedef enum {
	GEN_ENV_DATA_WHEN_ERROR = 1,
	GEN_ENV_DATA_WHEN_RESUME_SUCCESS = 2,
	GEN_ENV_DATA_WHEN_SUCCESS = 3,
} env_data_type;

/*
 * env data head
 */
typedef struct {
	report_event_sta_pack pack_info;
	env_data_type type;

} __packed env_data_head;

/*
 * env data structure
 */
typedef struct {
	env_data_head head;
	uint8_t *data;

} __packed env_data_pack;

/*
 * send event sta and it's env data to the diag app.
 * @event event id
 * @sta event sta
 * @env_data_type When is the environmental data generated.
 * @env_data env data
 * @env_len env data len.
 * @return -1:error, >=0:OK
 */
extern int diag_send_event_stat_and_env_data(diag_event_id event,
			diag_event_stat sta,
			env_data_type env_typ,
			uint8_t *env_data,
			size_t env_len);

#endif


