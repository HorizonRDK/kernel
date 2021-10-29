/*
 *			 COPYRIGHT NOTICE
 *		 Copyright 2019 Horizon Robotics, Inc.
 *			 All rights reserved.
 */

#ifndef _HBIPC_LITE_H_
#define _HBIPC_LITE_H_

#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/semaphore.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include "bif_lite.h"

#define DEBUG

#ifdef DEBUG
#define hbipc_debug pr_info
#else
#define hbipc_debug(format, ...) do {} while (0)
#endif

#define hbipc_error pr_err

enum domain_id {
	X2BIFSPI,
	X2BIFSD,
	X2ETH,
};

struct domain_info {
	char *domain_name;
	int domain_id;
	char *device_name;
	enum ap_type type;
	enum working_mode mode;
	int crc_enable;
	struct channel_config channel_cfg;
};

#define UUID_LEN (16)

struct resource_queue {
	struct list_head list;
	spinlock_t lock;
	int frame_count;
	// [20201217]: add rx threshold
	int frame_drop_count;
};

struct session {
	int domain_id;
	int provider_id;
	int client_id;
};

struct session_desc {
	int valid;
	int connected;
	int domain_id;
	int provider_id;
	int client_id;
	struct resource_queue recv_list;
	struct semaphore frame_count_sem;
	// [20201217]: add rx threshold
	int rx_threshold;
	int flowcontrol_active_flag;
	int flowcontrol_passive_flag;
	int need_set_flowcontrol;
	int need_clear_flowcontrol;
};

#define SESSION_COUNT_MAX (5)
struct session_info {
	struct session_desc session_array[SESSION_COUNT_MAX];
	int count;
	int first_avail;
};

struct provider_desc {
	int valid;
	int provider_id;
	struct session_info session;
	// [20201217]: add rx threshold
	int rx_threshold;
};

#define PROVIDER_COUNT_MAX (1)
struct provider_info {
	struct provider_desc provider_array[PROVIDER_COUNT_MAX];
	int count;
	int first_avail;
};

struct server_desc {
	int valid;
	unsigned char server_id[UUID_LEN];
	struct provider_info provider;
	// [20201217]: add rx threshold
	int rx_threshold;
};

#define SERVER_COUNT_MAX (12)
struct server_info {
	struct server_desc server_array[SERVER_COUNT_MAX];
	int count;
	int first_avail;
};

struct provider_start_desc {
	int valid;
	int client_id;
};

#define PROVIDER_START_COUNT_MAX (5)
struct provider_start_info {
	struct provider_start_desc start_array[PROVIDER_START_COUNT_MAX];
	int count;
	int first_avail;
};

struct provider_server {
	int valid;
	int provider_id;
	unsigned char server_id[UUID_LEN];
	struct provider_start_info start_list;
};

#define PROVIDER_SERVER_MAP_COUNT ((SERVER_COUNT_MAX) * (PROVIDER_COUNT_MAX))
struct provider_server_map {
	struct provider_server map_array[PROVIDER_SERVER_MAP_COUNT];
	int count;
	int first_avail;
};

// protected by connect_mutex in domain
struct pid_serverid {
	unsigned char server_id[UUID_LEN];
	int valid;
	int pid;
};

struct pid_providerid_map {
	struct pid_serverid map[SERVER_COUNT_MAX];
	int (*get_index)(struct pid_providerid_map *);
	int (*set_index)(struct pid_providerid_map *, int,
	int, unsigned char *);
	int (*put_index)(struct pid_providerid_map *, int);
	int (*find_index)(struct pid_providerid_map *, int, unsigned char *);
	int (*find_pid)(struct pid_providerid_map *, int);
	int (*is_valid)(struct pid_providerid_map *, int);
};

struct comm_domain_statistic {
	int irq_handler_count;
	int rx_work_func_count;
	int interrupt_recv_count;
	int manage_recv_count;
	int data_recv_count;
	int manage_frame_count;
	int data_frame_count;
	int up_sem_count;
	int invalid_data_frame_count;
	int send_manage_count;
	int rx_flowcontrol_count;
	int rx_flowcontrol_flag;
	int write_call_count;
	int write_real_count;
	int read_call_count;
	int read_real_count;
	int accept_count;
	int write_resend_count;
	int write_resend_over_count;
	int mang_resend_count;
	int mang_resend_over_count;
	int concede_manage_send_count;
	int concede_data_send_count;
	int concede_data_recv_count;
	int release_data_frame_count;
	int nonblock_write_nomem_count;
	int frame_drop_count;
};

struct comm_domain {
	char *domain_name;
	int domain_id;
	char *device_name;
	struct mutex write_mutex;
	struct mutex read_mutex;
	struct server_info server;
	struct mutex connect_mutex;
	struct list_head manage_frame_list;
	struct provider_server_map map;
	struct pid_providerid_map providerid_map;
	struct comm_channel channel;
	int session_count;
	int unaccept_session_count;
	int block;
	struct comm_domain_statistic domain_statistics;
	enum ap_type type;
	enum working_mode mode;
	int crc_enable;
	int manage_send;
	int mang_send_error;
	char *mang_frame_send_error_buf;
	int data_send;
	int data_recv;
	int first_connect_frame;
};

struct send_mang_data {
	int result;
	int domain_id;
	unsigned char server_id[UUID_LEN];
	int provider_id;
	int client_id;
	unsigned long long buffer;
	int len;
};

struct hbipc_header {
	int length;
	int domain_id;
	int provider_id;
	int client_id;
};
#define HBIPC_HEADER_LEN (sizeof(struct hbipc_header))

#define MSG_TEXT_SIZE (64)
struct manage_message {
	int type;
	int reserve;
	int seq_num;
	char msg_text[MSG_TEXT_SIZE];
};
#define MANAGE_MSG_LEN (sizeof(struct manage_message))

#define MANAGE_CMD_REGISTER_PROVIDER                 (100)
#define MANAGE_CMD_UNREGISTER_PROVIDER               (101)
#define MANAGE_CMD_CONNECT_REQ                       (102)
#define MANAGE_CMD_DISCONNECT_REQ                    (103)
#define MANAGE_CMD_KEEPALIVE                         (104)
#define MANAGE_CMD_QUERY_SERVER                      (105)
#define MANAGE_CMD_QUERY_REGISTER                    (106)
#define MANAGE_CMD_SET_FLOWCONTROL                   (107)
#define MANAGE_CMD_CLEAR_FLOWCONTROL                 (108)

int get_map_index(struct provider_server_map *map, int provider_id);
int get_start_list_first_avail_index(struct provider_start_info *start_inf);
int get_start_index(struct provider_start_info *start_inf, int client_id);
int get_map_index_from_server(struct provider_server_map *map,
struct send_mang_data *data);
int domain_init(struct comm_domain *domain, struct domain_info *domain_inf);
void domain_deinit(struct comm_domain *domain);
int bif_lite_init_domain(struct comm_domain *domain);
void bif_lite_exit_domain(struct comm_domain *domain);
int bif_lite_irq_register_domain(struct comm_domain *domain,
irq_handler_t irq_handler);
int bif_lite_irq_unregister_domain(struct comm_domain *domain);
void bif_del_frame_domain(struct comm_domain *domain,
struct bif_frame_cache *frame);
int bif_tx_put_frame_domain(struct comm_domain *domain, void *data,
int len, struct send_mang_data *mang_data);
int recv_handle_stock_frame(struct comm_domain *domain);
struct session_desc *is_valid_session(struct comm_domain *domain,
struct send_mang_data *data, struct server_desc **server_des,
struct provider_desc **provider_des);
int register_server_provider(struct comm_domain *domain,
struct send_mang_data *data);
int unregister_server_provider(struct comm_domain *domain,
struct send_mang_data *data);
int unregister_server_provider_abnormal(struct comm_domain *domain,
struct send_mang_data *data);
int disconnect_stopserver_abnormal(
struct comm_domain *domain, struct send_mang_data *data);
int register_connect(struct comm_domain *domain,
struct send_mang_data *data);
int unregister_connect(struct comm_domain *domain,
struct send_mang_data *data);
int recv_handle_manage_frame(struct comm_domain *domain);
int recv_handle_data_frame(struct comm_domain *domain);
int recv_frame_interrupt(struct comm_domain *domain);
int accept_session(struct comm_domain *domain,
struct send_mang_data *data, struct session_desc **connect);
void bif_del_session_frame_domain(struct comm_domain *domain,
struct bif_frame_cache *frame);
int domain_stock_frame_num(struct comm_domain *domain);
int start_server(struct comm_domain *domain, struct send_mang_data *data);
int stop_server(struct comm_domain *domain, struct send_mang_data *data);
int mang_frame_send2opposite(struct comm_domain *domain,
int type, struct send_mang_data *data);
int mang_frame_send2opposite_without_lock(struct comm_domain *domain,
int type, struct send_mang_data *data);
void clear_server_cp_manager(struct comm_domain *domain);
int domain_register_high_level_clear(struct comm_domain *domain, clear_func_t clear_func);
void domain_unregister_high_level_clear(struct comm_domain *domain);
void clear_invalid_connect_ap_abnormal(struct comm_domain *domain);
#ifdef CONFIG_HOBOT_BIF_ETHERNET
int recv_frame_eth(struct comm_domain *domain);
#endif
int bif_domain_send_irq(struct comm_domain *domain);
int resource_queue_count(struct resource_queue *queue);
// [20201130]: we need get_server_index from server_id in dev layer
int get_server_index(struct server_info *server_inf,
unsigned char *server_id);
int resource_queue_drop_count(struct resource_queue *queue);

#endif  /* _HBIPC_LITE_H_ */
