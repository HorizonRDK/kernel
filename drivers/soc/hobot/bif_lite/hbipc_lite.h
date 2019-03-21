#ifndef _HBIPC_LITE_H_
#define _HBIPC_LITE_H_

#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/semaphore.h>
#include "bif_lite_utility.h"

#define DEBUG

#ifdef DEBUG
#define hbipc_debug printk
#else
#define hbipc_debug(format, ...) do {} while (0)
#endif

#define hbipc_error printk

struct domain_info {
	char *domain_name;
	int domain_id;
	char *device_name;
};
extern struct domain_info domain_config;

#define UUID_LEN (16)

struct resource_queue {
	struct list_head list;
	int frame_count;
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
};

#define PROVIDER_COUNT_MAX (5)
struct provider_info {
	struct provider_desc provider_array[PROVIDER_COUNT_MAX];
	int count;
	int first_avail;
};

struct server_desc {
	int valid;
	unsigned char server_id[UUID_LEN];
	struct provider_info provider;
};

#define SERVER_COUNT_MAX (5)
struct server_info {
	struct server_desc server_array[SERVER_COUNT_MAX];
	int count;
	int first_avail;
};

struct provider_server {
	int valid;
	int provider_id;
	unsigned char server_id[UUID_LEN];
};

#define PROVIDER_SERVER_MAP_COUNT ((SERVER_COUNT_MAX) * (PROVIDER_COUNT_MAX))
struct provider_server_map {
	struct provider_server map_array[PROVIDER_SERVER_MAP_COUNT];
	int count;
	int first_avail;
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
};
extern struct comm_domain domain;

struct send_mang_data {
	int result;
	int domain_id;
	unsigned char server_id[UUID_LEN];
	int provider_id;
	int client_id;
	char *buffer;
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

void resource_queue_init(struct resource_queue *queue);
void resource_queue_deinit(struct resource_queue *queue);
void session_desc_init(struct session_desc *session_des);
void session_desc_deinit(struct session_desc *session_des);
void session_info_init(struct session_info *session_inf);
void session_info_deinit(struct session_info *session_inf);
void provider_desc_init(struct provider_desc *provider_des);
void provider_desc_deinit(struct provider_desc *provider_des);
void provider_info_init(struct provider_info *provider_inf);
void provider_info_deinit(struct provider_info *provider_inf);
void server_desc_init(struct server_desc *server_des);
void server_desc_deinit(struct server_desc *server_des);
void server_info_init(struct server_info *server_inf);
void server_info_deinit(struct server_info *server_inf);
int domain_init(struct comm_domain *domain, struct domain_info *domain_inf);
void domain_deinit(struct comm_domain *domain);
int get_session_first_avail_index(struct session_info *session_inf);
int get_provider_first_avail_index(struct provider_info *provider_inf);
int get_server_first_avail_index(struct server_info *server_inf);
int get_server_index(struct server_info *server_inf, unsigned char *server_id);
int get_provider_index(struct provider_info *provider_inf, int provider_id);
int get_match_provider_index(struct provider_info *provider_inf,
char *parameter);
int get_session_index(struct session_info *session_inf,
struct session *connect);
int get_session_first_nonconnect_index(struct session_info *session_inf);
int regisger_server(struct send_mang_data *data);
int unregister_server(struct send_mang_data *data);
int register_provider(struct send_mang_data *data);
int unregister_provider(struct send_mang_data *data);
int register_server_provider(struct send_mang_data *data);
int unregister_server_provider(struct send_mang_data *data);
int accept_session(struct send_mang_data *data, struct session_desc **connect);
int recv_handle_manage_frame(void);
int recv_handle_data_frame(struct session_desc *session_des,
struct bif_frame_cache **frame);
int handle_manage_frame(struct bif_frame_cache *frame);
int register_connect(struct send_mang_data *data);
int unregister_connect(struct send_mang_data *data);
int get_map_first_avail_index(struct provider_server_map *map);
int get_map_index(struct provider_server_map *map, int provider_id);
int get_map_index_with_lock(struct provider_server_map *map, int provider_id);
int register_map(struct send_mang_data *data);
int register_map_with_lock(struct send_mang_data *data);
int unregister_map(struct send_mang_data *data);
int unregister_map_with_lock(struct send_mang_data *data);
struct session_desc *is_valid_session(struct send_mang_data *data,
struct server_desc **server_des, struct provider_desc **provider_des);
void provider_server_init(struct provider_server *relation);
void provider_server_deinit(struct provider_server *relation);
void provider_server_map_init(struct provider_server_map *map);
void provider_server_map_deinit(struct provider_server_map *map);
int start_server(struct send_mang_data *data, int *provider_id);

#endif  /* _HBIPC_LITE_H_ */
