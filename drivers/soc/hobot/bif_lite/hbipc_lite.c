/*
 *			 COPYRIGHT NOTICE
 *		 Copyright 2019 Horizon Robotics, Inc.
 *			 All rights reserved.
 */

#include <linux/list.h>
#include <linux/interrupt.h>
#include "hbipc_lite.h"
#include "hbipc_errno.h"

static void resource_queue_init(struct resource_queue *queue)
{
	INIT_LIST_HEAD(&queue->list);
	queue->frame_count = 0;
	spin_lock_init(&queue->lock);
}

static void resource_queue_deinit(struct comm_domain *domain,
struct resource_queue *queue)
{
	// need remove frame
	struct list_head *pos = NULL;
	struct list_head *n = NULL;
	struct bif_frame_cache *bif_frame = NULL;

	pr_debug("resource_queue_deinit: %d\n", queue->frame_count);

	spin_lock(&queue->lock);
	list_for_each_safe(pos, n, &queue->list) {
		list_del(pos);
		bif_frame =
		list_entry(pos, struct bif_frame_cache, frame_cache_list);
		bif_del_frame_from_session_list(&domain->channel, bif_frame);
		--queue->frame_count;
	}
	spin_unlock(&queue->lock);
}

static void session_desc_init(struct session_desc *session_des)
{
	session_des->valid = 0;
	session_des->connected = 0;
	session_des->domain_id = -1;
	session_des->provider_id = -1;
	session_des->client_id = -1;
	resource_queue_init(&session_des->recv_list);
	sema_init(&session_des->frame_count_sem, 0);
}

static void session_desc_deinit(struct comm_domain *domain,
struct session_desc *session_des)
{
	session_des->valid = 0;
	session_des->connected = 0;
	session_des->domain_id = -1;
	session_des->provider_id = -1;
	session_des->client_id = -1;
	resource_queue_deinit(domain, &session_des->recv_list);
}

static void session_info_init(struct session_info *session_inf)
{
	int i = 0;

	for (i = 0; i < SESSION_COUNT_MAX; ++i)
		session_desc_init(session_inf->session_array + i);
	session_inf->count = SESSION_COUNT_MAX;
	session_inf->first_avail = 0;
}

static void session_info_deinit(struct comm_domain *domain,
struct session_info *session_inf)
{
	int i = 0;

	for (i = 0; i < SESSION_COUNT_MAX; ++i)
		session_desc_deinit(domain, session_inf->session_array + i);
	session_inf->count = 0;
	session_inf->first_avail = 0;
}

static void provider_desc_init(struct provider_desc *provider_des)
{
	provider_des->valid = 0;
	provider_des->provider_id = 0;
	session_info_init(&provider_des->session);
}

static void provider_desc_deinit(struct comm_domain *domain,
struct provider_desc *provider_des)
{
	provider_des->valid = 0;
	provider_des->provider_id = 0;
	session_info_deinit(domain, &provider_des->session);
}

static void provider_info_init(struct provider_info *provider_inf)
{
	int i = 0;

	for (i = 0; i < PROVIDER_COUNT_MAX; ++i)
		provider_desc_init(provider_inf->provider_array + i);
	provider_inf->count = PROVIDER_COUNT_MAX;
	provider_inf->first_avail = 0;
}

static void provider_info_deinit(struct comm_domain *domain,
struct provider_info *provider_inf)
{
	int i = 0;

	for (i = 0; i < PROVIDER_COUNT_MAX; ++i)
		provider_desc_deinit(domain, provider_inf->provider_array + i);
	provider_inf->count = 0;
	provider_inf->first_avail = 0;
}

static void server_desc_init(struct server_desc *server_des)
{
	server_des->valid = 0;
	memset(server_des->server_id, 0, UUID_LEN);
	provider_info_init(&server_des->provider);
}

static void server_desc_deinit(struct comm_domain *domain,
struct server_desc *server_des)
{
	server_des->valid = 0;
	memset(server_des->server_id, 0, UUID_LEN);
	provider_info_deinit(domain, &server_des->provider);
}

static void server_info_init(struct server_info *server_inf)
{
	int i = 0;

	for (i = 0; i < SERVER_COUNT_MAX; ++i)
		server_desc_init(server_inf->server_array + i);
	server_inf->count = SERVER_COUNT_MAX;
	server_inf->first_avail = 0;
}

static void server_info_deinit(struct comm_domain *domain,
struct server_info *server_inf)
{
	int i = 0;

	for (i = 0; i < SERVER_COUNT_MAX; ++i)
		server_desc_deinit(domain, server_inf->server_array + i);
	server_inf->count = 0;
	server_inf->first_avail = 0;
}

static void provider_start_desc_init(struct provider_start_desc *start_des)
{
	start_des->valid = 0;
	start_des->client_id = 0;
}

static void provider_start_desc_deinit(struct provider_start_desc *start_des)
{
	start_des->valid = 0;
	start_des->client_id = 0;
}

static void provider_start_info_init(struct provider_start_info *start_info)
{
	int i = 0;

	for (i = 0; i < PROVIDER_START_COUNT_MAX; ++i)
		provider_start_desc_init(start_info->start_array + i);
	start_info->count = PROVIDER_START_COUNT_MAX;
	start_info->first_avail = 0;
}

static void provider_start_info_deinit(struct provider_start_info *start_info)
{
	int i = 0;

	for (i = 0; i < PROVIDER_START_COUNT_MAX; ++i)
		provider_start_desc_deinit(start_info->start_array + i);
	start_info->count = 0;
	start_info->first_avail = 0;
}

static void provider_server_init(struct provider_server *relation)
{
	relation->valid = 0;
	relation->provider_id = -1;
	memset(relation->server_id, 0, UUID_LEN);
	provider_start_info_init(&relation->start_list);
}

static void provider_server_deinit(struct provider_server *relation)
{
	relation->valid = 0;
	relation->provider_id = -1;
	memset(relation->server_id, 0, UUID_LEN);
	provider_start_info_deinit(&relation->start_list);
}

static void provider_server_map_init(struct provider_server_map *map)
{
	int i = 0;

	for (i = 0; i < PROVIDER_SERVER_MAP_COUNT; ++i)
		provider_server_init(map->map_array + i);
	map->count = PROVIDER_SERVER_MAP_COUNT;
	map->first_avail = 0;
}

static void provider_server_map_deinit(struct provider_server_map *map)
{
	int i = 0;

	for (i = 0; i < PROVIDER_SERVER_MAP_COUNT; ++i)
		provider_server_deinit(map->map_array + i);
	map->count = 0;
	map->first_avail = 0;
}

#define TX_RETRY_MAX (100)
int mang_frame_send2opposite(struct comm_domain *domain,
int type, struct send_mang_data *data)
{
	char mang_frame_send_buf[HBIPC_HEADER_LEN + MANAGE_MSG_LEN];
	struct hbipc_header *header =
	(struct hbipc_header *)mang_frame_send_buf;
	struct manage_message *message =
	(struct manage_message *)(mang_frame_send_buf +
	HBIPC_HEADER_LEN);
	int *p = (int *)(message->msg_text);
	int ret = 0;
	int remaining_time = 0;
	int retry_count = 0;

	++domain->domain_statistics.send_manage_count;

	header->length = MANAGE_MSG_LEN;
	header->domain_id = data->domain_id;
	header->provider_id = 0;
	header->client_id = 0;
	message->type = type;

	switch (type) {
	case MANAGE_CMD_REGISTER_PROVIDER:
		/*
		 * pass information:
		 * domain_id
		 * server_id
		 * provider_id
		 */
		*p++ = data->domain_id;
		memcpy(p, data->server_id, UUID_LEN);
		p += 4;
		*p = data->provider_id;
		break;
	case MANAGE_CMD_UNREGISTER_PROVIDER:
		/*
		 * pass information:
		 * domain_id
		 * server_id
		 * provider_id
		 */
		*p++ = data->domain_id;
		memcpy(p, data->server_id, UUID_LEN);
		p += 4;
		*p = data->provider_id;
		break;
	case MANAGE_CMD_CONNECT_REQ:
		/*
		 * pass information:
		 * domain_id
		 * server_id
		 * provider_id
		 * client_id
		 */
		*p++ = data->domain_id;
		memcpy(p, data->server_id, UUID_LEN);
		p += 4;
		*p++ = data->provider_id;
		*p = data->client_id;
		break;
	case MANAGE_CMD_DISCONNECT_REQ:
		/*
		 * pass information:
		 * domain_id
		 * server_id
		 * provider_id
		 * client_id
		 */
		*p++ = data->domain_id;
		memcpy(p, data->server_id, UUID_LEN);
		p += 4;
		*p++ = data->provider_id;
		*p = data->client_id;
		break;
	case MANAGE_CMD_QUERY_SERVER:
		/* no message body */
		break;
	default:
		goto error;
	}

	domain->manage_send = 1;
	mutex_lock(&domain->write_mutex);

	// resend last error manage frame
	if (domain->mang_send_error) {
repeat_resend:
		ret = bif_tx_put_frame(&domain->channel,
		domain->mang_frame_send_error_buf,
			HBIPC_HEADER_LEN + MANAGE_MSG_LEN);
		if (ret < 0) {
			if (ret == BIF_TX_ERROR_NO_MEM) {
				++retry_count;
				++domain->domain_statistics.mang_resend_count;
				if (retry_count > TX_RETRY_MAX) {
					++domain->domain_statistics.mang_resend_over_count;
					hbipc_error("repeat put_frame overtry\n");
				} else {
					remaining_time =
					msleep_interruptible(5);
					if (!remaining_time) {
						// lint warning
						goto repeat_resend;
					} else {
						// prompt message
						hbipc_error("repeat send mang interruptible\n");
					}
				}
			} else {
				// lint warning
				hbipc_error("repeat bif_tx_put_frame error\n");
			}

			mutex_unlock(&domain->write_mutex);
			domain->manage_send = 0;
			goto error;
		} else {
			// manage frame resend successfully
			domain->mang_send_error = 0;
		}
	}
	retry_count = 0;

resend:
	ret = bif_tx_put_frame(&domain->channel, mang_frame_send_buf,
		HBIPC_HEADER_LEN + MANAGE_MSG_LEN);
	if (ret < 0) {
		if (ret == BIF_TX_ERROR_NO_MEM) {
			++retry_count;
	++domain->domain_statistics.mang_resend_count;
			if (retry_count > TX_RETRY_MAX) {
				++domain->domain_statistics.mang_resend_over_count;
				hbipc_error("bif_tx_put_frame overtry\n");
			} else {
				remaining_time = msleep_interruptible(5);
				if (!remaining_time) {
					// lint warning
					goto resend;
				} else {
					// prompt message
					hbipc_error("send mang interruptible\n");
				}
			}
		} else {
			// lint warning
			hbipc_error("bif_tx_put_frame error\n");
		}

		// prepare to resent error manage frame
		memcpy(domain->mang_frame_send_error_buf,
		mang_frame_send_buf,
		HBIPC_HEADER_LEN + MANAGE_MSG_LEN);
		domain->mang_send_error = 1;
		mutex_unlock(&domain->write_mutex);
		domain->manage_send = 0;
		goto error;
	}

	mutex_unlock(&domain->write_mutex);
	domain->manage_send = 0;

	return 0;
error:
	return -1;
}
EXPORT_SYMBOL(mang_frame_send2opposite);

int domain_init(struct comm_domain *domain, struct domain_info *domain_inf)
{
	mutex_init(&domain->write_mutex);
	mutex_init(&domain->read_mutex);
	mutex_init(&domain->connect_mutex);
	domain->domain_name = domain_inf->domain_name;
	domain->domain_id = domain_inf->domain_id;
	domain->device_name = domain_inf->device_name;
	server_info_init(&domain->server);
	provider_server_map_init(&domain->map);
	INIT_LIST_HEAD(&domain->manage_frame_list);
	domain->session_count = 0;
	domain->unaccept_session_count = 0;
	domain->block = 1;
	domain_inf->channel_cfg.block = 1;
	domain->type = domain_inf->type;
	domain->mode = domain_inf->mode;
	domain->crc_enable = domain_inf->crc_enable;

	domain->mang_frame_send_error_buf = bif_malloc(HBIPC_HEADER_LEN +
		MANAGE_MSG_LEN);
	if (!domain->mang_frame_send_error_buf) {
		pr_info("mang_frame_send_error_buf malloc fail\n");
		provider_server_map_deinit(&domain->map);
		server_info_deinit(domain, &domain->server);
		mutex_destroy(&domain->connect_mutex);
		mutex_destroy(&domain->read_mutex);
		mutex_destroy(&domain->write_mutex);
		return -1;
	}

	if (channel_init(&domain->channel, &domain_inf->channel_cfg) < 0) {
		pr_info("channel_init error\n");
		provider_server_map_deinit(&domain->map);
		server_info_deinit(domain, &domain->server);
		mutex_destroy(&domain->connect_mutex);
		mutex_destroy(&domain->read_mutex);
		mutex_destroy(&domain->write_mutex);
		return -1;
	}

	return 0;
}
EXPORT_SYMBOL(domain_init);

void domain_deinit(struct comm_domain *domain)
{
	domain->domain_name = NULL;
	domain->domain_id = -1;
	domain->device_name = NULL;
	server_info_deinit(domain, &domain->server);
	provider_server_map_deinit(&domain->map);
	// had better free manage frame
	INIT_LIST_HEAD(&domain->manage_frame_list);
	domain->session_count = 0;
	domain->unaccept_session_count = 0;
	channel_deinit(&domain->channel);
	mutex_destroy(&domain->write_mutex);
	mutex_destroy(&domain->read_mutex);
	mutex_destroy(&domain->connect_mutex);
	bif_free(domain->mang_frame_send_error_buf);
}
EXPORT_SYMBOL(domain_deinit);

static int get_session_first_avail_index(struct session_info *session_inf)
{
	int i = 0;

	for (i = 0; i < SESSION_COUNT_MAX; ++i) {
		if (!(session_inf->session_array[i].valid))
			break;
	}

	if (i < SESSION_COUNT_MAX)
		return i;
	else
		return -1;
}

static int get_session_first_nonconnect_index(struct session_info *session_inf)
{
	int i = 0;

	for (i = 0; i < SESSION_COUNT_MAX; ++i) {
		if ((session_inf->session_array[i].valid) &&
			!(session_inf->session_array[i].connected))
			break;
	}

	if (i < SESSION_COUNT_MAX)
		return i;
	else
		return -1;
}

static int get_provider_first_avail_index(struct provider_info *provider_inf)
{
	int i = 0;

	for (i = 0; i < PROVIDER_COUNT_MAX; ++i) {
		if (!(provider_inf->provider_array[i].valid))
			break;
	}

	if (i < PROVIDER_COUNT_MAX)
		return i;
	else
		return -1;
}

static int get_server_first_avail_index(struct server_info *server_inf)
{
	int i = 0;

	for (i = 0; i < SERVER_COUNT_MAX; ++i) {
		if (!(server_inf->server_array[i].valid))
			break;
	}

	if (i < SERVER_COUNT_MAX)
		return i;
	else
		return 0;
}

static int get_server_index(struct server_info *server_inf,
unsigned char *server_id)
{
	int i = 0;

	for (i = 0; i < SERVER_COUNT_MAX; ++i) {
		if ((server_inf->server_array[i].valid) &&
			(!memcmp(server_inf->server_array[i].server_id,
			server_id, UUID_LEN)))
			break;
	}

	if (i < SERVER_COUNT_MAX)
		return i;
	else
		return -1;
}

static int get_provider_index(struct provider_info *provider_inf,
int provider_id)
{
	int i = 0;

	for (i = 0; i < PROVIDER_COUNT_MAX; ++i) {
		if ((provider_inf->provider_array[i].valid) &&
			(provider_inf->provider_array[i].provider_id ==
			provider_id))
			break;
	}

	if (i < PROVIDER_COUNT_MAX)
		return i;
	else
		return -1;
}

static int get_session_index(struct session_info *session_inf,
struct session *connect)
{
	int i = 0;

	for (i = 0; i < SESSION_COUNT_MAX; ++i) {
		if ((session_inf->session_array[i].valid) &&
			(session_inf->session_array[i].provider_id ==
			connect->provider_id) &&
			(session_inf->session_array[i].client_id ==
			connect->client_id))
			break;
	}

	if (i < SESSION_COUNT_MAX)
		return i;
	else
		return -1;
}

int get_start_list_first_avail_index(struct provider_start_info *start_inf)
{
	int i = 0;

	for (i = 0; i < PROVIDER_START_COUNT_MAX; ++i) {
		if (!(start_inf->start_array[i].valid))
			break;
	}

	if (i < PROVIDER_START_COUNT_MAX)
		return i;
	else
		return -1;
}
EXPORT_SYMBOL(get_start_list_first_avail_index);

int get_start_index(struct provider_start_info *start_inf, int client_id)
{
	int i = 0;

	for (i = 0; i < PROVIDER_START_COUNT_MAX; ++i) {
		if ((start_inf->start_array[i].valid) &&
			(start_inf->start_array[i].client_id == client_id))
			break;
	}

	if (i < PROVIDER_START_COUNT_MAX)
		return i;
	else
		return -1;
}
EXPORT_SYMBOL(get_start_index);

static int get_map_first_avail_index(struct provider_server_map *map)
{
	int i = 0;

	for (i = 0; i < PROVIDER_SERVER_MAP_COUNT; ++i)
		if (!(map->map_array[i].valid))
			break;

	if (i < PROVIDER_SERVER_MAP_COUNT)
		return i;
	else
		return -1;
}

int get_map_index(struct provider_server_map *map, int provider_id)
{
	int i = 0;

	for (i = 0; i < PROVIDER_SERVER_MAP_COUNT; ++i)
		if ((map->map_array[i].valid) &&
	(map->map_array[i].provider_id == provider_id))
			break;

	if (i < PROVIDER_SERVER_MAP_COUNT)
		return i;
	else
		return -1;
}
EXPORT_SYMBOL(get_map_index);

int get_map_index_from_server(struct provider_server_map *map,
struct send_mang_data *data)
{
	int i = 0;

	for (i = 0; i < PROVIDER_SERVER_MAP_COUNT; ++i)
		if ((map->map_array[i].valid) &&
	(!memcmp(map->map_array[i].server_id, data->server_id, UUID_LEN)))
			break;

	if (i < PROVIDER_SERVER_MAP_COUNT)
		return i;
	else
		return -1;
}
EXPORT_SYMBOL(get_map_index_from_server);

struct session_desc *is_valid_session(struct comm_domain *domain,
struct send_mang_data *data, struct server_desc **server_des,
struct provider_desc **provider_des)
{
	int index = 0;
	struct server_desc *server = NULL;
	struct provider_desc *provider = NULL;
	struct provider_server *relation = NULL;
	struct session_desc *session_des = NULL;
	struct session connect;

	mutex_lock(&domain->connect_mutex);
	index = get_map_index(&domain->map, data->provider_id);
	if (index < 0)
		goto error;
	relation = domain->map.map_array + index;

	index = get_server_index(&domain->server, relation->server_id);
	if (index < 0)
		goto error;
	server = domain->server.server_array + index;

	index = get_provider_index(&server->provider, data->provider_id);
	if (index < 0)
		goto error;
	provider = server->provider.provider_array + index;

	connect.domain_id = data->domain_id;
	connect.provider_id = data->provider_id;
	connect.client_id = data->client_id;
	index = get_session_index(&provider->session, &connect);
	if (index < 0)
		goto error;
	session_des = provider->session.session_array + index;

	// return output parameters
	if (server_des)
		*server_des = server;

	if (provider_des)
		*provider_des = provider;

	mutex_unlock(&domain->connect_mutex);

	return session_des;
error:
	mutex_unlock(&domain->connect_mutex);
	return NULL;
}
EXPORT_SYMBOL(is_valid_session);

static int regisger_server(struct comm_domain *domain,
struct send_mang_data *data)
{
	struct server_desc *server = NULL;
	int ret = 0;

	// register server_id just first time
	if (get_server_index(&domain->server, data->server_id) < 0) {
		if (domain->server.count <= 0) {
			hbipc_error("server resource insufficient\n");
			ret = HBIPC_ERROR_RMT_RES_ALLOC_FAIL;
			goto error;
		} else {
			server = domain->server.server_array +
			domain->server.first_avail;
			server->valid = 1;
			memcpy(server->server_id, data->server_id, UUID_LEN);
			--domain->server.count;
			domain->server.first_avail =
			get_server_first_avail_index(&domain->server);
		}
	}

	hbipc_debug("server_info: count = %d first_avail = %d\n",
	domain->server.count, domain->server.first_avail);

	return 0;
error:
	return ret;
}

static int unregister_server(struct comm_domain *domain,
struct send_mang_data *data)
{
	int index = 0;
	struct server_desc *server = NULL;

	index = get_server_index(&domain->server, data->server_id);
	server = domain->server.server_array + index;

	// unregister server only no provider exist
	if (server->provider.count == PROVIDER_COUNT_MAX) {
		server->valid = 0;
		++domain->server.count;
		domain->server.first_avail =
		get_server_first_avail_index(&domain->server);
	}

	hbipc_debug("server_info: count = %d first_avail = %d\n",
	domain->server.count, domain->server.first_avail);

	return 0;
}

static int register_provider(struct comm_domain *domain,
struct send_mang_data *data)
{
	int index = 0;
	struct server_desc *server = NULL;
	struct provider_desc *provider = NULL;
	int ret = 0;

	index = get_server_index(&domain->server, data->server_id);
	server = domain->server.server_array + index;

	// register provider_id just first time
	if (get_provider_index(&server->provider, data->provider_id) < 0) {
		if (server->provider.count <= 0) {
			ret = HBIPC_ERROR_RMT_RES_ALLOC_FAIL;
			hbipc_error("provider resource insufficient\n");
			goto error;
		} else {
			provider = server->provider.provider_array +
			server->provider.first_avail;
			provider->valid = 1;
			provider->provider_id = data->provider_id;
			--server->provider.count;
			server->provider.first_avail =
			get_provider_first_avail_index(&server->provider);
		}
	} else {
		hbipc_error("provider duplicate register\n");
		ret = HBIPC_ERROR_REPEAT_REGISTER;
		goto error;
	}

	hbipc_debug("provider_info: count = %d first_avail = %d\n",
	server->provider.count, server->provider.first_avail);

	return 0;
error:
	return ret;
}

static int clear_connect(struct comm_domain *domain,
struct session_info *session_inf)
{
	int i = 0;

	for (i = 0; i < SESSION_COUNT_MAX; ++i) {
		if (session_inf->session_array[i].valid) {
			session_inf->session_array[i].valid = 0;
#ifdef CONFIG_HOBOT_BIF_AP
			session_inf->session_array[i].connected = 0;
#else
			if (!session_inf->session_array[i].connected)
				--domain->unaccept_session_count;
			else
				session_inf->session_array[i].connected = 0;
#endif
	resource_queue_deinit(domain,
		&(session_inf->session_array[i].recv_list));

			++session_inf->count;
			session_inf->first_avail =
			get_session_first_avail_index(session_inf);
			--domain->session_count;
#ifdef CONFIG_HOBOT_BIF_AP
			up(&(session_inf->session_array[i].frame_count_sem));
#endif
		}
	}

	hbipc_debug("session_info: count = %d first_avail = %d \
	session_count = %d unaccept_session_count = %d\n",
	session_inf->count, session_inf->first_avail,
	domain->session_count, domain->unaccept_session_count);

	return 0;
}

static int unregister_provider(struct comm_domain *domain,
struct send_mang_data *data)
{
	int index = 0;
	struct server_desc *server = NULL;
	struct provider_desc *provider = NULL;
	int ret = 0;
	int i = 0;

	index = get_server_index(&domain->server, data->server_id);
	if (index < 0) {
		ret = HBIPC_ERROR_INVALID_SERVERID;
		hbipc_error("invalid server_id:\n");
		for (i = 0; i < UUID_LEN; ++i)
			hbipc_error("%d	", data->server_id[i]);
		hbipc_error("\n");
		goto error;
	}
	server = domain->server.server_array + index;

	index = get_provider_index(&server->provider, data->provider_id);
	if (index < 0) {
		ret = HBIPC_ERROR_INVALID_PROVIDERID;
		hbipc_error("invalid provider_id: %d\n", data->provider_id);
		goto error;
	} else {
		provider = server->provider.provider_array + index;
		provider->valid = 0;
		clear_connect(domain, &provider->session);
		++server->provider.count;
		server->provider.first_avail =
		get_provider_first_avail_index(&server->provider);
	}

	hbipc_debug("provider_info: count = %d first_avail = %d\n",
	server->provider.count, server->provider.first_avail);

	return 0;
error:
	return ret;
}

static int register_map(struct comm_domain *domain,
struct send_mang_data *data)
{
	struct provider_server *relation = NULL;

	relation = domain->map.map_array + domain->map.first_avail;
	relation->valid = 1;
	memcpy(relation->server_id, data->server_id, UUID_LEN);
	relation->provider_id = data->provider_id;
	provider_start_info_init(&relation->start_list);
	--domain->map.count;
	domain->map.first_avail = get_map_first_avail_index(&domain->map);

	hbipc_debug("provider_server_map: count = %d first_avail = %d\r\n",
	domain->map.count, domain->map.first_avail);

	return 0;
}

static int unregister_map(struct comm_domain *domain,
struct send_mang_data *data)
{
	int index = 0;
	struct provider_server *relation = NULL;

	index = get_map_index(&domain->map, data->provider_id);
	relation = domain->map.map_array + index;
	relation->valid = 0;
	provider_start_info_deinit(&relation->start_list);
	++domain->map.count;
	domain->map.first_avail = get_map_first_avail_index(&domain->map);

	hbipc_debug("provider_server_map: count = %d first_avail = %d\r\n",
	domain->map.count, domain->map.first_avail);

	return 0;
}

int register_server_provider(struct comm_domain *domain,
struct send_mang_data *data)
{
	int ret = 0;

	mutex_lock(&domain->connect_mutex);
	ret = regisger_server(domain, data);
	if (ret < 0) {
		hbipc_error("reisger_server error\n");
		goto register_server_error;
	}

	ret = register_provider(domain, data);
	if (ret < 0) {
		hbipc_error("register_provider error\n");
		goto register_provider_error;
	}

	register_map(domain, data);

	mutex_unlock(&domain->connect_mutex);
#ifndef CONFIG_HOBOT_BIF_AP
	// send register provider datagram
	ret = mang_frame_send2opposite(domain,
		MANAGE_CMD_REGISTER_PROVIDER, data);
	if (ret < 0) {
		ret = HBIPC_ERROR_HW_TRANS_ERROR;
		hbipc_error("send MANG_CMD_REGISTER_PROVIDER error\n");
		goto send_cmd_error;
	}
#endif

	return 0;
#ifndef CONFIG_HOBOT_BIF_AP
send_cmd_error:
	mutex_lock(&domain->connect_mutex);
	unregister_map(domain, data);
	unregister_provider(domain, data);
#endif
register_provider_error:
	unregister_server(domain, data);
register_server_error:
	mutex_unlock(&domain->connect_mutex);
	return ret;
}
EXPORT_SYMBOL(register_server_provider);

static int register_server_provider_query(struct comm_domain *domain)
{
	int i = 0;
	int j = 0;
	struct server_desc *server = NULL;
	struct provider_desc *provider = NULL;
	int ret = 0;
	struct send_mang_data data;

	mutex_lock(&domain->connect_mutex);

	for (i = 0; i < SERVER_COUNT_MAX; ++i) {
		server = domain->server.server_array + i;
		if (server->valid) {
			for (j = 0; j < PROVIDER_COUNT_MAX; ++j) {
				provider = server->provider.provider_array + j;
				if (provider->valid) {
					data.domain_id = domain->domain_id;
					memcpy(data.server_id,
					server->server_id, UUID_LEN);
					data.provider_id =
					provider->provider_id;
					ret = mang_frame_send2opposite(domain,
					MANAGE_CMD_REGISTER_PROVIDER, &data);
					if (ret < 0) {
						// prompt message
						hbipc_error("query send MANG_CMD_REGISTER_PROVIDER error\n");
					}
				}
			}
		}
	}

	mutex_unlock(&domain->connect_mutex);

	return ret;
}

int unregister_server_provider(struct comm_domain *domain,
struct send_mang_data *data)
{
	int ret = 0;

	mutex_lock(&domain->connect_mutex);
	ret = unregister_provider(domain, data);
	if (ret < 0) {
		hbipc_error("unregister_provider error\n");
		mutex_unlock(&domain->connect_mutex);
		goto error;
	}

	unregister_server(domain, data);
	unregister_map(domain, data);

	mutex_unlock(&domain->connect_mutex);

#ifndef CONFIG_HOBOT_BIF_AP
	// send unregister provider datagram
	ret = mang_frame_send2opposite(domain,
		MANAGE_CMD_UNREGISTER_PROVIDER, data);
	if (ret < 0) {
		ret = HBIPC_ERROR_HW_TRANS_ERROR;
		hbipc_error("send MANG_CMD_UNREGISTER_PROVIDER error\n");
		goto error;
	}
#endif

	return 0;
error:
	return ret;
}
EXPORT_SYMBOL(unregister_server_provider);

int unregister_server_provider_abnormal(struct comm_domain *domain,
struct send_mang_data *data)
{
	int ret = 0;
	struct provider_server *relation = NULL;
	int pid = data->provider_id;
	int i = 0;
	struct server_desc *server = NULL;
	short int *provider_id_factor = NULL;

	mutex_lock(&domain->connect_mutex);

	for (i = 0; i < SERVER_COUNT_MAX; ++i) {
		server = domain->server.server_array + i;
		if (!server->valid)
			continue;
		provider_id_factor = (short int *)(server->server_id);
		data->provider_id = pid + *provider_id_factor;

	ret = get_map_index(&domain->map, data->provider_id);
	if (ret >= 0) {
		// abnormal shutdown
		relation = domain->map.map_array + ret;
		memcpy(data->server_id, relation->server_id, UUID_LEN);
	} else {
		// this provider_id is unregistered normally
		continue;
	}

	ret = unregister_provider(domain, data);
	if (ret < 0) {
		hbipc_error("unregister_provider error\n");
			continue;
	}

	unregister_server(domain, data);
	unregister_map(domain, data);

	mutex_unlock(&domain->connect_mutex);
	// send unregister provider datagram
	ret = mang_frame_send2opposite(domain,
		MANAGE_CMD_UNREGISTER_PROVIDER, data);
		mutex_lock(&domain->connect_mutex);
	if (ret < 0) {
		ret = HBIPC_ERROR_HW_TRANS_ERROR;
		hbipc_error("send MANG_CMD_UNREGISTER_PROVIDER error\n");
			continue;
		}
	}

	mutex_unlock(&domain->connect_mutex);

	return 0;
}
EXPORT_SYMBOL(unregister_server_provider_abnormal);

int disconnect_stopserver_abnormal(
struct comm_domain *domain, struct send_mang_data *data)
{
	int ret = 0;
	int i = 0;
	int j = 0;
	int k = 0;
	struct server_desc *server = NULL;
	struct provider_desc *provider = NULL;
	struct session_desc *connect = NULL;
	struct provider_server *relation = NULL;
	struct provider_start_desc *provider_start = NULL;

	mutex_lock(&domain->connect_mutex);

	// disconnect abnormal connect
	for (i = 0; i < SERVER_COUNT_MAX; ++i) {
		server = domain->server.server_array + i;
		if (server->valid) {
			for (j = 0; j < PROVIDER_COUNT_MAX; ++j) {
				provider = server->provider.provider_array + j;
				if (provider->valid) {
					for (k = 0; k < SESSION_COUNT_MAX;
					++k) {
						connect =
				provider->session.session_array + k;
				if ((connect->valid) &&
				(connect->client_id == data->client_id)) {
					// abnormal shutdown, should disconnect
					ret = get_map_index(&domain->map,
					connect->provider_id);
					if (ret < 0)
						continue;
					else {
						relation =
						domain->map.map_array + ret;
						memcpy(data->server_id,
						relation->server_id, UUID_LEN);
					}
					data->provider_id =
					connect->provider_id;
					connect->valid = 0;
					connect->connected = 0;
			resource_queue_deinit(domain,
				&connect->recv_list);
					++provider->session.count;
					provider->session.first_avail =
			get_session_first_avail_index(&provider->session);
					--domain->session_count;
					mutex_unlock(&domain->connect_mutex);
		mang_frame_send2opposite(domain,
			MANAGE_CMD_DISCONNECT_REQ, data);
					mutex_lock(&domain->connect_mutex);
				}
			}
		}
	}
		}
	}

	// stop abnormal server
	for (i = 0; i < PROVIDER_SERVER_MAP_COUNT; ++i) {
		relation = domain->map.map_array + i;
		if (relation->valid) {
			for (j = 0; j < PROVIDER_START_COUNT_MAX; ++j) {
				provider_start =
				relation->start_list.start_array + j;
				if ((provider_start->valid) &&
		(provider_start->client_id == data->client_id)) {
					provider_start->valid = 0;
					++relation->start_list.count;
					relation->start_list.first_avail =
		get_start_list_first_avail_index(&relation->start_list);
				}
			}
		}
	}

	mutex_unlock(&domain->connect_mutex);

	return 0;
}
EXPORT_SYMBOL(disconnect_stopserver_abnormal);

int register_connect(struct comm_domain *domain,
struct send_mang_data *data)
{
	int ret = 0;
	int index = 0;
	struct server_desc *server = NULL;
	struct provider_desc *provider = NULL;
	struct session_desc *connect = NULL;
#ifdef CONFIG_HOBOT_BIF_AP
	struct provider_server *relation = NULL;
#endif
	int i = 0;

	mutex_lock(&domain->connect_mutex);
#ifdef CONFIG_HOBOT_BIF_AP
	ret = get_map_index(&domain->map, data->provider_id);
	if (ret < 0) {
		hbipc_error("invalid provider: %d\n", data->provider_id);
		ret = HBIPC_ERROR_INVALID_PROVIDERID;
		goto error;
	} else {
		relation = domain->map.map_array + ret;
		memcpy(data->server_id, relation->server_id, UUID_LEN);
	}

	ret = get_start_index(&relation->start_list, data->client_id);
	if (ret < 0) {
		ret = HBIPC_ERROR_INVALID_PROVIDERID;
		goto error;
	}
#endif
	index = get_server_index(&domain->server, data->server_id);
	if (index < 0) {
		hbipc_error("invalid server_id:\n");
		for (i = 0; i < UUID_LEN; ++i)
			hbipc_error("%d	", data->server_id[i]);
		hbipc_error("\n");
		ret = HBIPC_ERROR_INVALID_SERVERID;
		goto error;
	}
	server = domain->server.server_array + index;

	index = get_provider_index(&server->provider, data->provider_id);
	if (index < 0) {
		hbipc_error("invalid provider_id: %d\n", data->provider_id);
		ret = HBIPC_ERROR_INVALID_PROVIDERID;
		goto error;
	}
	provider = server->provider.provider_array + index;

	if (provider->session.count <= 0) {
		hbipc_error("connect resource insufficient\n");
		ret = HBIPC_ERROR_RMT_RES_ALLOC_FAIL;
		goto error;
	}
	connect = provider->session.session_array +
	provider->session.first_avail;
	resource_queue_deinit(domain, &connect->recv_list);
	connect->valid = 1;
#ifdef CONFIG_HOBOT_BIF_AP
	connect->connected = 1;
#else
	connect->connected = 0;
#endif
	connect->domain_id = data->domain_id;
	connect->provider_id = data->provider_id;
	connect->client_id = data->client_id;
	--provider->session.count;
	provider->session.first_avail =
	get_session_first_avail_index(&provider->session);
	sema_init(&connect->frame_count_sem, 0);
	++domain->session_count;
#ifndef CONFIG_HOBOT_BIF_AP
	++domain->unaccept_session_count;
#endif
	mutex_unlock(&domain->connect_mutex);
#ifdef CONFIG_HOBOT_BIF_AP
	ret = mang_frame_send2opposite(domain,
		MANAGE_CMD_CONNECT_REQ, data);
	if (ret < 0) {
		ret = HBIPC_ERROR_HW_TRANS_ERROR;
		hbipc_error("send MANAGE_CMD_CONNECT_REQ error\n");
		goto send_cmd_error;
	}
#endif

	hbipc_debug("session_info: count = %d first_avail = %d\n",
	provider->session.count, provider->session.first_avail);

	return 0;
#ifdef CONFIG_HOBOT_BIF_AP
send_cmd_error:
	mutex_lock(&domain->connect_mutex);
	connect->valid = 0;
	connect->connected = 0;
	++provider->session.count;
	provider->session.first_avail =
	get_session_first_avail_index(&provider->session);
	--domain->session_count;
#endif
error:
	mutex_unlock(&domain->connect_mutex);
	return ret;
}
EXPORT_SYMBOL(register_connect);

int unregister_connect(struct comm_domain *domain,
struct send_mang_data *data)
{
	int ret = 0;
	int index = 0;
	struct server_desc *server = NULL;
	struct provider_desc *provider = NULL;
	struct session_desc *connect_des = NULL;
	struct session connect;
#ifdef CONFIG_HOBOT_BIF_AP
	struct provider_server *relation = NULL;
#endif
	int i = 0;

	mutex_lock(&domain->connect_mutex);
#ifdef CONFIG_HOBOT_BIF_AP
	ret = get_map_index(&domain->map, data->provider_id);
	if (ret < 0) {
		hbipc_error("invalid provider: %d\n", data->provider_id);
		ret = HBIPC_ERROR_INVALID_PROVIDERID;
		goto error;
	} else {
		relation = domain->map.map_array + ret;
		memcpy(data->server_id, relation->server_id, UUID_LEN);
	}
#endif
	index = get_server_index(&domain->server, data->server_id);
	if (index < 0) {
		hbipc_error("invalid server_id:\n");
		for (i = 0; i < UUID_LEN; ++i)
			hbipc_error("%d	", data->server_id[i]);
		hbipc_error("\n");
		ret = HBIPC_ERROR_INVALID_SERVERID;
		goto error;
	}
	server = domain->server.server_array + index;

	index = get_provider_index(&server->provider, data->provider_id);
	if (index < 0) {
		hbipc_error("invalid provider_id: %d\n", data->provider_id);
		ret = HBIPC_ERROR_INVALID_PROVIDERID;
		goto error;
	}
	provider = server->provider.provider_array + index;

	connect.domain_id = data->domain_id;
	connect.provider_id = data->provider_id;
	connect.client_id = data->client_id;
	index = get_session_index(&provider->session, &connect);
	if (index < 0) {
		hbipc_error("invalid session: %d_%d_%d\n", data->domain_id,
			data->provider_id, data->client_id);
		ret = HBIPC_ERROR_INVALID_SESSION;
		goto error;
	}
	connect_des = provider->session.session_array + index;

	connect_des->valid = 0;
	connect_des->connected = 0;
	resource_queue_deinit(domain, &connect_des->recv_list);
	++provider->session.count;
	provider->session.first_avail =
	get_session_first_avail_index(&provider->session);
	--domain->session_count;
#ifndef CONFIG_HOBOT_BIF_AP
	// wake up maybe block read ioctl in CP
	up(&connect_des->frame_count_sem);
#endif
	mutex_unlock(&domain->connect_mutex);
#ifdef CONFIG_HOBOT_BIF_AP
	ret = mang_frame_send2opposite(domain,
		MANAGE_CMD_DISCONNECT_REQ, data);
	if (ret < 0) {
		ret = HBIPC_ERROR_HW_TRANS_ERROR;
		hbipc_error("send MANAGE_CMD_DISCONNECT_REQ error\n");
		goto send_cmd_error;
	}
#endif

	hbipc_debug("session_info: count = %d first_avail = %d\n",
	provider->session.count, provider->session.first_avail);

	return 0;
#ifdef CONFIG_HOBOT_BIF_AP
send_cmd_error:
	mutex_lock(&domain->connect_mutex);
#endif
error:
	mutex_unlock(&domain->connect_mutex);
	return ret;
}
EXPORT_SYMBOL(unregister_connect);

static int handle_manage_frame(struct comm_domain *domain,
struct bif_frame_cache *frame)
{
	struct manage_message *message =
	(struct manage_message *)(frame->framecache + HBIPC_HEADER_LEN);
	int *p = (int *)(message->msg_text);
	struct send_mang_data data;

	hbipc_debug("manage frame type: %d\r\n", message->type);
	switch (message->type) {
	case MANAGE_CMD_CONNECT_REQ:
		/*
		 * get information:
		 * domain_id
		 * server_id
		 * provder_id
		 * client_id
		 */
		data.domain_id = *p++;
		memcpy(data.server_id, p, UUID_LEN);
		p += 4;
		data.provider_id = *p++;
		data.client_id = *p;

		if (register_connect(domain, &data) < 0)
			goto error;
		break;
	case MANAGE_CMD_DISCONNECT_REQ:
		/*
		 * get information:
		 * domain_id
		 * server_id
		 * provder_id
		 * client_id
		 */
		data.domain_id = *p++;
		memcpy(data.server_id, p, UUID_LEN);
		p += 4;
		data.provider_id = *p++;
		data.client_id = *p;

		if (unregister_connect(domain, &data) < 0)
			goto error;
		break;
	case MANAGE_CMD_REGISTER_PROVIDER:
		/*
		 * get information:
		 * domain_id
		 * server_id
		 * provider_id
		 */
		data.domain_id = *p++;
		memcpy(data.server_id, p, UUID_LEN);
		p += 4;
		data.provider_id = *p;

		if (register_server_provider(domain, &data) < 0)
			goto error;
		break;
	case MANAGE_CMD_UNREGISTER_PROVIDER:
		/*
		 * get information:
		 * domain_id
		 * server_id
		 * provider_id
		 */
		data.domain_id = *p++;
		memcpy(data.server_id, p, UUID_LEN);
		p += 4;
		data.provider_id = *p;

		if (unregister_server_provider(domain, &data) < 0)
			goto error;
		break;
	case MANAGE_CMD_QUERY_SERVER:
		register_server_provider_query(domain);
		break;
	default:
		goto error;
	}

	return 0;
error:
	return -1;
}

/*
 * return value:
 * -1: error & no frame get
 * 0: manage frame get
 * 1: othrer frame get
 */
int recv_handle_manage_frame(struct comm_domain *domain)
{
	int ret = 0;
	struct bif_frame_cache *frame = NULL;
	struct hbipc_header *header = NULL;
	struct send_mang_data data;
	struct session_desc *session_des = NULL;
	struct list_head *pos = NULL;
	struct bif_frame_cache *frame_tmp = NULL;

	mutex_lock(&domain->read_mutex);
	++domain->domain_statistics.manage_recv_count;
	if ((bif_rx_get_frame(&domain->channel, &frame) < 0) || (!frame)) {
		mutex_unlock(&domain->read_mutex);
		return -1;
	}

	// iterate frame cache list of channel
	// handle every frame get this time
	while (!list_empty(&(domain->channel.rx_frame_cache_p->frame_cache_list))) {
		// when enter while loop every time, read_mutex is locked
		// get the first frame from frame cache list
		pos = domain->channel.rx_frame_cache_p->frame_cache_list.next;
		frame_tmp =
		list_entry(pos, struct bif_frame_cache, frame_cache_list);
		header = (struct hbipc_header *)frame_tmp->framecache;

		if ((header->provider_id == 0) && (header->client_id == 0)) {
			// manage frame
			// just for handle frame link list relation
			++domain->domain_statistics.manage_frame_count;
			list_del(&frame_tmp->frame_cache_list);
			//bif_frame_decrease_count(&domain->channel);
			list_add_tail(&frame_tmp->frame_cache_list,
				&domain->manage_frame_list);
			//mutex_unlock(&domain->read_mutex);

			ret = handle_manage_frame(domain, frame_tmp);
			//mutex_lock(&domain->read_mutex);
			bif_del_frame_from_list(&domain->channel, frame_tmp);
			//list_del(pos);
			//kfree(frame_tmp);
		} else {
			// data frame
			// check session validity and insert data frame
			++domain->domain_statistics.data_frame_count;
			data.domain_id = header->domain_id;
			data.provider_id = header->provider_id;
			data.client_id = header->client_id;

			//mutex_unlock(&domain->read_mutex);
			session_des =
			is_valid_session(domain, &data, NULL, NULL);
			//mutex_lock(&domain->read_mutex);
			if (!session_des) {
				hbipc_debug("interrupt recv invalid session\n");
				bif_del_frame_from_list(&domain->channel,
				frame_tmp);
			} else {
				// at extreme condition, session_des maybe
				// invalid at here, but do not make harm
				// if register operation with init
				list_del(&frame_tmp->frame_cache_list);
				//bif_frame_decrease_count(&domain->channel);
				spin_lock(&(session_des->recv_list.lock));
				list_add_tail(&frame_tmp->frame_cache_list,
				&session_des->recv_list.list);
				++session_des->recv_list.frame_count;
				spin_unlock(&(session_des->recv_list.lock));
				up(&session_des->frame_count_sem);
				++domain->domain_statistics.up_sem_count;
			}
		}
	}
	mutex_unlock(&domain->read_mutex);

	return 0;
#if 0
	header = (struct hbipc_header *)frame->framecache;

	if ((header->provider_id == 0) && (header->client_id == 0)) {
		// manage frame
		// just for handle frame link list relation
		list_del(&frame->frame_cache_list);
		list_add_tail(&frame->frame_cache_list,
		&domain->manage_frame_list);
		mutex_unlock(&domain->read_mutex);

		ret = handle_manage_frame(domain, frame);
		mutex_lock(&domain->read_mutex);
		bif_del_frame_from_list(&domain->channel, frame);
		mutex_unlock(&domain->read_mutex);
		if (ret < 0)
			return -1;
		else
			return 0;
	} else {
		// data frame
		// check session validity and insert data frame
		data.domain_id = header->domain_id;
		data.provider_id = header->provider_id;
		data.client_id = header->client_id;

		mutex_unlock(&domain->read_mutex);
		session_des = is_valid_session(domain, &data, NULL, NULL);
		mutex_lock(&domain->read_mutex);
		if (!session_des) {
			hbipc_debug("mang recv invalid session\n");
			bif_del_frame_from_list(&domain->channel, frame);
		} else {
			list_del(&frame->frame_cache_list);
			list_add_tail(&frame->frame_cache_list,
			&session_des->recv_list.list);
			++session_des->recv_list.frame_count;
			up(&session_des->frame_count_sem);
		}

		mutex_unlock(&domain->read_mutex);

		return 1;
	}
#endif
}
EXPORT_SYMBOL(recv_handle_manage_frame);

int recv_handle_stock_frame(struct comm_domain *domain)
{
	struct bif_frame_cache *frame = NULL;
	struct hbipc_header *header = NULL;

	mutex_lock(&domain->read_mutex);
	while (!bif_rx_get_stock_frame(&domain->channel, &frame)) {
		header = (struct hbipc_header *)frame->framecache;

		if ((header->provider_id == 0) && (header->client_id == 0)) {
			// manage frame
			// just for handle frame link list relation
			list_del(&frame->frame_cache_list);
			list_add_tail(&frame->frame_cache_list,
			&domain->manage_frame_list);

			//mutex_unlock(&domain->read_mutex);
			handle_manage_frame(domain, frame);
			//mutex_lock(&domain->read_mutex);
			bif_del_frame_from_list(&domain->channel, frame);
		} else {
			// data frame
			bif_del_frame_from_list(&domain->channel, frame);
		}
	}
	mutex_unlock(&domain->read_mutex);

	return 0;
}
EXPORT_SYMBOL(recv_handle_stock_frame);

/*
 * return value:
 * -1: error & no frame get
 * 0: specific frame get
 * 1: othrer frame get
 */
int recv_handle_data_frame(struct comm_domain *domain)
{
	int ret = 0;
	struct bif_frame_cache *frame = NULL;
	struct hbipc_header *header = NULL;
	struct send_mang_data data;
	struct session_desc *session_des = NULL;
	struct list_head *pos = NULL;
	struct bif_frame_cache *frame_tmp = NULL;

	mutex_lock(&domain->read_mutex);
	++domain->domain_statistics.data_recv_count;
	if ((bif_rx_get_frame(&domain->channel, &frame) < 0)
		|| (!frame)) {
		mutex_unlock(&domain->read_mutex);
		return -1;
	}

	// iterate frame cache list of channel
	// handle every frame get this time
	while (!list_empty(&(domain->channel.rx_frame_cache_p->frame_cache_list))) {
		// when enter while loop every time, read_mutex is locked
		// get the first frame from frame cache list
		pos = domain->channel.rx_frame_cache_p->frame_cache_list.next;
		frame_tmp =
		list_entry(pos, struct bif_frame_cache, frame_cache_list);
		header = (struct hbipc_header *)frame_tmp->framecache;

		if ((header->provider_id == 0) && (header->client_id == 0)) {
			// manage frame
			// just for handle frame link list relation
			++domain->domain_statistics.manage_frame_count;
			list_del(&frame_tmp->frame_cache_list);
			//bif_frame_decrease_count(&domain->channel);
			list_add_tail(&frame_tmp->frame_cache_list,
			&domain->manage_frame_list);
			//mutex_unlock(&domain->read_mutex);

			ret = handle_manage_frame(domain, frame_tmp);
			//mutex_lock(&domain->read_mutex);
			bif_del_frame_from_list(&domain->channel, frame_tmp);
			//list_del(pos);
			//kfree(frame_tmp);
		} else {
			// data frame
			// check session validity and insert data frame
			++domain->domain_statistics.data_frame_count;
			data.domain_id = header->domain_id;
			data.provider_id = header->provider_id;
			data.client_id = header->client_id;

			//mutex_unlock(&domain->read_mutex);
			session_des =
			is_valid_session(domain, &data, NULL, NULL);
			//mutex_lock(&domain->read_mutex);
			if (!session_des) {
				printk_ratelimited(KERN_INFO "data recv invalid session\n");
				bif_del_frame_from_list(&domain->channel,
				frame_tmp);
			} else {
				// at extreme condition,
				// session_des maybe invalid at here
				// but do not make harm
				// if register operation with init
				list_del(&frame_tmp->frame_cache_list);
				//bif_frame_decrease_count(&domain->channel);
				spin_lock(&(session_des->recv_list.lock));
				list_add_tail(&frame_tmp->frame_cache_list,
				&session_des->recv_list.list);
				++session_des->recv_list.frame_count;
				spin_unlock(&(session_des->recv_list.lock));
				up(&session_des->frame_count_sem);
				++domain->domain_statistics.up_sem_count;
			}
		}
	}
	mutex_unlock(&domain->read_mutex);

	return 0;
}
EXPORT_SYMBOL(recv_handle_data_frame);

/*
 * return value:
 * -1: error & no frame get
 * 0: manage frame get
 * 1: othrer frame get
 */
int recv_frame_interrupt(struct comm_domain *domain)
{
	int ret = 0;
	struct bif_frame_cache *frame = NULL;
	struct hbipc_header *header = NULL;
	struct send_mang_data data;
	struct session_desc *session_des = NULL;
	struct list_head *pos = NULL;
	struct bif_frame_cache *frame_tmp = NULL;

	// concede manage frame
	if (domain->manage_send)
		msleep_interruptible(5);

	mutex_lock(&domain->read_mutex);
	++domain->domain_statistics.interrupt_recv_count;
	if ((bif_rx_get_frame(&domain->channel, &frame) < 0)
		|| (!frame)) {
		mutex_unlock(&domain->read_mutex);
		return -1;
	}

	// iterate frame cache list of channel
	// handle every frame get this time
	while (!list_empty(&(domain->channel.rx_frame_cache_p->frame_cache_list))) {
		// when enter while loop every time, read_mutex is locked
		// get the first frame from frame cache list
		pos = domain->channel.rx_frame_cache_p->frame_cache_list.next;
		frame_tmp =
		list_entry(pos, struct bif_frame_cache, frame_cache_list);
		header = (struct hbipc_header *)frame_tmp->framecache;

		if ((header->provider_id == 0) && (header->client_id == 0)) {
			// manage frame
			// just for handle frame link list relation
			++domain->domain_statistics.manage_frame_count;
			list_del(&frame_tmp->frame_cache_list);
			//bif_frame_decrease_count(&domain->channel);
			list_add_tail(&frame_tmp->frame_cache_list,
				&domain->manage_frame_list);
			//mutex_unlock(&domain->read_mutex);

			ret = handle_manage_frame(domain, frame_tmp);
			//mutex_lock(&domain->read_mutex);
			bif_del_frame_from_list(&domain->channel, frame_tmp);
			//list_del(pos);
			//kfree(frame_tmp);
		} else {
			// data frame
			// check session validity and insert data frame
			++domain->domain_statistics.data_frame_count;
			data.domain_id = header->domain_id;
			data.provider_id = header->provider_id;
			data.client_id = header->client_id;

			//mutex_unlock(&domain->read_mutex);
			session_des = is_valid_session(domain, &data, NULL, NULL);
			//mutex_lock(&domain->read_mutex);
			if (!session_des) {
				printk_ratelimited(KERN_INFO "interrupt recv invalid session\n");
				bif_del_frame_from_list(&domain->channel, frame_tmp);
			} else {
				// at extreme condition, session_des maybe invalid at here
				// but do not make harm if register operation with init
				list_del(&frame_tmp->frame_cache_list);
				//bif_frame_decrease_count(&domain->channel);
				spin_lock(&(session_des->recv_list.lock));
				list_add_tail(&frame_tmp->frame_cache_list,
				&session_des->recv_list.list);
				++session_des->recv_list.frame_count;
				spin_unlock(&(session_des->recv_list.lock));
				up(&session_des->frame_count_sem);
				++domain->domain_statistics.up_sem_count;
			}
		}
	}
	mutex_unlock(&domain->read_mutex);

	return 0;
#if 0
	header = (struct hbipc_header *)frame->framecache;

	if ((header->provider_id == 0) && (header->client_id == 0)) {
		// manage frame
		// just for handle frame link list relation
		list_del(&frame->frame_cache_list);
		list_add_tail(&frame->frame_cache_list,
		&domain->manage_frame_list);
		mutex_unlock(&domain->read_mutex);

		ret = handle_manage_frame(domain, frame);
		mutex_lock(&domain->read_mutex);
		bif_del_frame_from_list(&domain->channel, frame);
		mutex_unlock(&domain->read_mutex);
		if (ret < 0)
			return -1;
		else
			return 0;
	} else {
		// data frame
		// check session validity and insert data frame
		data.domain_id = header->domain_id;
		data.provider_id = header->provider_id;
		data.client_id = header->client_id;

		mutex_unlock(&domain->read_mutex);
		session_des = is_valid_session(domain, &data, NULL, NULL);
		mutex_lock(&domain->read_mutex);
		if (!session_des) {
			hbipc_debug("interrupt recv invalid session\n");
			bif_del_frame_from_list(&domain->channel, frame);
		} else {
			list_del(&frame->frame_cache_list);
			spin_lock(&(session_des->recv_list.lock));
			list_add_tail(&frame->frame_cache_list,
			&session_des->recv_list.list);
			++session_des->recv_list.frame_count;
			spin_unlock(&(session_des->recv_list.lock));
			up(&session_des->frame_count_sem);
		}

		mutex_unlock(&domain->read_mutex);

		return 1;
	}
#endif
}
EXPORT_SYMBOL(recv_frame_interrupt);

/*
 * return value:
 * < 0: error code
 * 0: valid session
 * 1: no error but no session
 */
int accept_session(struct comm_domain *domain,
struct send_mang_data *data, struct session_desc **connect)
{
	int ret = 0;
	int index = 0;
	struct server_desc *server = NULL;
	struct provider_desc *provider = NULL;
	struct session_desc *connect_des = NULL;
	int i = 0;

	if (domain->unaccept_session_count > 0) {
		mutex_lock(&domain->connect_mutex);
		index = get_server_index(&domain->server, data->server_id);
		if (index < 0) {
			hbipc_error("invalid server_id:\n");
			for (i = 0; i < UUID_LEN; ++i)
				hbipc_error("%d	", data->server_id[i]);
			hbipc_error("\n");
			ret = HBIPC_ERROR_INVALID_SERVERID;
			mutex_unlock(&domain->connect_mutex);
			goto error;
		}
		server = domain->server.server_array + index;

		index = get_provider_index(&server->provider,
		data->provider_id);
		if (index < 0) {
			hbipc_error("invalid provider_id: %d\n",
			data->provider_id);
			ret = HBIPC_ERROR_INVALID_PROVIDERID;
			mutex_unlock(&domain->connect_mutex);
			goto error;
		}
		provider = server->provider.provider_array + index;

		if (provider->session.count <= 0) {
			//hbipc_debug("no session\n");
			// no session at present, but it's not an error
			mutex_unlock(&domain->connect_mutex);
			return 1;
		}

		// get first non-connected session
		index = get_session_first_nonconnect_index(&provider->session);
		if (index < 0) {
			mutex_unlock(&domain->connect_mutex);
			return 1;
		}
		connect_des = provider->session.session_array + index;
		connect_des->connected = 1;
		*connect = connect_des;
		--domain->unaccept_session_count;
		mutex_unlock(&domain->connect_mutex);

		return 0;
	} else {
		mutex_lock(&domain->connect_mutex);

		index = get_server_index(&domain->server, data->server_id);
		if (index < 0) {
			hbipc_error("invalid server_id:\n");
			for (i = 0; i < UUID_LEN; ++i)
				hbipc_error("%d	", data->server_id[i]);
			hbipc_error("\n");
			ret = HBIPC_ERROR_INVALID_SERVERID;
			mutex_unlock(&domain->connect_mutex);
			goto error;
		}
		server = domain->server.server_array + index;

		index = get_provider_index(&server->provider,
		data->provider_id);
		if (index < 0) {
			hbipc_error("invalid provider_id: %d\n",
			data->provider_id);
			ret = HBIPC_ERROR_INVALID_PROVIDERID;
			mutex_unlock(&domain->connect_mutex);
			goto error;
	}

		mutex_unlock(&domain->connect_mutex);

	return 1;
	}
error:
	return ret;
}
EXPORT_SYMBOL(accept_session);

int bif_lite_init_domain(struct comm_domain *domain)
{
	int ret = 0;

	ret = bif_lite_init(&domain->channel);

	return ret;
}
EXPORT_SYMBOL(bif_lite_init_domain);

void bif_lite_exit_domain(struct comm_domain *domain)
{
	bif_lite_exit(&domain->channel);
}
EXPORT_SYMBOL(bif_lite_exit_domain);

int bif_lite_irq_register_domain(struct comm_domain *domain,
irq_handler_t irq_handler)
{
	int ret = 0;

	ret = bif_lite_register_irq(&domain->channel, irq_handler);

	return ret;
}
EXPORT_SYMBOL(bif_lite_irq_register_domain);

void bif_del_frame_domain(struct comm_domain *domain,
struct bif_frame_cache *frame)
{
	bif_del_frame_from_list(&domain->channel, frame);
}
EXPORT_SYMBOL(bif_del_frame_domain);

void bif_del_session_frame_domain(struct comm_domain *domain,
struct bif_frame_cache *frame)
{
	bif_del_frame_from_session_list(&domain->channel, frame);
}
EXPORT_SYMBOL(bif_del_session_frame_domain);


int bif_tx_put_frame_domain(struct comm_domain *domain, void *data, int len)
{
	int ret = 0;

	ret = bif_tx_put_frame(&domain->channel, data, len);

	return ret;
}
EXPORT_SYMBOL(bif_tx_put_frame_domain);

int domain_stock_frame_num(struct comm_domain *domain)
{
	return channel_stock_frame_num(&domain->channel);
}
EXPORT_SYMBOL(domain_stock_frame_num);

int start_server(struct comm_domain *domain, struct send_mang_data *data)
{
	int ret = 0;
	struct provider_server *relation = NULL;
	struct provider_start_desc *provider_start_des = NULL;

	mutex_lock(&domain->connect_mutex);

	ret = get_map_index_from_server(&domain->map, data);
	// map have been created
	if (ret >= 0) {
		relation = domain->map.map_array + ret;
		ret = get_start_index(&relation->start_list, data->client_id);
		if (ret < 0) {
			// current client didn't start this provider
			if (relation->start_list.count <= 0) {
				data->result = HBIPC_ERROR_RMT_RES_ALLOC_FAIL;
				ret = 0;
			}
			else {
				provider_start_des =
				relation->start_list.start_array +
				relation->start_list.first_avail;
				provider_start_des->valid = 1;
				provider_start_des->client_id =
				data->client_id;
				--relation->start_list.count;
				relation->start_list.first_avail =
				get_start_list_first_avail_index(&relation->start_list);
				data->provider_id = relation->provider_id;
				ret = 0;
			}
		} else {
			// current client has already started this provider
			data->result = HBIPC_ERROR_REPEAT_STARTSERVER;
			ret = 0;
		}
	} else
		data->result = HBIPC_ERROR_INVALID_SERVERID;

	mutex_unlock(&domain->connect_mutex);

	return ret;
}
EXPORT_SYMBOL(start_server);

int stop_server(struct comm_domain *domain, struct send_mang_data *data)
{
	int ret = 0;
	struct provider_server *relation = NULL;
	struct provider_start_desc *provider_start_des = NULL;

	mutex_lock(&domain->connect_mutex);

	ret = get_map_index(&domain->map, data->provider_id);
	if (ret < 0) {
		hbipc_error("invalid provider: %d\n", data->provider_id);
		data->result = HBIPC_ERROR_INVALID_PROVIDERID;
	} else {
		relation = domain->map.map_array + ret;
		ret = get_start_index(&relation->start_list, data->client_id);
		if (ret < 0)
			data->result = HBIPC_ERROR_INVALID_PROVIDERID;
		else {
			provider_start_des =
			relation->start_list.start_array + ret;
			provider_start_des->valid = 0;
			++relation->start_list.count;
			relation->start_list.first_avail =
			get_start_list_first_avail_index(&relation->start_list);
			ret = 0;
		}
	}

	mutex_unlock(&domain->connect_mutex);

	return ret;
}
EXPORT_SYMBOL(stop_server);
