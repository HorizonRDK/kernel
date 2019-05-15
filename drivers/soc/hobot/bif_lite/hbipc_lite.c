#include <linux/list.h>
#include <linux/interrupt.h>

#include "hbipc_lite.h"
#include "bif_lite_utility.h"
#include "hbipc_errno.h"

struct domain_info domain_config = {"X2BIF001", 0, "/dev/x2_bif"};
struct comm_domain domain;
int session_count;
int unaccept_session_count;

void resource_queue_init(struct resource_queue *queue)
{
	INIT_LIST_HEAD(&queue->list);
	queue->frame_count = 0;
	spin_lock_init(&queue->lock);
}

void resource_queue_deinit(struct resource_queue *queue)
{
	// need remove frame
	struct list_head *pos = NULL;
	struct list_head *n = NULL;
	struct bif_frame_cache *bif_frame = NULL;

	spin_lock(&queue->lock);
	list_for_each_safe(pos, n, &queue->list) {
		bif_frame =
		list_entry(pos, struct bif_frame_cache, frame_cache_list);
		bif_del_frame_from_list(bif_frame);
		--queue->frame_count;
	}
	spin_unlock(&queue->lock);
}

void session_desc_init(struct session_desc *session_des)
{
	session_des->valid = 0;
	session_des->connected = 0;
	session_des->domain_id = -1;
	session_des->provider_id = -1;
	session_des->client_id = -1;
	resource_queue_init(&session_des->recv_list);
	sema_init(&session_des->frame_count_sem, 0);
}

void session_desc_deinit(struct session_desc *session_des)
{
	session_des->valid = 0;
	session_des->connected = 0;
	session_des->domain_id = -1;
	session_des->provider_id = -1;
	session_des->client_id = -1;
	resource_queue_deinit(&session_des->recv_list);
}

void session_info_init(struct session_info *session_inf)
{
	int i = 0;

	for (i = 0; i < SESSION_COUNT_MAX; ++i)
		session_desc_init(session_inf->session_array + i);
	session_inf->count = SESSION_COUNT_MAX;
	session_inf->first_avail = 0;
}

void session_info_deinit(struct session_info *session_inf)
{
	int i = 0;

	for (i = 0; i < SESSION_COUNT_MAX; ++i)
		session_desc_deinit(session_inf->session_array + i);
	session_inf->count = 0;
	session_inf->first_avail = 0;
}

void provider_desc_init(struct provider_desc *provider_des)
{
	provider_des->valid = 0;
	provider_des->provider_id = 0;
	session_info_init(&provider_des->session);
}

void provider_desc_deinit(struct provider_desc *provider_des)
{
	provider_des->valid = 0;
	provider_des->provider_id = 0;
	session_info_deinit(&provider_des->session);
}

void provider_info_init(struct provider_info *provider_inf)
{
	int i = 0;

	for (i = 0; i < PROVIDER_COUNT_MAX; ++i)
		provider_desc_init(provider_inf->provider_array + i);
	provider_inf->count = PROVIDER_COUNT_MAX;
	provider_inf->first_avail = 0;
}

void provider_info_deinit(struct provider_info *provider_inf)
{
	int i = 0;

	for (i = 0; i < PROVIDER_COUNT_MAX; ++i)
		provider_desc_deinit(provider_inf->provider_array + i);
	provider_inf->count = 0;
	provider_inf->first_avail = 0;
}

void server_desc_init(struct server_desc *server_des)
{
	server_des->valid = 0;
	memset(server_des->server_id, 0, UUID_LEN);
	provider_info_init(&server_des->provider);
}

void server_desc_deinit(struct server_desc *server_des)
{
	server_des->valid = 0;
	memset(server_des->server_id, 0, UUID_LEN);
	provider_info_deinit(&server_des->provider);
}

void server_info_init(struct server_info *server_inf)
{
	int i = 0;

	for (i = 0; i < SERVER_COUNT_MAX; ++i)
		server_desc_init(server_inf->server_array + i);
	server_inf->count = SERVER_COUNT_MAX;
	server_inf->first_avail = 0;
}

void server_info_deinit(struct server_info *server_inf)
{
	int i = 0;

	for (i = 0; i < SERVER_COUNT_MAX; ++i)
		server_desc_deinit(server_inf->server_array + i);
	server_inf->count = 0;
	server_inf->first_avail = 0;
}

void provider_start_desc_init(struct provider_start_desc *start_des)
{
	start_des->valid = 0;
	start_des->client_id = 0;
}

void provider_start_desc_deinit(struct provider_start_desc *start_des)
{
	start_des->valid = 0;
	start_des->client_id = 0;
}

void provider_start_info_init(struct provider_start_info *start_info)
{
	int i = 0;

	for (i = 0; i < PROVIDER_START_COUNT_MAX; ++i)
		provider_start_desc_init(start_info->start_array + i);
	start_info->count = PROVIDER_START_COUNT_MAX;
	start_info->first_avail = 0;
}

void provider_start_info_deinit(struct provider_start_info *start_info)
{
	int i = 0;

	for (i = 0; i < PROVIDER_START_COUNT_MAX; ++i)
		provider_start_desc_init(start_info->start_array + i);
	start_info->count = 0;
	start_info->first_avail = 0;
}

void provider_server_init(struct provider_server *relation)
{
	relation->valid = 0;
	relation->provider_id = -1;
	memset(relation->server_id, 0, UUID_LEN);
	provider_start_info_init(&relation->start_list);
}

void provider_server_deinit(struct provider_server *relation)
{
	relation->valid = 0;
	relation->provider_id = -1;
	memset(relation->server_id, 0, UUID_LEN);
	provider_start_info_deinit(&relation->start_list);
}

void provider_server_map_init(struct provider_server_map *map)
{
	int i = 0;

	for (i = 0; i < PROVIDER_SERVER_MAP_COUNT; ++i)
		provider_server_init(map->map_array + i);
	map->count = PROVIDER_SERVER_MAP_COUNT;
	map->first_avail = 0;
}

void provider_server_map_deinit(struct provider_server_map *map)
{
	int i = 0;

	for (i = 0; i < PROVIDER_SERVER_MAP_COUNT; ++i)
		provider_server_deinit(map->map_array + i);
	map->count = 0;
	map->first_avail = 0;
}


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

	return 0;
}

void domain_deinit(struct comm_domain *domain)
{
	mutex_destroy(&domain->write_mutex);
	mutex_destroy(&domain->read_mutex);
	mutex_destroy(&domain->connect_mutex);
	domain->domain_name = NULL;
	domain->domain_id = -1;
	domain->device_name = NULL;
	server_info_deinit(&domain->server);
	provider_server_map_deinit(&domain->map);
	// had better free manage frame
	INIT_LIST_HEAD(&domain->manage_frame_list);
}

int get_session_first_avail_index(struct session_info *session_inf)
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

int get_session_first_nonconnect_index(struct session_info *session_inf)
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

int get_provider_first_avail_index(struct provider_info *provider_inf)
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

int get_server_first_avail_index(struct server_info *server_inf)
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

int get_server_index(struct server_info *server_inf, unsigned char *server_id)
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

int get_provider_index(struct provider_info *provider_inf, int provider_id)
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

int get_match_provider_index(struct provider_info *provider_inf,
char *parameter)
{
	int i = 0;

	// currently, we return the first valid provider index
	// in future, we will match provider with parameters
	for (i = 0; i < PROVIDER_COUNT_MAX; ++i) {
		if (provider_inf->provider_array[i].valid)
			break;
	}

	if (i < PROVIDER_COUNT_MAX)
		return i;
	else
		return -1;
}

int get_session_index(struct session_info *session_inf, struct session *connect)
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

int get_map_first_avail_index(struct provider_server_map *map)
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

int get_map_index_with_lock(struct provider_server_map *map, int provider_id)
{
	int i = 0;

	mutex_lock(&domain.connect_mutex);
	for (i = 0; i < PROVIDER_SERVER_MAP_COUNT; ++i)
		if ((map->map_array[i].valid) &&
	(map->map_array[i].provider_id == provider_id))
			break;
	mutex_unlock(&domain.connect_mutex);

	if (i < PROVIDER_SERVER_MAP_COUNT)
		return i;
	else
		return -1;
}

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

struct session_desc *is_valid_session(struct send_mang_data *data,
struct server_desc **server_des, struct provider_desc **provider_des)
{
	int index = 0;
	struct server_desc *server = NULL;
	struct provider_desc *provider = NULL;
	struct provider_server *relation = NULL;
	struct session_desc *session_des = NULL;
	struct session connect;

	mutex_lock(&domain.connect_mutex);
	index = get_map_index(&domain.map, data->provider_id);
	if (index < 0)
		goto error;
	relation = domain.map.map_array + index;

	index = get_server_index(&domain.server, relation->server_id);
	if (index < 0)
		goto error;
	server = domain.server.server_array + index;

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

	mutex_unlock(&domain.connect_mutex);

	return session_des;
error:
	mutex_unlock(&domain.connect_mutex);
	return NULL;
}

int mang_frame_send2opposite(int type, struct send_mang_data *data)
{
	char mang_frame_send_buf[ALIGN((HBIPC_HEADER_LEN + MANAGE_MSG_LEN), BIFSPI_LEN_ALIGN)];
	struct hbipc_header *header =
	(struct hbipc_header *)mang_frame_send_buf;
	struct manage_message *message =
	(struct manage_message *)(mang_frame_send_buf +
	HBIPC_HEADER_LEN);
	int *p = (int *)(message->msg_text);

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
	default:
		goto error;
	}

	mutex_lock(&domain.write_mutex);

	if (bif_tx_put_frame(mang_frame_send_buf, HBIPC_HEADER_LEN +
	MANAGE_MSG_LEN) < 0) {
		hbipc_error("bif_tx_put_frame error\n");
		mutex_unlock(&domain.write_mutex);
		goto error;
	}

	mutex_unlock(&domain.write_mutex);

	return 0;
error:
	return -1;
}

int regisger_server(struct send_mang_data *data)
{
	struct server_desc *server = NULL;
	int ret = 0;

	// register server_id just first time
	if (get_server_index(&domain.server, data->server_id) < 0) {
		if (domain.server.count <= 0) {
			hbipc_error("server resource insufficient\n");
			ret = HBIPC_ERROR_RMT_RES_ALLOC_FAIL;
			goto error;
		} else {
			server = domain.server.server_array +
			domain.server.first_avail;
			server->valid = 1;
			memcpy(server->server_id, data->server_id, UUID_LEN);
			--domain.server.count;
			domain.server.first_avail =
			get_server_first_avail_index(&domain.server);
		}
	}

	hbipc_debug("server_info: count = %d first_avail = %d\n",
	domain.server.count, domain.server.first_avail);

	return 0;
error:
	return ret;
}

int unregister_server(struct send_mang_data *data)
{
	int index = 0;
	struct server_desc *server = NULL;

	index = get_server_index(&domain.server, data->server_id);
	server = domain.server.server_array + index;

	// unregister server only no provider exist
	if (server->provider.count == PROVIDER_COUNT_MAX) {
		server->valid = 0;
		++domain.server.count;
		domain.server.first_avail =
		get_server_first_avail_index(&domain.server);
	}

	hbipc_debug("server_info: count = %d first_avail = %d\n",
	domain.server.count, domain.server.first_avail);

	return 0;
}

int register_provider(struct send_mang_data *data)
{
	int index = 0;
	struct server_desc *server = NULL;
	struct provider_desc *provider = NULL;
	int ret = 0;

	index = get_server_index(&domain.server, data->server_id);
	server = domain.server.server_array + index;

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

int clear_connect(struct session_info *session_inf)
{
	int i = 0;

	for (i = 0; i < SESSION_COUNT_MAX; ++i) {
		if (session_inf->session_array[i].valid) {
			session_inf->session_array[i].valid = 0;
#ifdef CONFIG_HOBOT_BIF_AP
			session_inf->session_array[i].connected = 0;
#else
			if (!session_inf->session_array[i].connected)
				--unaccept_session_count;
			else
				session_inf->session_array[i].connected = 0;
#endif
	resource_queue_deinit(&(session_inf->session_array[i].recv_list));

			++session_inf->count;
			session_inf->first_avail =
			get_session_first_avail_index(session_inf);
			--session_count;
#ifdef CONFIG_HOBOT_BIF_AP
			up(&(session_inf->session_array[i].frame_count_sem));
#endif
		}
	}

	hbipc_debug("session_info: count = %d first_avail = %d session_count = %d unaccept_session_count = %d\n",
	session_inf->count, session_inf->first_avail,
	session_count, unaccept_session_count);

	return 0;
}

int unregister_provider(struct send_mang_data *data)
{
	int index = 0;
	struct server_desc *server = NULL;
	struct provider_desc *provider = NULL;
	int ret = 0;

	index = get_server_index(&domain.server, data->server_id);
	if (index < 0) {
		ret = HBIPC_ERROR_INVALID_SERVERID;
		hbipc_error("invalid server_id");
		goto error;
	}
	server = domain.server.server_array + index;

	index = get_provider_index(&server->provider, data->provider_id);
	if (index < 0) {
		ret = HBIPC_ERROR_INVALID_PROVIDERID;
		hbipc_error("invalid provider_id\n");
		goto error;
	} else {
		provider = server->provider.provider_array + index;
		provider->valid = 0;
		clear_connect(&provider->session);
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

int register_map(struct send_mang_data *data)
{
	struct provider_server *relation = NULL;

	relation = domain.map.map_array + domain.map.first_avail;
	relation->valid = 1;
	memcpy(relation->server_id, data->server_id, UUID_LEN);
	relation->provider_id = data->provider_id;
	provider_start_info_init(&relation->start_list);
	--domain.map.count;
	domain.map.first_avail = get_map_first_avail_index(&domain.map);

	hbipc_debug("provider_server_map: count = %d first_avail = %d\r\n",
	domain.map.count, domain.map.first_avail);

	return 0;
}

int register_map_with_lock(struct send_mang_data *data)
{
	struct provider_server *relation = NULL;

	mutex_lock(&domain.connect_mutex);

	relation = domain.map.map_array + domain.map.first_avail;
	relation->valid = 1;
	memcpy(relation->server_id, data->server_id, UUID_LEN);
	relation->provider_id = data->provider_id;
	provider_start_info_init(&relation->start_list);
	--domain.map.count;
	domain.map.first_avail = get_map_first_avail_index(&domain.map);

	mutex_unlock(&domain.connect_mutex);

	hbipc_debug("provider_server_map: count = %d first_avail = %d\r\n",
	domain.map.count, domain.map.first_avail);

	return 0;
}

int unregister_map(struct send_mang_data *data)
{
	int index = 0;
	struct provider_server *relation = NULL;

	index = get_map_index(&domain.map, data->provider_id);
	relation = domain.map.map_array + index;
	relation->valid = 0;
	provider_start_info_deinit(&relation->start_list);
	++domain.map.count;
	domain.map.first_avail = get_map_first_avail_index(&domain.map);

	hbipc_debug("provider_server_map: count = %d first_avail = %d\r\n",
	domain.map.count, domain.map.first_avail);

	return 0;
}

int unregister_map_with_lock(struct send_mang_data *data)
{
	int index = 0;
	struct provider_server *relation = NULL;

	mutex_lock(&domain.connect_mutex);

	index = get_map_index(&domain.map, data->provider_id);
	relation = domain.map.map_array + index;
	relation->valid = 0;
	provider_start_info_deinit(&relation->start_list);
	++domain.map.count;
	domain.map.first_avail = get_map_first_avail_index(&domain.map);

	mutex_unlock(&domain.connect_mutex);

	hbipc_debug("provider_server_map: count = %d first_avail = %d\r\n",
	domain.map.count, domain.map.first_avail);

	return 0;
}

int register_server_provider(struct send_mang_data *data)
{
	int ret = 0;

	mutex_lock(&domain.connect_mutex);
	ret = regisger_server(data);
	if (ret < 0) {
		hbipc_error("reisger_server error\n");
		goto register_server_error;
	}

	ret = register_provider(data);
	if (ret < 0) {
		hbipc_error("register_provider error\n");
		goto register_provider_error;
	}

	register_map(data);

	mutex_unlock(&domain.connect_mutex);
#ifndef CONFIG_HOBOT_BIF_AP
	// send register provider datagram
	ret = mang_frame_send2opposite(MANAGE_CMD_REGISTER_PROVIDER, data);
	if (ret < 0) {
		ret = HBIPC_ERROR_HW_TRANS_ERROR;
		hbipc_error("send MANG_CMD_REGISTER_PROVIDER error\n");
		goto send_cmd_error;
	}
#endif

	return 0;
#ifndef CONFIG_HOBOT_BIF_AP
send_cmd_error:
	mutex_lock(&domain.connect_mutex);
	unregister_map(data);
	unregister_provider(data);
#endif
register_provider_error:
	unregister_server(data);
register_server_error:
	mutex_unlock(&domain.connect_mutex);
	return ret;
}

int unregister_server_provider(struct send_mang_data *data)
{
	int ret = 0;

	mutex_lock(&domain.connect_mutex);
	ret = unregister_provider(data);
	if (ret < 0) {
		hbipc_error("unregister_provider error\n");
		mutex_unlock(&domain.connect_mutex);
		goto error;
	}

	unregister_server(data);
	unregister_map(data);

	mutex_unlock(&domain.connect_mutex);

#ifndef CONFIG_HOBOT_BIF_AP
	// send unregister provider datagram
	ret = mang_frame_send2opposite(MANAGE_CMD_UNREGISTER_PROVIDER, data);
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

int unregister_server_provider_abnormal(struct send_mang_data *data)
{
	int ret = 0;
	struct provider_server *relation = NULL;

	mutex_lock(&domain.connect_mutex);
	ret = get_map_index(&domain.map, data->provider_id);
	if (ret >= 0) {
		// abnormal shutdown
		relation = domain.map.map_array + ret;
		memcpy(data->server_id, relation->server_id, UUID_LEN);
	} else {
		mutex_unlock(&domain.connect_mutex);
		return 0;
	}

	ret = unregister_provider(data);
	if (ret < 0) {
		hbipc_error("unregister_provider error\n");
		mutex_unlock(&domain.connect_mutex);
		goto error;
	}

	unregister_server(data);
	unregister_map(data);

	mutex_unlock(&domain.connect_mutex);

	// send unregister provider datagram
	ret = mang_frame_send2opposite(MANAGE_CMD_UNREGISTER_PROVIDER, data);
	if (ret < 0) {
		ret = HBIPC_ERROR_HW_TRANS_ERROR;
		hbipc_error("send MANG_CMD_UNREGISTER_PROVIDER error\n");
		goto error;
	}

	return 0;
error:
	return ret;
}

int disconnect_stopserver_abnormal(struct send_mang_data *data)
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

	mutex_lock(&domain.connect_mutex);

	// disconnect abnormal connect
	for (i = 0; i < SERVER_COUNT_MAX; ++i) {
		server = domain.server.server_array + i;
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
					ret = get_map_index(&domain.map,
					connect->provider_id);
					if (ret < 0)
						continue;
					else {
						relation =
						domain.map.map_array + ret;
						memcpy(data->server_id,
						relation->server_id, UUID_LEN);
					}
					data->provider_id =
					connect->provider_id;
					connect->valid = 0;
					connect->connected = 0;
			resource_queue_deinit(&connect->recv_list);
					++provider->session.count;
					provider->session.first_avail =
			get_session_first_avail_index(&provider->session);
					--session_count;
					mutex_unlock(&domain.connect_mutex);
		mang_frame_send2opposite(MANAGE_CMD_DISCONNECT_REQ, data);
					mutex_lock(&domain.connect_mutex);
				}
			}
		}
	}
		}
	}

	// stop abnormal server
	for (i = 0; i < PROVIDER_SERVER_MAP_COUNT; ++i) {
		relation = domain.map.map_array + i;
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

	mutex_unlock(&domain.connect_mutex);

	return 0;
}

int register_connect(struct send_mang_data *data)
{
	int ret = 0;
	int index = 0;
	struct server_desc *server = NULL;
	struct provider_desc *provider = NULL;
	struct session_desc *connect = NULL;
#ifdef CONFIG_HOBOT_BIF_AP
	struct provider_server *relation = NULL;
#endif

	mutex_lock(&domain.connect_mutex);
#ifdef CONFIG_HOBOT_BIF_AP
	ret = get_map_index(&domain.map, data->provider_id);
	if (ret < 0) {
		hbipc_error("invalid provider\n");
		ret = HBIPC_ERROR_INVALID_PROVIDERID;
		goto error;
	} else {
		relation = domain.map.map_array + ret;
		memcpy(data->server_id, relation->server_id, UUID_LEN);
	}

	ret = get_start_index(&relation->start_list, data->client_id);
	if (ret < 0) {
		ret = HBIPC_ERROR_INVALID_PROVIDERID;
		goto error;
	}
#endif
	index = get_server_index(&domain.server, data->server_id);
	if (index < 0) {
		hbipc_error("invalid server_id\n");
		ret = HBIPC_ERROR_INVALID_SERVERID;
		goto error;
	}
	server = domain.server.server_array + index;

	index = get_provider_index(&server->provider, data->provider_id);
	if (index < 0) {
		hbipc_error("invalid provider_id\n");
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
	++session_count;
#ifndef CONFIG_HOBOT_BIF_AP
	++unaccept_session_count;
#endif
	mutex_unlock(&domain.connect_mutex);
#ifdef CONFIG_HOBOT_BIF_AP
	ret = mang_frame_send2opposite(MANAGE_CMD_CONNECT_REQ, data);
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
	mutex_lock(&domain.connect_mutex);
	connect->valid = 0;
	connect->connected = 0;
	++provider->session.count;
	provider->session.first_avail =
	get_session_first_avail_index(&provider->session);
	--session_count;
#endif
error:
	mutex_unlock(&domain.connect_mutex);
	return ret;
}

int unregister_connect(struct send_mang_data *data)
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

	mutex_lock(&domain.connect_mutex);
#ifdef CONFIG_HOBOT_BIF_AP
	ret = get_map_index(&domain.map, data->provider_id);
	if (ret < 0) {
		hbipc_error("invalid provider\n");
		ret = HBIPC_ERROR_INVALID_PROVIDERID;
		goto error;
	} else {
		relation = domain.map.map_array + ret;
		memcpy(data->server_id, relation->server_id, UUID_LEN);
	}
#endif
	index = get_server_index(&domain.server, data->server_id);
	if (index < 0) {
		hbipc_error("invalid server_id\n");
		ret = HBIPC_ERROR_INVALID_SERVERID;
		goto error;
	}
	server = domain.server.server_array + index;

	index = get_provider_index(&server->provider, data->provider_id);
	if (index < 0) {
		hbipc_error("invalid provider_id\n");
		ret = HBIPC_ERROR_INVALID_PROVIDERID;
		goto error;
	}
	provider = server->provider.provider_array + index;

	connect.domain_id = data->domain_id;
	connect.provider_id = data->provider_id;
	connect.client_id = data->client_id;
	index = get_session_index(&provider->session, &connect);
	if (index < 0) {
		hbipc_error("invalid session\n");
		ret = HBIPC_ERROR_INVALID_SESSION;
		goto error;
	}
	connect_des = provider->session.session_array + index;

	connect_des->valid = 0;
	connect_des->connected = 0;
	resource_queue_deinit(&connect_des->recv_list);
	++provider->session.count;
	provider->session.first_avail =
	get_session_first_avail_index(&provider->session);
	--session_count;
#ifndef CONFIG_HOBOT_BIF_AP
	up(&connect_des->frame_count_sem);
#endif
	mutex_unlock(&domain.connect_mutex);
#ifdef CONFIG_HOBOT_BIF_AP
	ret = mang_frame_send2opposite(MANAGE_CMD_DISCONNECT_REQ, data);
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
	mutex_lock(&domain.connect_mutex);
#endif
error:
	mutex_unlock(&domain.connect_mutex);
	return ret;
}

int handle_manage_frame(struct bif_frame_cache *frame)
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

		if (register_connect(&data) < 0)
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

		if (unregister_connect(&data) < 0)
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

		if (register_server_provider(&data) < 0)
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

		if (unregister_server_provider(&data) < 0)
			goto error;
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
int recv_handle_manage_frame(void)
{
	int ret = 0;
	struct bif_frame_cache *frame = NULL;
	struct hbipc_header *header = NULL;
	struct send_mang_data data;
	struct session_desc *session_des = NULL;

	mutex_lock(&domain.read_mutex);
	if ((bif_rx_get_frame(&frame) < 0) || (!frame)) {
		mutex_unlock(&domain.read_mutex);
		return -1;
	}

	header = (struct hbipc_header *)frame->framecache;

	if ((header->provider_id == 0) && (header->client_id == 0)) {
		// manage frame
		// just for handle frame link list relation
		list_del(&frame->frame_cache_list);
		list_add_tail(&frame->frame_cache_list,
		&domain.manage_frame_list);
		mutex_unlock(&domain.read_mutex);

		ret = handle_manage_frame(frame);
		mutex_lock(&domain.read_mutex);
		bif_del_frame_from_list(frame);
		mutex_unlock(&domain.read_mutex);
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

		mutex_unlock(&domain.read_mutex);
		session_des = is_valid_session(&data, NULL, NULL);
		mutex_lock(&domain.read_mutex);
		if (!session_des) {
			hbipc_debug("mang recv invalid session\n");
			bif_del_frame_from_list(frame);
		} else {
			list_del(&frame->frame_cache_list);
			list_add_tail(&frame->frame_cache_list,
			&session_des->recv_list.list);
			++session_des->recv_list.frame_count;
		}

		mutex_unlock(&domain.read_mutex);

		return 1;
	}
}

int recv_handle_stock_frame(void)
{
	struct bif_frame_cache *frame = NULL;
	struct hbipc_header *header = NULL;

	mutex_lock(&domain.read_mutex);
	while (!bif_rx_get_stock_frame(&frame)) {
		header = (struct hbipc_header *)frame->framecache;

		if ((header->provider_id == 0) && (header->client_id == 0)) {
			// manage frame
			// just for handle frame link list relation
			list_del(&frame->frame_cache_list);
			list_add_tail(&frame->frame_cache_list,
			&domain.manage_frame_list);

			mutex_unlock(&domain.read_mutex);
			handle_manage_frame(frame);
			mutex_lock(&domain.read_mutex);
			bif_del_frame_from_list(frame);
		} else {
			// data frame
			bif_del_frame_from_list(frame);
		}
	}
	mutex_unlock(&domain.read_mutex);

	return 0;
}

/*
 * return value:
 * -1: error & no frame get
 * 0: specific frame get
 * 1: othrer frame get
 */
int recv_handle_data_frame(struct session_desc *session_des,
struct bif_frame_cache **frame)
{
	int ret = 0;
	struct list_head *pos = NULL;
	struct bif_frame_cache *frame_tmp = NULL;
	struct hbipc_header *header = NULL;
	struct send_mang_data data;
	struct session_desc *session_des_tmp = NULL;

	// get stock frame
	if (session_des->recv_list.frame_count > 0) {
		pos = session_des->recv_list.list.next;
		*frame = list_entry(pos, struct bif_frame_cache,
		frame_cache_list);
		// don't modify frame count here, but when real consume
		//--session_des->recv_list.frame_count;
		return 0;
	}

	mutex_lock(&domain.read_mutex);

	if ((bif_rx_get_frame(&frame_tmp) < 0) || (!frame_tmp)) {
		mutex_unlock(&domain.read_mutex);
		return -1;
	}

	header = (struct hbipc_header *)frame_tmp->framecache;
	if ((header->domain_id == session_des->domain_id) &&
		(header->provider_id == session_des->provider_id) &&
		(header->client_id == session_des->client_id)) {
		// specific data frame
		list_del(&frame_tmp->frame_cache_list);
		list_add_tail(&frame_tmp->frame_cache_list,
		&(session_des->recv_list.list));
		*frame = frame_tmp;
		++session_des->recv_list.frame_count;
		mutex_unlock(&domain.read_mutex);

		return 0;
	} else if ((header->provider_id == 0) && (header->client_id == 0)) {
		// manage frame
		// just for handle frame link list relation
		list_del(&frame_tmp->frame_cache_list);
		list_add_tail(&frame_tmp->frame_cache_list,
		&domain.manage_frame_list);
		mutex_unlock(&domain.read_mutex);

		ret = handle_manage_frame(frame_tmp);
		mutex_lock(&domain.read_mutex);
		bif_del_frame_from_list(frame_tmp);
		mutex_unlock(&domain.read_mutex);
		if (ret < 0)
			return -1;
		else
			return 1;
	} else {
		// othrer data frame
		data.domain_id = header->domain_id;
		data.provider_id = header->provider_id;
		data.client_id = header->client_id;

		mutex_unlock(&domain.read_mutex);
		session_des_tmp = is_valid_session(&data, NULL, NULL);
		mutex_lock(&domain.read_mutex);
		if (session_des_tmp) {
			list_del(&frame_tmp->frame_cache_list);
			list_add_tail(&frame_tmp->frame_cache_list,
			&session_des_tmp->recv_list.list);
			++session_des_tmp->recv_list.frame_count;
		} else {
			hbipc_debug("data recv invalid session\n");
			bif_del_frame_from_list(frame_tmp);
		}
		mutex_unlock(&domain.read_mutex);

		return 1;
	}
}

/*
 * return value:
 * -1: error & no frame get
 * 0: manage frame get
 * 1: othrer frame get
 */
int recv_frame_interrupt(void)
{
	int ret = 0;
	struct bif_frame_cache *frame = NULL;
	struct hbipc_header *header = NULL;
	struct send_mang_data data;
	struct session_desc *session_des = NULL;

	mutex_lock(&domain.read_mutex);
	if ((bif_rx_get_frame(&frame) < 0) || (!frame)) {
		mutex_unlock(&domain.read_mutex);
		return -1;
	}

	header = (struct hbipc_header *)frame->framecache;

	if ((header->provider_id == 0) && (header->client_id == 0)) {
		// manage frame
		// just for handle frame link list relation
		list_del(&frame->frame_cache_list);
		list_add_tail(&frame->frame_cache_list,
		&domain.manage_frame_list);
		mutex_unlock(&domain.read_mutex);

		ret = handle_manage_frame(frame);
		mutex_lock(&domain.read_mutex);
		bif_del_frame_from_list(frame);
		mutex_unlock(&domain.read_mutex);
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

		mutex_unlock(&domain.read_mutex);
		session_des = is_valid_session(&data, NULL, NULL);
		mutex_lock(&domain.read_mutex);
		if (!session_des) {
			hbipc_debug("interrupt recv invalid session\n");
			bif_del_frame_from_list(frame);
		} else {
			list_del(&frame->frame_cache_list);
			spin_lock(&(session_des->recv_list.lock));
			list_add_tail(&frame->frame_cache_list,
			&session_des->recv_list.list);
			++session_des->recv_list.frame_count;
			spin_unlock(&(session_des->recv_list.lock));
			up(&session_des->frame_count_sem);
		}

		mutex_unlock(&domain.read_mutex);

		return 1;
	}
}

/*
 * return value:
 * < 0: error code
 * 0: valid session
 * 1: no error but no session
 */
int accept_session(struct send_mang_data *data, struct session_desc **connect)
{
	int ret = 0;
	int index = 0;
	struct server_desc *server = NULL;
	struct provider_desc *provider = NULL;
	struct session_desc *connect_des = NULL;

	//recv_handle_manage_frame();

	if (unaccept_session_count > 0) {
		mutex_lock(&domain.connect_mutex);
		index = get_server_index(&domain.server, data->server_id);
		if (index < 0) {
			hbipc_error("invalid server_id\n");
			ret = HBIPC_ERROR_INVALID_SERVERID;
			mutex_unlock(&domain.connect_mutex);
			goto error;
		}
		server = domain.server.server_array + index;

		index = get_provider_index(&server->provider,
		data->provider_id);
		if (index < 0) {
			hbipc_error("invalid provider_id\n");
			ret = HBIPC_ERROR_INVALID_PROVIDERID;
			mutex_unlock(&domain.connect_mutex);
			goto error;
		}
		provider = server->provider.provider_array + index;

		if (provider->session.count <= 0) {
			//hbipc_debug("no session\n");
			// no session at present, but it's not an error
			mutex_unlock(&domain.connect_mutex);
			return 1;
		}

		// get first non-connected session
		index = get_session_first_nonconnect_index(&provider->session);
		if (index < 0) {
			mutex_unlock(&domain.connect_mutex);
			return 1;
		}
		connect_des = provider->session.session_array + index;
		connect_des->connected = 1;
		*connect = connect_des;
		--unaccept_session_count;
		mutex_unlock(&domain.connect_mutex);

		return 0;
	}

	return 1;
error:
	return ret;
}

/*
 * return value:
 * > 0: valid provider id
 * -1: no valid provider id
 */
int start_server(struct send_mang_data *data, int *provider_id)
{
	int index = 0;
	struct server_desc *server = NULL;
	struct provider_desc *provider = NULL;

	recv_handle_manage_frame();

	mutex_lock(&domain.connect_mutex);
	index = get_server_index(&domain.server, data->server_id);
	if (index < 0) {
		mutex_unlock(&domain.connect_mutex);
		goto error;
	} else
		server = domain.server.server_array + index;

	index = get_match_provider_index(&server->provider, NULL);
	if (index < 0) {
		mutex_unlock(&domain.connect_mutex);
		goto error;
	} else
		provider = server->provider.provider_array + index;
	mutex_unlock(&domain.connect_mutex);

	return provider->provider_id;
error:
	return -1;
}

irqreturn_t hbipc_irq_handler(int irq, void *data)
{
	recv_frame_interrupt();

	return IRQ_HANDLED;
}
