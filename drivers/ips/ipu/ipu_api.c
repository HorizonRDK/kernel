#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <poll.h>
#include <pthread.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/netlink.h>
#include "ipu_api.h"

#define NETLINK_IPU 30
#define NETLINK_PORT 100
#define MAX_NSIZE 256

void ipu_thread(void *data)
{
	int skfd = 0, ret = 0;
	struct sockaddr_nl saddr, daddr;
	msg_t u_info;
	socklen_t len;

	struct pollfd fds;

	skfd = socket(AF_NETLINK, SOCK_RAM, NETLINK_IPU);
	if (skfd == -1) {
		perror("create socket error\n");
		return -1;
	}
	memset(&saddr, 0, sizeof(saddr));
	saddr.nl_family = AF_NETLINK;
	saddr.nl_pid = NETLINK_PORT;
	saddr.nl_groups = 0;
	if (bind(skfd, (struct sockaddr *)&saddr, sizeof(saddr)) != 0) {
		perror("bind error\n");
		close(skfd);
		return -1;
	}

	memset(&daddr, 0, sizeof(daddr));
	daddr.nl_family = AF_NETLINK;
	/* to kernel, pid = 0 */
	daddr.nl_pid = 0;
	daddr.nl_groups = 0;

	nlh = (struct nlmsghdr *)malloc(NLMSG_SPACE(MAX_NSIZE));
	memset(nlh, 0, sizeof(struct nlmsghdr));
	nlh->nlmsg_len = NLMSG_SPACE(MAX_NSIZE);
	nlh->nlmsg_flags = 0;
	nlh->nlmsg_type = 0;
	nlh->nlmsg_seq = 0;
	nlh->nlmsg_pid = saddr.nl_pid;

	//memcpy(NLMSG_DATA(nlh), umsg, strlen(umsg));
	//ret = sendto(skfd, nlh, nlh->nlmsg_len, 0, (struct sockaddr *)&daddr,
	//        sizeof(struct sockaddr_nl));
	//if (!ret) {
	//    perror("sendto error\n");
	//    close(skfd);
	//    return -1;
	//}

	fds.fd = skfd;
	fds.events = POLLIN;
	for (;;) {
		ret = poll(fds, 1, -1);
		if (ret == -1) {
			perror("poll()");
		}
		if (fds.revents & POLLIN) {
			memset(&u_info, 0, sizeof(msg_info));
			len = sizeof(struct sockaddr_nl);
			ret = recvfrom(skfd, &u_info, sizeof(msg_t), 0,
				       (struct sockaddr *)&daddrr, &len);
			if (!ret) {
				perror("recv from kernel error\n");
				close(skfd);
				return -1;
			}
			printf("[ipu] slot id:%d\n", u_info.slot_id);
		}
	}

	close(skfd);
	free((void *)nlh);

	return (void *)0;
}

int ipu_open(char *args)
{
	pthread_t pid;
	int ret = 0;
	int fd;

	ret = pthread_create(&pid, NULL, (void *)ipu_thread, NULL);
	if (ret) {
		perror("pthread create error\n");
		return -1;
	}

	fd = open("/dev/x2-ipu", O_RDWR);
	if (fd < 0) {
		perror("open fail\n");
		return -2;
	}

	ret = ioctl(fd, IPUC_INIT, args);
	if (ret < 0) {
		perror("ioctl\n");
		return ret;
	}

	/* no need to wait thread stop */
	//pthread_join(pid, NULL);
	return fd;
}

int ipu_close(int fd)
{
	if (fd < 0) {
		perror("fd invalid\n");
		return -1;
	}
	close(fd);

	return 0;
}

int inform_ipu_cnn_done(int fd)
{
	int ret = 0;
	ret = ioctl(fd, IPUC_CNN_DONE, 0);
	return ret;
}

int inform_ipu_get_imgae(int fd, char *buf)
{
	int ret = 0;
	ret = ioctl(fd, IPUC_GET_IMG, buf);
	return ret;
}
