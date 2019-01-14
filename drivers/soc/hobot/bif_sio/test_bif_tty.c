#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include<stdio.h>		/*标准输入输出定义 */
#include<stdlib.h>		/*标准函数库定义 */
#include<unistd.h>		/*Unix 标准函数定义 */
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>		/*文件控制定义 */
#include<termios.h>		/*PPSIX 终端控制定义 */
#include<errno.h>		/*错误号定义 */
#include<string.h>
#include<time.h>

int main(void)
{

	int gm_fd;
	int i;
	char buffer[1024] = { 0 };
	char file_name[20] = { 0 };
	char *p;
	int len;
	int speed_arr[] = { B115200, B19200, B9600, B4800, B2400, B1200, B300 };
	int name_arr[] = { 115200, 19200, 9600, 4800, 2400, 1200, 300 };
	struct termios options;
	time_t timep;

	for (i = 0; i < 4; i++) {

		printf("test ttyBif module ....\n");
		sprintf(file_name, "/dev/ttyBIF%d", i);
		gm_fd = open(file_name, O_RDWR, 0777);
		if (gm_fd == -1) {
			perror("can not open /dev/ttyBif ... ");
			return -1;
		}

		tcgetattr(gm_fd, &options);
		cfsetispeed(&options, speed_arr[1]);
		cfsetospeed(&options, speed_arr[1]);

		lseek(gm_fd, 0, 0);
		time(&timep);
		p = asctime(gmtime(&timep));	/* Wed Jun 30 21:49:08 1993/n */
		memset(buffer, 0, sizeof(buffer));
#if 1
		buffer[0] = 'A';
		buffer[1] = 'P';
		buffer[2] = ':';
		buffer[3] = ' ';
#else
		buffer[0] = 'C';
		buffer[1] = 'P';
		buffer[2] = ':';
		buffer[3] = ' ';
#endif
		strncat(buffer + 4, file_name, 12);
		strncat(buffer + 16, p, 25);
		//len = write(gm_fd, buffer, strlen(buffer));
		len = write(gm_fd, buffer, 512);
		if (len == -1) {
			perror("Cannot write /dev/ttyBif ... ");
			close(gm_fd);
			return -1;
		}
		printf("write %d data success ...\n", len);
		memset(buffer, 0, sizeof(buffer));
		lseek(gm_fd, 0, 0);
		len = read(gm_fd, buffer, 1024);
		if (len == -1) {
			perror("read file err...\n");
			close(gm_fd);
			return -1;
		}
		printf("read %d data success ...\n", len);
		printf("read data %s\n", buffer);
		close(gm_fd);

	}

	return 0;
}
