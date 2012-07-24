#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include "../pt3_ioctl.h"

#define DEV0 "/dev/pt3video0"
#define DEV1 "/dev/pt3video1"
#define DEV2 "/dev/pt3video2"
#define DEV3 "/dev/pt3video3"

int
test_open(const char* src, const char* dst)
{
	char buf[1024 * 6];
	size_t rsize;
	int fd_src, fd_dst, i, status, rc;

	fd_src = open(src, O_RDONLY);
	if (fd_src <= 0) {
		fprintf(stderr, "can not open '%s'.\n", src);
		return 1;
	}
	fd_dst = open(dst, O_WRONLY | O_CREAT, 0644);
	if (fd_dst <= 0) {
		close(fd_src);
		fprintf(stderr, "can not open '%s'.\n", dst);
		return 1;
	}
	printf("test : %s\n", src);

	rc = ioctl(fd_src, GET_STATUS, &status);
	if (rc < 0)
		perror("fail get status");
	printf("status = 0x%08x\n", status);

	rc = ioctl(fd_src, SET_TEST_MODE_ON, 0);
	if (rc < 0)
		perror("fail set test mode on.");

	rc = ioctl(fd_src, GET_STATUS, &status);
	if (rc < 0)
		perror("fail get status");
	printf("status = 0x%08x\n", status);

	for (i = 0; i < 100; i++) {
		rsize = read(fd_src, buf, sizeof(buf));
		write(fd_dst, buf, rsize);
	}

	rc = ioctl(fd_src, SET_TEST_MODE_OFF, 0);
	if (rc < 0)
		perror("fail set test mode off.");

	rc = ioctl(fd_src, GET_STATUS, &status);
	if (rc < 0)
		perror("fail get status");
	printf("status = 0x%08x\n", status);

last:
	close(fd_dst);
	close(fd_src);

	return 0;
}

int
rec_open(const char* src, const char* dst, int channel)
{
	FREQUENCY freq;
	char buf[1024 * 6];
	size_t rsize;
	int fd_src, fd_dst, i, status, rc;
	unsigned int count;

	fd_src = open(src, O_RDONLY);
	if (fd_src <= 0) {
		fprintf(stderr, "can not open '%s'.\n", src);
		return 1;
	}
	fd_dst = open(dst, O_WRONLY | O_CREAT, 0644);
	if (fd_dst <= 0) {
		close(fd_src);
		fprintf(stderr, "can not open '%s'.\n", dst);
		return 1;
	}
	printf("rec : %s\n", src);

	rc = ioctl(fd_src, GET_STATUS, &status);
	if (rc < 0)
		perror("fail get status");
	printf("status = 0x%08x\n", status);

	freq.frequencyno = channel;
	freq.slot = 0;
	rc = ioctl(fd_src, SET_CHANNEL, &freq);
	if (rc)
		fprintf(stderr, "set frequencyno=%d slot=%d rc=0x%x\n",
				freq.frequencyno, freq.slot, rc);

	rc = ioctl(fd_src, GET_STATUS, &status);
	if (rc < 0)
		perror("fail get status");
	printf("status = 0x%08x\n", status);

	rc = ioctl(fd_src, START_REC, 0);
	if (rc)
		fprintf(stderr, "start rec rc=0x%x\n", rc);

	rc = ioctl(fd_src, GET_TS_ERROR_PACKET_COUNT, &count);
	if (rc < 0)
		perror("fail get ts error packet count");
	printf("ts error packet = %d\n", count);
	
	rc = ioctl(fd_src, GET_STATUS, &status);
	if (rc < 0)
		perror("fail get status");
	printf("status = 0x%08x\n", status);

	sleep(2);

	for (i = 0; i < 1000; i++) {
		rsize = read(fd_src, buf, sizeof(buf));
		if (rsize > 0)
			write(fd_dst, buf, rsize);
	}

	rc = ioctl(fd_src, GET_STATUS, &status);
	if (rc < 0)
		perror("fail get status");
	printf("status = 0x%08x\n", status);

	rc = ioctl(fd_src, GET_TS_ERROR_PACKET_COUNT, &count);
	if (rc < 0)
		perror("fail get ts error packet count");
	printf("ts error packet = %d\n", count);

	rc = ioctl(fd_src, STOP_REC, 0);
	if (rc)
		fprintf(stderr, "stop rec rc=0x%x\n", rc);

	rc = ioctl(fd_src, GET_STATUS, &status);
	if (rc < 0)
		perror("fail get status");
	printf("status = 0x%08x\n", status);
last:
	close(fd_dst);
	close(fd_src);

	return 0;
}

int
main(int argc, char * const argv[])
{
	int i;
	char str[100];
	/*
	test_open(DEV0, "test0.ts");
	test_open(DEV1, "test1.ts");
	test_open(DEV2, "test2.ts");
	test_open(DEV3, "test3.ts");
	*/

	/*
	for (i = 0; i < 36; i++) {
		sprintf(str, "rec0-%02d.ts", i);
		rec_open(DEV0, str, i);
	}
	*/
	rec_open(DEV1, "rec1.ts", 21);
	rec_open(DEV2, "rec2.ts", 99);
	rec_open(DEV3, "rec3.ts", 76);
}
