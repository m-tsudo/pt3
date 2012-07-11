#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define DEV0 "/dev/pt3video0"
#define DEV1 "/dev/pt3video1"
#define DEV2 "/dev/pt3video2"
#define DEV3 "/dev/pt3video3"

int
test_open(const char* src, const char* dst)
{
	char buf[1024];
	size_t rsize;
	int fd_src, fd_dst;

	fd_src = open(src, O_RDONLY);
	if (fd_src <= 0) {
		fprintf(stderr, "can not open '%s'.\n", src);
		return 1;
	}
	fd_dst = open(dst, O_WRONLY | O_CREAT);
	if (fd_dst <= 0) {
		close(fd_src);
		fprintf(stderr, "can not open '%s'.\n", dst);
		return 1;
	}

	rsize = read(fd_src, buf, sizeof(buf));
	write(fd_dst, buf, rsize);

	close(fd_dst);
	close(fd_src);

	return 0;
}

int
main(int argc, char * const argv[])
{
	test_open(DEV0, "dev0.ts");
	test_open(DEV1, "dev1.ts");
	test_open(DEV2, "dev2.ts");
	test_open(DEV3, "dev3.ts");
}
