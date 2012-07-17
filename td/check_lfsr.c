#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>

static unsigned short
get_lfsr(int isdb, int index)
{
	return (unsigned short)((1 + 2 * isdb + index) * 12345);
}

int
main(int argc, char * const argv[])
{
	unsigned short lfsr, buf;
	int fd, i;

	if (argc != 4)
		return EXIT_SUCCESS;

	fd = open(argv[1], O_RDONLY);
	if (fd <= 0) {
		printf("can not open file %s.\n", argv[1]);
		return EXIT_SUCCESS;
	}

	lfsr = get_lfsr(atoi(argv[2]), atoi(argv[3]));
	for (i = 0; i < 3 * 1024 * 100 / 2; i++) {
		read(fd, &buf, sizeof(buf));
		if (buf != lfsr) {
			printf("check NG!\n");
			goto last;
		}
		lfsr = (lfsr >> 1) ^ (-(lfsr & 1) & 0xb400);
	}
	printf("check OK.\n");
last:
	close(fd);

	return EXIT_SUCCESS;
}
