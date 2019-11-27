#include <stdio.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>


int main(int argc, char** argv)
{
	int source_fd = -1;
	int dest_fd = -1;
	char inbuff[8];

	if (argc < 2) {
		printf("Usage: %s input_file output_file\n", argv[0]);
	}

	source_fd = open(argv[1], O_RDONLY);
	dest_fd = open(argv[2], (O_RDWR | O_CREAT), (S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH));

	if ((source_fd < 0) || (dest_fd < 0)) {
		printf("unable to open input or output file\n");
	}

	while (read(source_fd, inbuff, 8) == 8) {
		int i = 0;
		char outbuff = 0;
		for (i = 0; i < 8; i++) {
			outbuff |= ((inbuff[i] & 0x01) << i);
		}
		write(dest_fd, &outbuff, 1);
	}

	close(source_fd);
	close(dest_fd);

	return 0;
}
