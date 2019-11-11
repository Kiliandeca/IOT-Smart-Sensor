/****************************************************************************
 *  ADC Rx test
 *   main.c
 *
 * Copyright 2013-2014 Nathael Pajani <nathael.pajani@ed3l.fr>
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************************/

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <sys/time.h>



#define SERIAL_BAUD  B1152000

int serial_setup(char* name)
{
	struct termios tio;
	int fd = -1;

	/* Open serial port */
	fd = open(name, O_RDWR | O_NONBLOCK);
	if (fd < 0) {
		perror("Unable to open communication with companion chip");
		return -1;
	}
	/* Setup serial port */
	memset(&tio, 0, sizeof(tio));
	tio.c_cflag = CS8 | CREAD | CLOCAL;	/* 8n1, see termios.h for more information */
	tio.c_cc[VMIN] = 1;
	tio.c_cc[VTIME] = 5;
	cfsetospeed(&tio, SERIAL_BAUD);
	cfsetispeed(&tio, SERIAL_BAUD);
	tcsetattr(fd, TCSANOW, &tio);

	return fd;
}



int main(int argc, char* argv[])
{
	int tty_fd = -1;
	int out_fd = 1;

	/* Need Serial port and destination file as parameter */
	if (argc != 3) {
		printf("Need tty device number and destination file\n");
		printf("Please start with %s /dev/ttyUSB0 raw_audio (for example)\n", argv[0]);
		return -1;
	}

	/* Open tty */
	tty_fd = serial_setup(argv[1]);
	if (tty_fd < 0) {
		printf("Unable to open specified serial port %s\n", argv[1]);
		return -4;
	}

	/* And never stop getting data ! */
	while (1) {
		char buf[BUF_SIZE];
		int ret = 0, len = 0;

		memset(buf, 0, BUF_SIZE);
		len = read(tty_fd, buf, BUF_SIZE);

	} /* End of infinite loop */

	close(tty_fd);
	close(out_fd);
	return 0;
}


