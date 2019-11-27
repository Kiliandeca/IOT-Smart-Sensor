/****************************************************************************
 *   host/sd_translate/main.c
 *
 * E-Xanh Gardener SD card reader
 *
 * Copyright 2017 Nathael Pajani <nathael.pajani@ed3l.fr>
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
 *************************************************************************** */


#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <getopt.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <arpa/inet.h>


#define PROG_NAME  "Exanh sensors data dumper"
#define VERSION  "0.1"


void help(char *prog_name)
{
	fprintf(stderr, "-------- "PROG_NAME" ------\n");
	fprintf(stderr, "Usage: %s [options]\n" \
		"  Available options:\n" \
		"  \t -s | --source=raw_dump_file_name : Source file holding the raw data\n" \
		"  \t -i | --invalid-too : Also dump invalid block content\n" \
		"  \t -h | --help : Print this help\n" \
		"  \t -v | --version : Print programm version\n", prog_name);
	fprintf(stderr, "------------------------------------------------\n");
}



/***************************************************************************** */
/* RTC and time */

struct rtc_time {
	uint8_t hundredth_sec; /* 0 to 99 - BCD encoding */
	uint8_t sec; /* 0 to 59 - BCD encoding */
	uint8_t min; /* 0 to 59 - BCD encoding */
	uint8_t hour; /* 0 to 24 or AM/PM + 0 to 12 - BCD encoding */
	uint8_t day; /* 1 to 31 - BCD encoding */
	uint8_t weekday; /* 0 to 6 - BCD encoding */
	uint8_t month; /* 1 to  12 - BCD encoding */
	uint8_t year; /* 0 to 99 - BCD encoding */
};

void date_dump(struct rtc_time* date, char* msg)
{
	printf("%s : %02x-%02x-%02x %02x:%02x:%02x\n", msg,
				date->year, date->month, date->day,
				date->hour, date->min, date->sec);
}

/***************************************************************************** */
/* SD raw dump */
#define MMC_MAX_SECTOR_SIZE  512
#define MMC_BUF_SIZE  MMC_MAX_SECTOR_SIZE
#define MMC_RECORD_SIZE  32
static uint32_t mmc_block_num = 0;
uint8_t mmc_data[MMC_BUF_SIZE];

#define MMC_BLK_VALID_MAGIC  "This is a valid data block" /* Must be less than 32 */
#define MMC_BLK_VALID_IDX_MAGIC "ValidIDX"
#define MMC_BLK_VALID_IDX_MAGIC_SIZE 8
#define MMC_LAST_DATA_IDX_DATE_OFFSET  MMC_BLK_VALID_IDX_MAGIC_SIZE
#define MMC_LAST_DATA_IDX_OFFSET  10  /* 10 * 4 bytes */
#define MMC_LAST_DATA_IDX_BLOCK   1

int read_block(int source_fd)
{
	int len = read(source_fd, mmc_data, MMC_BUF_SIZE);
	if (len == MMC_BUF_SIZE) {
		/* Everything OK, increase block number indicator and return */
		mmc_block_num++;
		return 0;
	} else if (len < 0) {
		/* Bad one ... */
		perror("Read error:");
		printf("Source file read error on block %d.\n", mmc_block_num);
	} else if (len == 0) {
		/* End of file */
		printf("End of source file after block %d.\n", mmc_block_num);
	} else {
		printf("Last bit of data not big enough to fill a block\n");
	}
	return -1;
}

void dump_record(int offset)
{
	struct rtc_time date;
	uint16_t* data = NULL;
	int i = 0;

	/* Print record date */
	memcpy(&date, (mmc_data + offset), sizeof(struct rtc_time));
	date_dump(&date, "\tRecord date");
	offset += sizeof(struct rtc_time);

	/* Print record data */
	data = (uint16_t*)(mmc_data + offset);
	/* Switch data to our endianness */
	for (i = 1; i <= 8; i++) {
		data[i] = (uint16_t)ntohs(data[i]);
	}
	printf("\tFrom sensor %d:\n", (*(mmc_data + offset + 1) & 0x1F));
	printf("\t- Soil: %d\n", data[1]);
	printf("\t- Lux: %d, IR: %d, UV: %d\n", data[2], data[3], data[4]);
	printf("\t- Patm: %d hPa, Temp: %d,%02d degC, Humidity: %d,%d rH\n\n",
			data[5],
			data[6] / 10,  (data[6]> 0) ? (data[6] % 10) : ((-data[6]) % 10),
			data[7] / 10, data[7] % 10);
}

int dump_block(int source_fd, int dump_invalid)
{
	int offset = 0;
	struct rtc_time date;
	int tmp101_deci_degrees = 0, abs_deci = 0;
	int mv_batt = 0;

	if (read_block(source_fd) != 0) {
		return -1;
	}
	/* Check for MAGIC */
	if (strncmp((char*)mmc_data, MMC_BLK_VALID_MAGIC, (sizeof(MMC_BLK_VALID_MAGIC) - 1)) == 0) {
		printf("Block %d Seems valid.\n", mmc_block_num - 1);
	} else {
		printf("Block %d has wrong magic.\n", mmc_block_num - 1);
		if (dump_invalid == 0) {
			return 0;
		}
	}
	offset = 32;
	/* Dump data */
	/* Block creation date */
	memcpy(&date, (mmc_data + offset), sizeof(struct rtc_time));
	date_dump(&date, "Block creation date");
	offset += sizeof(struct rtc_time);
	/* Internal temperature */
	memcpy(&tmp101_deci_degrees, (mmc_data + offset), sizeof(tmp101_deci_degrees));
	abs_deci = tmp101_deci_degrees;
	if (tmp101_deci_degrees < 0) {
		abs_deci = -tmp101_deci_degrees;
	}
	printf("Internal temperature : % 4d.%02d\n", (tmp101_deci_degrees / 10), (abs_deci % 10));
	offset += sizeof(tmp101_deci_degrees);
	/* Battery voltage */
	memcpy(&mv_batt, (mmc_data + offset), sizeof(mv_batt));
	printf("Batterie: %d.%02d V\n", (mv_batt/1000), ((mv_batt/10)%100));

	/* Begining of data */
	offset = 64;

	while (offset <= (MMC_BUF_SIZE - MMC_RECORD_SIZE)) {
		dump_record(offset);
		offset += 32;
	}
	return 1;
}

int check_dump_valid(int source_fd)
{
	if (read_block(source_fd) != 0) {
		printf("Error when reading first block. Critical.\n");
		return -1;
	}
	if (read_block(source_fd) != 0) {
		printf("Error when reading second block. Critical.\n");
		return -1;
	}
	if (strncmp((char*)mmc_data, MMC_BLK_VALID_IDX_MAGIC, MMC_BLK_VALID_IDX_MAGIC_SIZE) != 0) {
		printf("Error : block 2 does not hold a valid magic\n");
		return -1;
	} else {
		static uint32_t maybe_last_valid_block = 0;
		printf("Valid second block found.\n");
		maybe_last_valid_block = ((uint32_t*)mmc_data)[MMC_LAST_DATA_IDX_OFFSET];
		printf("Number of data blocks is at least %d.\n", maybe_last_valid_block);
	}
	return 0;
}


int main(int argc, char** argv)
{
	char* source_name = NULL;
	int source = -1;
	int dump_invalid = 0;
	int ret = 0;

	while(1) {
		int option_index = 0;
		int c = 0;
		struct option long_options[] = {
			{"source", required_argument, 0, 's'},
			{"invalid-too", no_argument, 0, 'i'},
			{"help", no_argument, 0, 'h'},
			{"version", no_argument, 0, 'v'},
			{0, 0, 0, 0}
		};

		c = getopt_long(argc, argv, "s:ihv", long_options, &option_index);

		/* no more options to parse */
		if (c == -1) break;
		switch (c) {
			/* s, source */
			case 's':
				source_name = optarg;
				break;

			/* v, version */
			case 'i':
				dump_invalid = 1;
				break;

			/* v, version */
			case 'v':
				printf("%s Version %s\n", PROG_NAME, VERSION);
				return 0;
				break;

			/* h, help */
			case 'h':
			default:
				help(argv[0]);
				return 0;
		}
	}

	if (source_name == NULL) {
		printf("Error, need raw dump file name\n");
		help(argv[0]);
		return -1;
	}

	/* Open source file */
	source = open(source_name, O_RDONLY);
	if (source < 0) {
		printf("Unable to open specified input file %s\n", source_name);
		return -2;
	}

	/* Check that second block is valid */
	if (check_dump_valid(source) != 0) {
		return -3;
	}

	do {
		ret = dump_block(source, dump_invalid);
	} while (ret >= 0);

	/* Done */
	printf("End of dump. Dumped %d blocks.\n", mmc_block_num);

	return 0;
}



