/*
 * lib/protocols/dtplug/defs.h
 *
 *
 * Copyright 2013-2015 Nathael Pajani <nathael.pajani@ed3l.fr>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
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
 */

#ifndef LIB_PROTOCOLS_DTPLUG_DEFS_H
#define LIB_PROTOCOLS_DTPLUG_DEFS_H


#include "lib/stdint.h"

/******************************************************************************/
/* These structures define the packets used to transfer data over the serial link
 *    between the dtplug (or domotab or any computing platform) and a module
 */

/* The header is included in each and every packet we need to transfer
 * It holds information about the message type and content and provides checksum
 * information of the header and the data part of the packet if any.
 *
 * Checksums information :
 *    The header checksum is a value such that the sum of all bytes of the header
 *        modulo 256 is equal to 0. (Use a mask with 0xFF to get the modulo)
 *    The data checksum the sum of all data bytes modulo 256.
 *
 * Sequence numbers, Quick data packets, data checksums and error codes
 *    The sequence number is on the 5 least significant bits of the sequence number field.
 *      this gives 64 sequence numbers which is more than enough.
 *    When the most significant bit (bit 7) of the sequence number field (seq_num) is set
 *      in a request (master to slave), the slave MUST send an acknowledge packet or a reply
 *      with the same sequence number.
 *      This may help checking which packet triggered the reply and packet losses.
 *    When bit 7 is set in a reply, it indicates that the packet is an error packet and that
 *      the union holds error information (error code and aditional error information.
 *      No data should be present in the packet.
 *    When set, bit 6 of the sequence number indicates that the union holds 'quick_data'.
 *      This bit cannot be used for error packets.
 *      A packet with "quick-data" cannot have additional data.
 *      The number of bytes in the 'quick_data' packet is set by bit 5, which is 1 for two
 *      bytes and 0 for one byte.
 *    When bit 6 is not set, and the packet is not an error packet, the union holds a
 *      'data_information' structure with the packet data 'size' (if any) and 'data_checksum'
 *      for the packet data part.
 *
 * Data size and big (continued) data packets
 *    When a packet does not have enough room for the whole data the most significant bit
 *      (bit 7) of the data size is set to mark a packet whose data will be continued in a
 *      subsequent PKT_TYPE_CONTINUED_DATA packet.
 *      this bit must be set in all but the last one of a continued data packet serie.
*/

/* Normal packets 'sel' field information : holds information about the data part.
 * A 'size' of 0 indicates that there is no data part.
 */
struct data_information {
	uint8_t size;  /* Size of the data part. The header size is fixed, so not included */
	uint8_t checksum;  /* Checksum for the data part */
} __attribute__ ((__packed__));
/* Error packet 'sel' field */
struct error_information {
	uint8_t error_code;
	uint8_t info;
} __attribute__ ((__packed__));

/* Packet header */
struct header {
	char start;    /* Start of paquet indicator */
	uint8_t type;  /* Packet type, used to provide information on the data structure */
	uint8_t checksum;  /* Header checksum */
	uint8_t seq_num; /* Packet sequence number on bits 0:5, error indicator on bit 7 */
	union {
		struct data_information data;
		struct error_information err;
		uint8_t quick_data[2];
	};
} __attribute__ ((__packed__));

#define FIRST_PACKET_CHAR  '#'
/* Sequence byte decoding */
#define PACKET_NEEDS_REPLY (0x01 << 7) /* Host to slave */
#define PACKET_IS_ERROR    (0x01 << 7) /* Slave to host */
#define QUICK_DATA_PACKET  (0x01 << 6)
#define QUICK_DATA_PACKET_ONE_BYTE  (0x00 << 5)
#define QUICK_DATA_PACKET_TWO_BYTES (0x01 << 5)
#define SEQUENCE_MASK  (0x1F)
/* Data size decoding */
#define BIG_DATA_PKT       (0x01 << 7) /* The total data size is above PACKET_DATA_SIZE, the packet will be continued. */
#define PKT_SIZE_MASK  (0x7F)

/* The following struct defines a generic packet.
 * Specific packets should be defined in specific structures.
 */
#define PACKET_DATA_SIZE  64
struct packet {
	struct header info;  /* Packet header */
	uint8_t data[PACKET_DATA_SIZE];   /* packet data */
} __attribute__ ((__packed__));



enum packet_types {
	/* Common packet types, must be handled by all the slaves */
	PKT_TYPE_RESET = 0, /* Soft reset of board */
	PKT_TYPE_PING, /* Reply with no data or urgent data */
	PKT_TYPE_GET_BOARD_INFO, /* Return board name, version, module version, and programm version. */
	PKT_TYPE_SET_TIME, /* Set current time for events timestamping */
	PKT_TYPE_GET_NUM_PACKETS, /* Get the number of packets received since system start */
	PKT_TYPE_GET_ERRORS, /* Ask the slave to return any active error code in the data */
	PKT_TYPE_GET_NUM_ERRORS, /* Get the number of errors since system start */
	PKT_TYPE_SET_USER_INFO, /* Change the current board user information (and reset board) */

	/* Config */
	PKT_TYPE_ADD_GPIO, /* Configure one of the IO as GPIO */
	PKT_TYPE_ADD_CS,   /* Configure one of the IO as SPI Chip select */

	/* ADC config specifies :
	 *   the channel number,
	 *   periodicity of sample (from on request up to continuous),
	 *   and number of values for the continuous average computation
	 */
	PKT_TYPE_ADD_ADC,  /* Configure one of the ADC capable IO as ADC */

	/* PWM config specifies :
	 *   the channel number,
	 *   the PWM period,
	 *   the channel default duty cycle,
	 */
	PKT_TYPE_ADD_PWM,  /* Configure one of the PWM capable IO as PWM */

	/* Continued data. Use for protocols with data size that goes beyond PACKET_DATA_SIZE */
	PKT_TYPE_CONTINUED_DATA,

	/* Temperature packets */
	PKT_TYPE_START_TEMP_CONVERSION, /* Requiered to update the temperature value */
	PKT_TYPE_GET_TEMPERATURE, /* Get temprature values */

	/* ADC, PWM and GPIO packets */
	PKT_TYPE_START_ADC_CONVERSION,
	PKT_TYPE_GET_ADC_VALUE,
	PKT_TYPE_SET_GPIO,
	PKT_TYPE_GET_GPIO,
	PKT_TYPE_SET_PWM_CHAN,

	/* RGB Leds control */
	/* Get and set have the following arguments :
	 *   the led (pixel) number (uint8_t)
	 *   red value (uint8_t)
	 *   green value (uint8_t)
	 *   blue value (uint8_t)
	 */
	PKT_TYPE_SET_RGB_LED, /* If in quick data packet, set the led to off (0,0,0) */
	PKT_TYPE_GET_RGB_LED,
	PKT_TYPE_CLEAR_LEDS, /* No arguments */

	/* Communication */
	PKT_TYPE_SEND_ON_BUS,

	/* End of list, to be used as offset for user defined values */
	PKT_TYPE_LAST,
};

/* Error / status codes */
enum reply_statuses {
	NO_ERROR = 0,
	GOT_MANY_ERRORS, /* This one is returned when there's not only one error */
	ERROR_PKT_NOT_HANDLED,
	ERROR_LAST_PKT_IN_PROCESS,
	ERROR_IN_PKT_STRUCTURE,
	ERROR_IN_DATA_CHECKSUM,
	ERROR_IN_DATA_VALUES,
	ERROR_IN_DATA_SIZE, /* To many data */
	ERROR_IN_UART_TX,  /* This one is critical if received, and much more critical when it occurs and cannot be sent. */
	/* Temperature */
	ERROR_NO_TEMP_SENSOR,
	ERROR_TEMP_CONVERSION,
	TEMPERATURE_ALERT,
	/* Configuration problem */
	ERROR_FLASH_ERASE, /* Error, unable to erase the user information block */
	ERROR_FLASH_WRITE, /* Error, unable to write the user information block */
	/* RF */
	RF_RX_QUEUE_EMPTY, /* Na RX packet in RX queue */
	ERROR_IN_RF_RX,
	ERROR_IN_RF_TX,
	/* Last error number ... must not be above 255 */
	LAST_ERROR_NUM,  /* This one is here to get the value of the last enum. */
};



/******************************************************************************/
/* Other information on the protocol */

/* When a packet is received and requires an ACK but no particular data, the slave replies
 *    using a PING packet, with the sequence field set to the sequence number of the last received packet).
 *    When the slave has urgent data to send (either errors or alerts), it uses the corresponding bits in
 *    the sequence byte and places the 'appropriate data (error_information or quick_dtaa) in the 'sel' field.
 */

/* Multiple-bytes data are sent in Big Endian (network endian) !!!!
 * This is the usual way to send data over a network, and even if most of our boards and computers are little endian
 *   there are very little multi-byte information in our protocol so it's OK
 * The endianness transformation may be done before or after the checksum control, it has no influence on the
 *   checksum algorithm we chose.
 */

/* What to do on error, and error recovery procedure.
 * When a slave has an internal error, it has two options.
 * Either it's the first one, and the master requested a reply. The slave can then send the error
 *   code in the reply.
 * Or it's not the first one or the master did not ask for a reply or ACK. Then the slave stores the
 *   error code for future transmission.
 *   If the master asked for a reply and the slave has many errors, the slave send "GOT_MANY_ERRORS" as error code.
 * When the master receives the "GOT_MANY_ERRORS" error code, it must send a "PKT_TYPE_GET_ERRORS" request.
 * The slave then replies with a list of all errors in the packet data with a header which does not indicate an
 *   error packet (this is the normal reply to a PKT_TYPE_GET_ERRORS request.
 * A slave cannot store more than 8 error codes, if the count is 8, consider that there is a BIG problem !
 */




#endif /* LIB_PROTOCOLS_DTPLUG_DEFS_H */
