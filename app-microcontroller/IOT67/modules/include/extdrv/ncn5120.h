/****************************************************************************
 *  extdrv/ncn5120.h
 *
 * Copyright 2014 Nathael Pajani <nathael.pajani@ed3l.fr>
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

/* NCN5120 driver
 * The NCN5120 chip is a KNX to serial transceiver made by On Semiconductor.
 * Refer to the NCN5120 documentation or http://www.onsemi.com/ for more information.
 */

#ifndef EXTDRV_NCN5120_H
#define EXTDRV_NCN5120_H

#include "lib/stdint.h"

struct ncn5120_status {
	uint8_t operation_mode;
	uint8_t out_of_sleep;
	uint8_t v20v_ok;
	uint8_t vdd2_ok;
	uint8_t vbus_ok;
	uint8_t vfilt_ok;
	uint8_t xtal_running;
	uint8_t thermal_warning;
	uint8_t out_of_thermal_shutdown;
};

struct ncn5120_comm_status {
	uint8_t slave_colision;
	uint8_t host_rx_error;
	uint8_t knx_tx_error;
	uint8_t protocol_error;
	uint8_t thermal_warning;
};

struct ncn5120_conf {
	uint8_t busy;
	uint8_t auto_acknowledge;
	uint8_t auto_polling;
	uint8_t crc_citt;
	uint8_t frame_end_marker;
};

enum ncn5120_states {
	NCN5120_STATE_NORMAL = 1,
	NCN5120_STATE_STANDBY,
	NCN5120_STATE_STARTUP,
	NCN5120_STATE_POWERUP,
	NCN5120_STATE_POWERUP_STOP,
	NCN5120_STATE_SYNC,
	NCN5120_STATE_STOP,
	NCN5120_STATE_BUSY,
	NCN5120_STATE_THERMAL_WARNING,
	NCN5120_STATE_RESET,
	NCN5120_STATE_MONITOR,
	NCN5120_STATE_SLEEP,
	NCN5120_STATE_THERMAL_SHUTDOWN,
};

/***************************************************************************** */
/*                NCN5120                                                      */
/***************************************************************************** */
struct ncn5120_device {
	uint8_t serial_num;
	/* Last known state for these : */
	uint8_t current_state;
	uint8_t current_op_mode;
	uint8_t thermal_state;
	/* Pin used for reset and save signals, and associated calbacks */
	struct pio reset_pin; /* RESET pin, used to know the state of the KNX chip */
	struct pio save_pin; /* SAVE pin, signal from KNX chip used as warning for power loss. */
	void (*save_callback)(uint32_t);
	void (*reset_callback)(uint32_t);

	/* Config */
	struct ncn5120_conf comm_config;
	int address_set;

	/* RX */
	/* Commands */
	volatile uint8_t rx_cmd_buff[280];
	volatile uint16_t rx_cmd_buff_idx;
	volatile uint16_t next_data_is_cmd_reply;
	/* Data */
	volatile uint8_t rx_data_running;
	void (*rx_callback)(uint32_t);
};

/* This value cannot be received on the KNX bus. When passed to the rx_callback it indicates that
 *   we received the end of a data packet.
 */
#define END_OF_DATA_PACKET  (0x100)

/***************************************************************************** */
/* List of services (Commands and data sent from host to NCN5120) */
enum ncn5120_services {
	/* Internal commands, device specific */
	U_Reset_Request = 0x01,    /* U_Reset.req - No data */
	U_State_Request = 0x02,    /* U_State.req - No data */
	U_SetBusy_Request = 0x03,  /* U_SetBusy.req - No data */
	U_QuitBusy_Request = 0x04, /* U_QuitBusy.req - No data */
	U_Busmon_Request = 0x05,   /* U_Busmon.req - No data */
	U_SetAddress_Request = 0xF1,    /* U_SetAddress.req - 3 data bytes : AddrHigh, AddrLow, DontCare  */
	U_SetRepetition_Request = 0xF2, /* U_SetRepetition.req - 3 data bytes : Rep_Count, DontCare, DontCare */
	U_L_DataOffset_Request = 0x08,  /* U_L_DataOffset.req - No data - Ored with MSB byte index in three lower bits */
	U_SystemStat_Request = 0x0D,    /* U_SystemStat.req - No data */
	U_StopMode_Request = 0x0E,      /* U_StopMode.req - No data */
	U_ExitStopMode_Request = 0x0F,  /* U_ExitStopMode.req - No data */
	U_Ackn_Request = 0x10,      /* U_Ackn.req - No data - Ored with 'nack' (0x04), 'busy' (0x02) and 'addressed' (0x01) */
	U_Configure_Request = 0x18, /* U_Configure.req - No data - Ored with 'auto-polling' (0x04), 'CRC-CITT' (0x02) and 'marker' (0x01) */
	U_IntRegWr_Request = 0x28,  /* U_IntRegWr.req - 1 data byte : data to write - Ored with two bits of internal reg addr */
	U_IntRegRd_Request = 0x38,  /* U_IntRegRd.req - No data - Ored with two bits of internal reg addr */
	U_PollingState_Request = 0xE0, /* U_PollingState.req - 3 data bytes : PollAddrHigh, PollAddrLow, PollState - Ored with 4 bits of slot number */
	/* KNX transmit data commands */
	U_L_DataStart_Request = 0x80,  /* U_L_DataStart.req - 1 data byte : Control byte */
	U_L_DataCont_Request = 0x80,  /* U_L_DataCont.req - Ored with 6 bits of index - 1 data byte : data */
	U_L_DataEnd_Request = 0x40,  /* U_L_DataEnd.req - Ored with 6 bits of index - 1 data byte : check byte */
};
/* List of replies (Replies to commands) */
enum ncn5120_replies {
	/* DLL (layer 2) services : device is transparent */
	L_Data_Standard = 0x90, /* FIXME : 3 indicator bits possible */
	L_Data_Extended = 0x10, /* FIXME : 3 indicator bits possible */
	L_Poll_Data = 0xF0,
	/* Acknowledge services (device is transparent in Bus monitor mode) */
	L_Ackn = 0x00, /* FIXME : 4 possible bit set */
	L_Data = 0x0B, /* FIXME : 0x8B is positive confirmation, 0x0B is negative confirmation */
	/* Control services, device specific */
	U_Reset_Reply = 0x03,
	U_State_Reply = 0x07, /* FIXME : upper 5 bits are state indicators */
	U_FrameState_Reply = 0x13, /* FIXME : 4 indicator bits possible */
	U_Configure_Reply = 0x01, /* FIXME : 5 indicator bits possible */
	U_Configure_Reply_Mask = 0x83, /* These are the constant bits */
	U_FrameEnd_Reply = 0xCB,
	U_StopMode_Reply = 0x2B,
	U_SystemStat_Reply = 0x4B, /* 1 data byte with 6 flags and 2 bits for mode */
};



/***************************************************************************** */
/* Services bits definitions */


/* Communication status bits */
#define NCN5120_COMM_STATUS_SLAVE_COLISION  (0x01 << 7) /* Collision during transmission of polling state */
#define NCN5120_COMM_STATUS_HOST_RX_ERROR   (0x01 << 6) /* Corrupted bytes sent by host controller (us) */
#define NCN5120_COMM_STATUS_KNX_TX_ERROR    (0x01 << 5) /* Transceiver error detected during frame transmission */
#define NCN5120_COMM_STATUS_PROTOCOL_ERROR  (0x01 << 4) /* incorrect sequence of commands sent by host */
#define NCN5120_COMM_STATUS_THERMAL_WARNING (0x01 << 3) /* Thermal warning condition detected */

/* Busy state */
#define NCN5120_ENTER_BUSY_STATE  1
#define NCN5120_QUIT_BUSY_STATE   0

/* Configure service status bits */
#define NCN5120_BUSY             (0x01 << 6)
#define NCN5120_AUTO_ACK         (0x01 << 5)
#define NCN5120_AUTO_POLLING     (0x01 << 4)
#define NCN5120_CRC_CITT_ACTIVE  (0x01 << 3)
#define NCN5120_FRAME_END_MARK_ACTIVE  (0x01 << 2)
/* Configure function request bits */
#define NCN5120_ACTIVATE_END_FRAME_MARKER  (0x01 << 0)
#define NCN5120_ACTIVATE_CRC_CITT     (0x01 << 1)
#define NCN5120_ACTIVATE_AUTO_POLL    (0x01 << 2)

/* System State service */
#define NCN5120_OP_MODE_MASK      (0x03)
#define NCN5120_OP_MODE_POWER_UP  0x00
#define NCN5120_OP_MODE_SYNC      0x01
#define NCN5120_OP_MODE_STOP      0x02
#define NCN5120_OP_MODE_NORMAL    0x03

/* ACK request bits */
#define NCN5120_ACK_NACK        (0x01 << 2)
#define NCN5120_ACK_BUSY        (0x01 << 1)
#define NCN5120_ACK_ADDRESSED   (0x01 << 0)


/***************************************************************************** */
/* Internal registers */
#define NCN5120_INT_REG_ADDR_SHIFT
#define NCN5120_INT_REG_ADDR_MASK

/* Watchdog register */
#define NCN5120_WDT_REG_ADDR  0x00
#define NCN5120_WDT_WDEN_SHIFT        7
#define NCN5120_WDT_ENABLE            (0x01 << NCN5120_WDT_WDEN_SHIFT)
#define NCN5120_WDT_DISABLE           0x00
#define NCN5120_WDT_DELAY_MASK        0x0F
#define NCN5120_WDT_DELAY_CONV_SHIFT  6
#define NCN5120_WDT_DELAY_MAX_VALUE   524

/* Analog Control register 0 */
#define NCN5120_ACR_0_ADDR    0x01
/* Xclock */
#define NCN5120_XCLOCK_SHIFT     3
#define NCN5120_XCLOCK_MASK      0x03
#define NCN5120_XCLOCK_DISABLE   0x00
#define NCN5120_XCLOCK_8MHZ      0x02
#define NCN5120_XCLOCK_16MHZ     0x03
/* Sleep */
#define NCN5120_SLEEP_ENABLE     0x80
#define NCN5120_SLEEP_DISABLE    0x00
/* V20V regulator */
#define NCN5120_V20V_ENABLE      0x40
/* DC2 converter */
#define NCN5120_DC2_ENABLE       0x20

/* Analog Control register 1 */
#define NCN5120_ACR_1_ADDR    0x02
#define NCN5120_ENTER_SLEEP      0x80

/* Analog Status register */
#define NCN5120_STATUS_REG_ADDR  0x03
#define NCN5120_STATUS_OUT_OF_SLEEP  (0x01 << 7)
#define NCN5120_STATUS_V20V_OK       (0x01 << 6)
#define NCN5120_STATUS_VDD2_OK       (0x01 << 5)
#define NCN5120_STATUS_VBUS_OK       (0x01 << 4)
#define NCN5120_STATUS_VFILT_OK      (0x01 << 3)
#define NCN5120_STATUS_XTAL_OK       (0x01 << 2)
#define NCN5120_STATUS_THERMAL_WARN  (0x01 << 1)
#define NCN5120_STATUS_OUT_OF_TSD    (0x01 << 0)



/***************************************************************************** */
/* NCN5120 Configuration and initialisation
 * rx_callback must not be NULL if you want to receive data. It will be called for
 *   any received character which is not part of a command reply.
 * If you did not set the device's address using ncn5120_set_address() then you are
 *   responsible of sending the ACK, NACK or BUSY using ncn5120_packet_acknowledge().
 * The UART will be configured in 38400, 8n1 and an intermediate callback will be
 *   used, so do not try to configure it yourself.
 * Analog configuration will be setup according to Techno-Innov's KNX module's
 *   requirements and communication configuration according to this module's
 *   requirements.
 */
void ncn5120_init(uint8_t serial, void (*rx_callback)(uint32_t));

/* Se callback on SAVEB and RESETB signal for emergency data saving.
 * Set a callback on SAVEB signal (goes low on KNX system errors)
 *   Possible errors : Temp warning (TW), Thermal shutdown warning (TSD)
 *   Read the system status to get the warning source. refer to System status
 *     service description (page 33 in ncn5120 documentation)
 * Set a callback on RESETB signal (goes low when watchdog is not reset)
 */
void ncn5120_register_handlers(const struct pio* save, void (*save_callback)(uint32_t),
							   const struct pio* reset, void (*reset_callback)(uint32_t));


/***************************************************************************** */
/* Watchdog configuration */
/* Enable and configure NCN5120 watchdog.
 * By default the watchdog is not active upon NCN5120 reset.
 * delay is a value in mili-seconds, with minimal value of 33ms and 33ms steps. If delay
 *   is 0 then the watchdog is turned OFF.
 * Refer to watchdog register description (page 50 in ncn5120 documentation).
 * Do not forget to call this function again before then of the watchdog timer. You can
 *   register a callback on RESETB signal (goes low when watchdog is not reset).
 */
void ncn5120_watchdog_config(uint32_t delay);


/***************************************************************************** */
/* Change the NCN5120 physical address and activate the auto-acknowledge function
 * NCN5120 starts accepting all frames whose destination address corresponds to the stored
 *   physical address or whose destination address is the group address by sending IACK on
 *   the bus. In case of an error detected during such frame reception, NCN5120 sends NACK
 *   instead of IACK.
 * The documentation says that the Set Address Service can be issued any time and that the
 *   new physical address and the autoacknowledge function will only get active after the
 *   KNX bus becomes idle. Anyway, it does not provide information on when the configuration
 *   byte will be sent, thus we do not allow the set address operation while receiving data.
 *
 * Returns -1 when receiving data and address did not get changed.
 *
 * Autoacknowledge can only be deactivated by a Reset Service (call to ncn5120_reset()). 
 */
int ncn5120_set_address(uint16_t address);


/***************************************************************************** */
/* Set repetition service
 * Specifies the maximum repetition count for transmitted frames when not acknowledged with IACK.
 * Separate counters can be set for NACK and BUSY frames.
 * Initial value of both counters is 3.
 * If the acknowledge from remote Data Link Layer is BUSY during frame transmission, NCN5120
 *    tries to repeat after at least 150 bit times KNX bus idle.
 * The BUSY counter determines the maximum amount of times the frame is repeated.
 * If the BUSY acknowledge is still received after the last try, an L_Data.con with a negative
 *    conformation is sent back to the host controller.
 * For all other cases (NACK acknowledgment received, invalid/corrupted acknowledge received or
 *    time−out after 30 bit times) NCN5120 will repeat after 50 bit times of KNX bus idle.
 * The NACK counter determines the maximum retries.
 * L_Data.con with a negative confirmation is send back to the host controller when the maximum
 *    retries were reached. In worst case, the same request is transmitted (NACK + BUSY + 1) times
 *    before NCN5120 stops retransmission.
 */
void ncn5120_set_number_of_retries(uint8_t busy_retries, uint8_t nack_retries);


/***************************************************************************** */
/* Get analog status
 * The operation_mode returned will be the latest known operation mode. It will not be updated
 *    during this call.
 */
int ncn5120_get_analog_status(struct ncn5120_status* status);


/* Get commuinication status as returned by U_State.req service */
int ncn5120_get_comm_status(struct ncn5120_comm_status* status);

/* Get system status.
 * The 'out_of_sleep' and 'out_of_thermal_shutdown' field have no meaning for this request
 *    and will be '0' regardless of the real state.
 */
int ncn5120_get_system_status(struct ncn5120_status* status);


/***************************************************************************** */
/* Request NCN5120 reset
 * Return 0 on success, -1 on error
 */
int ncn5120_reset(void);

/* Enter sleep mode if it has been enabled by config in Analog Control register 0 (call to ncn5120_set_config()
 *    with sleep_en = 1).
 * Warning: when entering sleep mode, only a Wake-Up Event on the FANIN/WAKE-pin can get the
 *    NCN5120 out of sleep. Make sure your hardware alows the generation of this event.
 */
void ncn5120_enter_sleep(void);

/* Set (enter or quit) busy mode.
 * Enter busy mode/state if 'state' is not 0.
 * In Busy mode and when autoacknowledge is active the NCN5120 rejects the frames whose
 *   destination address corresponds to the stored physical address by sending the BUSY
 *   acknowledge.
 * This service has no effect if autoacknowledge is not active.
 * Refer to p30 of NCN5120 documentation for busy state an p31 for autoacknowledge and set
 *   address service
 */
void ncn5120_set_busy_state(uint8_t state);

/* Set (enter or quit) stop mode.
 * Enter stop mode/state if 'state' is not 0.
 */
void ncn5120_set_stop_state(uint8_t state);


/* Enter Bus monitor mode
 * In this mode all data received from the KNX bus is sent to the host controller without
 *    performing any filtering on Data Link Layer. Acknowledge Frames are also transmitted
 *    transparently.
 * This state can only be exited by the Reset Service (ncn5120_reset()).
 */
void ncn5120_enter_monitor_mode(void);



/***************************************************************************** */
/* Receive packet helper
 * When an address got set using ncn5120_set_address() then the auto-ACK function is activated.
 * When it is not the case, the host is responsible for sending ACK, NACK or Busy on the Bus when
 *   it receives a packet
 */
void ncn5120_packet_acknowledge(uint8_t nack, uint8_t busy, uint8_t addressed);


/***************************************************************************** */
/* Send packet
 * When using a packet oriented communication with packet size and address included
 *   in the packet, these must be included in the packet by the software before
 *   calling this function.
 * The request_builder buffer is used to build parts of the request by sending the control
 *   and data bytes in the right order.
 *   There is no need to send them all at once so a small buffer is OK and saves memory. A
 *   three bytes buffer is OK for one data byte and the possible two index bytes sent at
 *   once.
 * The reply length depends on the communication configuration. It is at least the data
 *   size plus two control bytes.
 *
 * Return values:
 * This function returns a negativ value on errors:
 *    -1 when size is above KNX_MAX_FRAME_SIZE
 *    -2 when data has not been sent because the bus is busy receiving data.
 *    -3 when there has been an error while transmitting data to the NCN5120 (or checksum error)
 *    -4 if the NCN5120 indicates Rx errors (while we were sending, what would they mean ?)
 *    -5 if the data does not end with L_Data confirmation byte.
 * The function returns 1 when the packet got sent and received a positive ACK, or 0 when it
 *   received a negative ACK.
 */
int ncn5120_send_packet(uint8_t control_byte, uint8_t* buffer, uint16_t size);


#endif /* EXTDRV_NCN5120_H */

