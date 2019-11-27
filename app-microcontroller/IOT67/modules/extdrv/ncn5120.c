/****************************************************************************
 *  extdrv/ncn5120.c
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

#include "core/system.h"
#include "core/systick.h"
#include "core/pio.h"
#include "lib/string.h"
#include "lib/time.h"
#include "drivers/serial.h"
#include "drivers/gpio.h"
#include "extdrv/ncn5120.h"
#include "lib/protocols/knx.h"

/* Debug */
#include "lib/stdio.h"


/* NCN5120 driver
 * The NCN5120 chip is a KNX to serial transceiver made by On Semiconductor.
 * Refer to the NCN5120 documentation or http://www.onsemi.com/ for more information.
 */


static struct ncn5120_device ncn5120 = {
	.serial_num = 1,
	.current_state = NCN5120_STATE_NORMAL,
	.save_callback = NULL,
	.reset_callback = NULL,
	.rx_callback = NULL,
	.address_set = 0,
	.rx_data_running = 0,
};



/***************************************************************************** */
/* Interrupt handlers */

/* Handle RESETB pin state change */
static void reset_event(uint32_t gpio)
{
	if (ncn5120.reset_callback!= NULL) {
		ncn5120.reset_callback(gpio);
	}
}
/* Handle SAVEB pin state change */
static void save_event(uint32_t gpio)
{
	if (ncn5120.save_callback != NULL) {
		ncn5120.save_callback(gpio);
	}
}


/* Handle data and replies received on NCN5120 UART */
static void ncn5120_handle_rx(uint8_t c)
{
	if (ncn5120.next_data_is_cmd_reply > 0) {
		/* Receive command reply */
		ncn5120.rx_cmd_buff[ncn5120.rx_cmd_buff_idx++] = c;
		ncn5120.next_data_is_cmd_reply--;
	} else {
		/* Receive KNX data */
		static struct time_spec last_byte;
		static uint32_t last_timer_tick;

		/* Timer based end of frame detection */
		if (ncn5120.rx_data_running == 0) {
			ncn5120.rx_data_running = 1;
			get_time_in_interrupt(&last_byte);
			last_timer_tick = systick_get_timer_val();
		} else {
			struct time_spec this_byte, diff;
			uint32_t this_tick = systick_get_timer_val();
			get_time_in_interrupt(&this_byte);
			/* Check end of Receiving condition. Consider that the time difference will never be above a few msec. */
			get_time_diff(&last_byte, &this_byte, &diff);
			if (diff.msec >= 1) {
				uint32_t reload = systick_get_timer_reload_val();
				uint32_t tick_diff = (this_tick + (reload * (diff.msec + 1)) - last_timer_tick);
				if (((tick_diff * 10) / reload) > 26) {
					/* More than 2.6ms elapsed since previous byte : end of frame */
					ncn5120.rx_data_running = 0;
				}
			}
			last_byte.seconds = this_byte.seconds;
			last_byte.msec = this_byte.msec;
			last_timer_tick = this_tick;
		}
		if (ncn5120.rx_callback != NULL) {
			if (ncn5120.rx_data_running == 1) {
				ncn5120.rx_callback(c);
			} else {
				ncn5120.rx_callback(c | END_OF_DATA_PACKET);
			}
		}
	}
}



/***************************************************************************** */
/* Get command replies */

static void ncn5120_drop_cmd_data(void)
{
	lpc_disable_irq();
	ncn5120.rx_cmd_buff_idx = 0;
	dsb();
	lpc_enable_irq();
}
/* Tell the UART Rx handler that we will receive some command reply bytes */
static void ncn5120_set_nb_cmd_req_data(uint16_t len)
{
	ncn5120.next_data_is_cmd_reply += len;
}
/* Get the requested reply bytes */
static void ncn5120_get_req_reply_data(uint8_t* buf, uint16_t len)
{
	/* Wait for data to be received (happens in ncn5120_handle_rx() interrupt routine) */
	do {} while (ncn5120.rx_cmd_buff_idx < len);
	/* Lock */
	lpc_disable_irq();
	memcpy(buf, (void*)ncn5120.rx_cmd_buff, len);
	ncn5120.rx_cmd_buff_idx -= len;
	/* Most of the time all data is read, but ... */
	if (ncn5120.rx_cmd_buff_idx != 0) {
		memcpy((void*)ncn5120.rx_cmd_buff, (void*)(ncn5120.rx_cmd_buff + len), ncn5120.rx_cmd_buff_idx);
	}
	dsb();
	lpc_enable_irq();
}

/***************************************************************************** */
/* Send requests - Services access */
static void ncn5120_send_service_request(uint8_t* buf, uint16_t len, uint16_t reply_len)
{
	/* Register how many byte this request should generate as reply */
	ncn5120_set_nb_cmd_req_data(reply_len);
	/* And send the request */
	serial_write(ncn5120.serial_num, (char*)buf, len);
}

/***************************************************************************** */
/* Internal registers access */
/* Set NCN5120 internal register by isuing U_IntRegWr.req command */
static void ncn5120_set_internal_register(uint8_t reg_addr, uint8_t value)
{
	uint8_t buf[2];
	if (reg_addr > 0x03) {
		return;
	}
	buf[0] = U_IntRegWr_Request | reg_addr;
	buf[1] = value;
	ncn5120_send_service_request(buf, 2, 0);
}
/* Get NCN5120 internal register by isuing U_IntRegRd.req command */
static uint8_t ncn5120_get_internal_register(uint8_t reg_addr)
{
	uint8_t reg_val = 0;

	/* Register address cannot be above 0x03 */
	reg_addr &= 0x03;

	/* Drop all previously received command data */
	ncn5120_drop_cmd_data();

	/* Send request */
	reg_addr |= U_IntRegRd_Request;
	ncn5120_send_service_request(&reg_addr, 1, 1);
	/* And wait for our data to come */
	ncn5120_get_req_reply_data(&reg_val, 1);
	return reg_val;
}



/***************************************************************************** */
/* Set NCN5120 configuration */
static void update_thermal_state(uint8_t thermal_warning)
{
	if (thermal_warning) {
		ncn5120.thermal_state = NCN5120_STATE_THERMAL_WARNING;
	} else {
		ncn5120.thermal_state = NCN5120_STATE_NORMAL;
	}
}

/* Decode config reply */
static void ncn5120_parse_conf_reply(struct ncn5120_conf* conf, uint8_t reply)
{
	conf->busy = (reply & NCN5120_BUSY);
	if (conf->busy) {
		ncn5120.current_op_mode = NCN5120_STATE_BUSY;
	}
	conf->auto_acknowledge = (reply & NCN5120_AUTO_ACK);
	conf->auto_polling = (reply & NCN5120_AUTO_POLLING);
	conf->crc_citt = (reply & NCN5120_CRC_CITT_ACTIVE);
	conf->frame_end_marker = (reply & NCN5120_FRAME_END_MARK_ACTIVE);
}
/* Communication configuration
 * Activate communication functionnality when corresponding parameter is not nul.
 * It is only possible to activate services and functions. De-activation requires NCN5120 reset.
 * Note :
 *   This driver does not support crc-citt and frame end indicators.
 *   These parameters will be ignored.
 */
static int ncn5120_set_comm_config(uint8_t auto_poll, uint8_t crc_citt, uint8_t use_frame_end_indicator)
{
	uint8_t request = U_Configure_Request;
	uint8_t reply = 0;
	/* Do not change config if NCN5120 is currently receiving data */
	if (ncn5120.rx_data_running) {
		return -1;
	}
	/* Send the new config */
	if (auto_poll) {
		request |= NCN5120_ACTIVATE_AUTO_POLL;
	}
	ncn5120_send_service_request(&request, 1, 1);
	/* And read the configure byte in reply */
	ncn5120_get_req_reply_data(&reply, 1);
	ncn5120_parse_conf_reply(&(ncn5120.comm_config), reply);
	return 0;
}


/* Set analog configuration
 * Activate analog functionnality when corresponding parameter is not nul.
 */
static void ncn5120_set_analog_config(uint8_t sleep_en, uint8_t v20v_en, uint8_t dc2_en, uint8_t xclock)
{
	uint8_t reg_val = 0;
	if (sleep_en) {
		reg_val |= NCN5120_SLEEP_ENABLE;
	}
	if (v20v_en) {
		reg_val |= NCN5120_V20V_ENABLE;
	}
	if (dc2_en) {
		reg_val |= NCN5120_DC2_ENABLE;
	}
	if (xclock) {
		reg_val |= ((xclock & NCN5120_XCLOCK_MASK) << NCN5120_XCLOCK_SHIFT);
	}
	ncn5120_set_internal_register(NCN5120_ACR_0_ADDR, reg_val);
}



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
void ncn5120_init(uint8_t serial, void (*rx_callback)(uint32_t))
{
	/* Serial configuration */
	ncn5120.serial_num = serial;
	ncn5120.rx_callback = rx_callback;
	uart_on(ncn5120.serial_num, 38400, ncn5120_handle_rx);

	/* Initialize time system. Can be called anytime, will just return if it has already been called. */
	time_init();

	/* Initial driver configuration : sleep_en, v20v_en, dc2_en, external clock */
	ncn5120_set_analog_config(0, 0, 0, 0);
	/* Communication config : auto_poll, crc_citt, use_frame_end_indicator
	 * Note that crc_citt and use_frame_end_indicator modes are not supported. */
	ncn5120_set_comm_config(0, 0, 0);
}

/* Se callback on SAVEB and RESETB signal for emergency data saving.
 * Set a callback on SAVEB signal (goes low on KNX system errors)
 *   Possible errors : Temp warning (TW), Thermal shutdown warning (TSD)
 *   Read the system status to get the warning source. refer to System status
 *     service description (page 33 in ncn5120 documentation)
 * Set a callback on RESETB signal (goes low when watchdog is not reset)
 */
void ncn5120_register_handlers(const struct pio* save, void (*save_callback)(uint32_t),
							   const struct pio* reset, void (*reset_callback)(uint32_t))
{   
	/* Save pin */
	config_pio(reset, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL));
	pio_copy(&ncn5120.save_pin, save);
	gpio_dir_in(ncn5120.save_pin);
	set_gpio_callback(save_event, &ncn5120.save_pin, EDGES_BOTH);
	ncn5120.save_callback = save_callback;

	/* Reset pin */
	config_pio(reset, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL));
	pio_copy(&ncn5120.reset_pin, reset);
	gpio_dir_in(ncn5120.reset_pin);
	set_gpio_callback(reset_event, &ncn5120.reset_pin, EDGES_BOTH);
	ncn5120.reset_callback = reset_callback;
}


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
void ncn5120_watchdog_config(uint32_t delay)
{
	uint8_t reg_val = 0;
	if (delay == 0) {
		reg_val = NCN5120_WDT_DISABLE;
	} else {
		if (delay > NCN5120_WDT_DELAY_MAX_VALUE) {
			delay = NCN5120_WDT_DELAY_MAX_VALUE;
		}
		/* Set WDEN and configure Watchdog timing WDT[3:0] (tWDTO) */
		reg_val = NCN5120_WDT_ENABLE;
		reg_val |= ((delay >> NCN5120_WDT_DELAY_CONV_SHIFT) & NCN5120_WDT_DELAY_MASK);
	}
	ncn5120_set_internal_register(NCN5120_WDT_REG_ADDR, reg_val);
}


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
int ncn5120_set_address(uint16_t address)
{
	uint8_t buf[4] = { U_SetAddress_Request, 0, 0, 0 };
	uint8_t reply = 0;
	/* Do not change address if NCN5120 is currently receiving data */
	if (ncn5120.rx_data_running) {
		return - 1;
	}
	buf[1] = ((address >> 8) & 0xFF);
	buf[2] = (address & 0xFF);
	/* Send the new address (three data bytes, the last one is ignored) */
	ncn5120_send_service_request(buf, 4, 1);
	/* And read the configure byte in reply */
	ncn5120_get_req_reply_data(&reply, 1);
	ncn5120_parse_conf_reply(&(ncn5120.comm_config), reply);
	ncn5120.address_set = 1;
	return 0;
}


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
void ncn5120_set_number_of_retries(uint8_t busy_retries, uint8_t nack_retries)
{
	uint8_t buf[4] = { U_SetRepetition_Request, 0, 0, 0 };
	buf[1] = (((busy_retries & 0x07) << 4) | (nack_retries & 0x07));
	/* Send the new retry count */
	ncn5120_send_service_request(buf, 4, 0);
}



/***************************************************************************** */
/* Get analog status
 * The operation_mode returned will be the latest known operation mode. It will not be updated
 *    during this call.
 */
int ncn5120_get_analog_status(struct ncn5120_status* status)
{
	uint8_t reg_val = 0;
	/* Erase buffer, so that returned status is consistent */
	memset(status, 0, sizeof(struct ncn5120_status));
	/* Do not request status if NCN5120 is currently receiving data */
	if (ncn5120.rx_data_running) {
		return -1;
	}
	/* Get internal status byte */
	reg_val = ncn5120_get_internal_register(NCN5120_STATUS_REG_ADDR);
	/* Update struct according to received byte */
	status->out_of_sleep = (reg_val & NCN5120_STATUS_OUT_OF_SLEEP);
	status->v20v_ok = (reg_val & NCN5120_STATUS_V20V_OK);
	status->vdd2_ok = (reg_val & NCN5120_STATUS_VDD2_OK);
	status->vbus_ok = (reg_val & NCN5120_STATUS_VBUS_OK);
	status->vfilt_ok = (reg_val & NCN5120_STATUS_VFILT_OK);
	status->xtal_running = (reg_val & NCN5120_STATUS_XTAL_OK);
	status->thermal_warning = (reg_val & NCN5120_STATUS_THERMAL_WARN);
	update_thermal_state(status->thermal_warning);
	status->out_of_thermal_shutdown = (reg_val & NCN5120_STATUS_OUT_OF_TSD);
	status->operation_mode = ncn5120.current_op_mode;
	return 0;
}


/* Get commuinication status as returned by U_State.req service */
int ncn5120_get_comm_status(struct ncn5120_comm_status* status)
{
	uint8_t request = U_State_Request;
	uint8_t reply = 0;
	/* Send communication status request */
	ncn5120_send_service_request(&request, 1, 1);
	/* And wait for our data to come */
	ncn5120_get_req_reply_data(&reply, 1);
	if ((reply & U_State_Reply) != U_State_Reply) {
		return -1;
	}
	status->slave_colision = (reply & NCN5120_COMM_STATUS_SLAVE_COLISION);
	status->host_rx_error = (reply & NCN5120_COMM_STATUS_HOST_RX_ERROR);
	status->knx_tx_error = (reply & NCN5120_COMM_STATUS_KNX_TX_ERROR);
	status->protocol_error = (reply & NCN5120_COMM_STATUS_PROTOCOL_ERROR);
	status->thermal_warning = (reply & NCN5120_COMM_STATUS_THERMAL_WARNING);
	update_thermal_state(status->thermal_warning);
	return 0;
}

/* Get system status.
 * The 'out_of_sleep' and 'out_of_thermal_shutdown' field have no meaning for this request
 *    and will be '0' regardless of the real state.
 */
int ncn5120_get_system_status(struct ncn5120_status* status)
{
	uint8_t request = U_SystemStat_Request;
	uint8_t reply[2], reg_val = 0;
	/* Erase buffer, so that returned status is consistent */
	memset(status, 0, sizeof(struct ncn5120_status));
	/* Do not request system status if NCN5120 is currently receiving data */
	if (ncn5120.rx_data_running) {
		return -1;
	}
	/* Send system status request */
	ncn5120_send_service_request(&request, 1, 2);
	/* And wait for our data to come */
	ncn5120_get_req_reply_data(reply, 2);
	if (reply[0] != U_SystemStat_Reply) {
		return -2;
	}
	/* Decode operation mode */
	switch (reply[1] & NCN5120_OP_MODE_MASK) {
		case NCN5120_OP_MODE_POWER_UP:
			status->operation_mode = NCN5120_STATE_POWERUP;
			break;
		case NCN5120_OP_MODE_SYNC:
			status->operation_mode = NCN5120_STATE_SYNC;
			break;
		case NCN5120_OP_MODE_STOP:
			status->operation_mode = NCN5120_STATE_STOP;
			break;
		case NCN5120_OP_MODE_NORMAL:
			status->operation_mode = NCN5120_STATE_NORMAL;
			break;
	}
	ncn5120.current_op_mode = status->operation_mode;
	/* The bit field is left shifted by compared to internal register status bitfield ... shift back */
	reg_val = (reply[1] >> 1);
	status->v20v_ok = (reg_val & NCN5120_STATUS_V20V_OK);
	status->vdd2_ok = (reg_val & NCN5120_STATUS_VDD2_OK);
	status->vbus_ok = (reg_val & NCN5120_STATUS_VBUS_OK);
	status->vfilt_ok = (reg_val & NCN5120_STATUS_VFILT_OK);
	status->xtal_running = (reg_val & NCN5120_STATUS_XTAL_OK);
	status->thermal_warning = (reg_val & NCN5120_STATUS_THERMAL_WARN);
	update_thermal_state(status->thermal_warning);
	return 0;
}


/***************************************************************************** */
/* Request NCN5120 reset
 * Return 0 on success, -1 on error
 */
int ncn5120_reset(void)
{
	uint8_t request = U_Reset_Request;
	uint8_t reply = 0;

	/* Empty buffer before device request. Any data in it is not relevant anymore
	 * Drop all previously received command data */
	ncn5120_drop_cmd_data();

	ncn5120_send_service_request(&request, 1, 1);
	/* Update current state and clear rx_data_running flag */
	ncn5120.current_state = NCN5120_STATE_RESET;
	ncn5120.rx_data_running = 0;
	/* And wait for our data to come, which will mean that the device reset is done. */
	ncn5120_get_req_reply_data(&reply, 1);
	if (reply != U_Reset_Reply) {
		return -1;
	}
	ncn5120.current_state = NCN5120_STATE_NORMAL;
	return 0;
}

/* Enter sleep mode if it has been enabled by config in Analog Control register 0 (call to ncn5120_set_config()
 *    with sleep_en = 1).
 * Warning: when entering sleep mode, only a Wake-Up Event on the FANIN/WAKE-pin can get the
 *    NCN5120 out of sleep. Make sure your hardware alows the generation of this event.
 */
void ncn5120_enter_sleep(void)
{
	ncn5120_set_internal_register(NCN5120_ACR_1_ADDR, NCN5120_ENTER_SLEEP);
	ncn5120.current_state = NCN5120_STATE_SLEEP;
}

/* Set (enter or quit) busy mode.
 * Enter busy mode/state if 'state' is not 0.
 * In Busy mode and when autoacknowledge is active the NCN5120 rejects the frames whose
 *   destination address corresponds to the stored physical address by sending the BUSY
 *   acknowledge.
 * This service has no effect if autoacknowledge is not active.
 * Refer to p30 of NCN5120 documentation for busy state an p31 for autoacknowledge and set
 *   address service
 */
void ncn5120_set_busy_state(uint8_t state)
{
	uint8_t request = 0;

	if (state != 0) {
		request = U_SetBusy_Request;
		ncn5120.current_state = NCN5120_STATE_BUSY;
	} else {
		request = U_QuitBusy_Request;
		ncn5120.current_state = NCN5120_STATE_NORMAL;
	}
	ncn5120_send_service_request(&request, 1, 0);
}

/* Set (enter or quit) stop mode.
 * Enter stop mode/state if 'state' is not 0.
 */
void ncn5120_set_stop_state(uint8_t state)
{
	uint8_t request = 0;
	uint8_t reply = 0;
	
	if (state != 0) {
		request = U_StopMode_Request;
	} else {
		request = U_ExitStopMode_Request;
	}
	ncn5120_send_service_request(&request, 1, 1);
	/* And wait for confirmation. */
	ncn5120_get_req_reply_data(&reply, 1);
	if (state != 0) {
		if (reply == U_StopMode_Reply) {
			ncn5120.current_state = NCN5120_STATE_STOP;
		}
	} else {
		if (reply == U_Reset_Reply) {
			ncn5120.current_state = NCN5120_STATE_NORMAL;
		}
	}
	/* Seems that there cannot be other cases, only infinite loops waiting for an idle KNX Bus */
}


/* Enter Bus monitor mode
 * In this mode all data received from the KNX bus is sent to the host controller without
 *    performing any filtering on Data Link Layer. Acknowledge Frames are also transmitted
 *    transparently.
 * This state can only be exited by the Reset Service (ncn5120_reset()).
 */
void ncn5120_enter_monitor_mode(void)
{
	uint8_t request = U_Busmon_Request;
	ncn5120_send_service_request(&request, 1, 0);
	ncn5120.current_state = NCN5120_STATE_MONITOR;
}



/***************************************************************************** */
/* Receive packet helper
 * When an address got set using ncn5120_set_address() then the auto-ACK function is activated.
 * When it is not the case, the host is responsible for sending ACK, NACK or Busy on the Bus when
 *   it receives a packet
 */
void ncn5120_packet_acknowledge(uint8_t nack, uint8_t busy, uint8_t addressed)
{
	uint8_t request = U_Ackn_Request;

	if (ncn5120.address_set == 1) {
		return;
	}

	if (nack) {
		request |= NCN5120_ACK_NACK;
	}
	if (busy) {
		request |= NCN5120_ACK_BUSY;
	}
	if (addressed) {
		request |= NCN5120_ACK_ADDRESSED;
	}
	ncn5120_send_service_request(&request, 1, 0);
}


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
#define REQUEST_BUILDER_SIZE  3
int ncn5120_send_packet(uint8_t control_byte, uint8_t* buffer, uint16_t size)
{
	uint8_t request_builder[REQUEST_BUILDER_SIZE];
	uint16_t data_idx = 0;
	uint8_t reply[2]; /* In our config there are two bytes after the end of frame silence. */
	uint8_t checksum = 0;

	if (size > KNX_MAX_FRAME_SIZE) {
		return -1;
	}
	if (ncn5120.rx_data_running) {
		return -2;
	}

	/* Build the request */
	/* initial control part */
	request_builder[0] = U_L_DataStart_Request;
	request_builder[1] = control_byte;
	ncn5120_send_service_request(request_builder, 2, 0);

	/* Data part */
	while (data_idx < size) {
		if (!(data_idx & 0x3F)) {
			request_builder[0] = U_L_DataOffset_Request | ((data_idx >> 6) & 0x07);
			request_builder[1] = U_L_DataCont_Request | (data_idx & 0x3F);
			request_builder[2] = buffer[data_idx];
			ncn5120_send_service_request(request_builder, 3, 0);
		} else {
			request_builder[0] = U_L_DataCont_Request | (data_idx & 0x3F);
			request_builder[1] = buffer[data_idx];
			ncn5120_send_service_request(request_builder, 2, 0);
		}
		/* Compute checksum */
		/* FIXME : check KNX documentation */
		checksum += buffer[data_idx++];
	}

	/* End marker and checksum */
	request_builder[0] = U_L_DataEnd_Request | (size & 0x3F); /* Low six bits of last byte index */
	request_builder[1] = checksum;
	/* Send end marker and data checksum and set the right reply length.
	 * We always have these four : one for L_Data indicator (start), one for checksum, one for 
	 *   U_FrameState indicator (we are in 8bits UART mode), and one for L_Data confirmation */
	ncn5120_send_service_request(request_builder, 2, (size + 4));


	/* And wait for the echo and data end indicator and status information */
	ncn5120_get_req_reply_data(reply, 1);
	/* Is it L_Data indicator (OK) or U_State indicator (error) */
	if ((reply[0] & U_State_Reply) == U_State_Reply) {
		/* Error case */
		ncn5120_drop_cmd_data();
		/* FIXME : handle errors */
		return -3;
	}
		
	/* Read all echo data to get rid of it */
	/* Note : we should no use ncn5120. structure here, but this trick will have the memcpy return
	 *  immediatly as destination and source are the same. Add one for the checksum, no need to
	 *  check again.*/
	ncn5120_get_req_reply_data((uint8_t*)ncn5120.rx_cmd_buff, (size + 1));

	/* Read the remaining bytes (-2 for L_Data indicator and checksum) */
	ncn5120_get_req_reply_data(reply, 2);
	/* And decode */
	/* First byte is the state. We are sender, there are no receive error ... (should not be) */
	if (reply[0] != U_FrameState_Reply) {
		uprintf(0, "Tx but state indicates Rx errors .... (0x%02x)\n", reply[0]);
		return -4;
	}
	/* And the next one is the interesting one : Positive or negative ACK */
	if ((reply[1] & 0x7F) != L_Data) {
		uprintf(0, "Data Tx feedback does not end with L_Data confirmation byte ! (0x%02x)\n", reply[1]);
		return -5;
	}
	/* Did we get a positive or negative ACK ? */
	if (reply[1] & 0x80) {
		return 1; /* Positive ACK */
	}
	return 0; /* Negative ACK */
}


