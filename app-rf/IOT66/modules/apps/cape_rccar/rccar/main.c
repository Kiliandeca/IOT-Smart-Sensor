/****************************************************************************
 *   apps/dev/rccar/main.c
 *
 * Micro-controller support for the RC-Car Cape for BeagleBone Black
 *
 * Copyright 2016 Nathael Pajani <nathael.pajani@ed3l.fr>
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


#include "core/system.h"
#include "core/systick.h"
#include "core/pio.h"
#include "lib/stdio.h"
#include "drivers/serial.h"
#include "drivers/gpio.h"
#include "drivers/ssp.h"
#include "drivers/i2c.h"
#include "drivers/adc.h"
#include "drivers/timers.h"
#include "extdrv/cc1101.h"
#include "extdrv/status_led.h"
#include "extdrv/tmp101_temp_sensor.h"
#include "extdrv/ws2812.h"
#include "lib/protocols/dtplug/slave.h"


#define MODULE_VERSION	0x02
#define MODULE_NAME "Cape RC-Car"


#define RF_868MHz  1
#define RF_915MHz  0
#if ((RF_868MHz) + (RF_915MHz) != 1)
#error Either RF_868MHz or RF_915MHz MUST be defined.
#endif


#define DEBUG 0
#define UART_DEBUG  UART0
#define BUFF_LEN 60

#define SELECTED_FREQ  FREQ_SEL_48MHz

#define LPC_TIMER_PIN_CONFIG   (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL | LPC_IO_DRIVE_HIGHCURENT)

/***************************************************************************** */
/* Pins configuration */
/* pins blocks are passed to set_pins() for pins configuration.
 * Unused pin blocks can be removed safely with the corresponding set_pins() call
 * All pins blocks may be safelly merged in a single block for single set_pins() call..
 */
const struct pio_config common_pins[] = {
	/* UART 0 */
	{ LPC_UART0_RX_PIO_0_1,  LPC_IO_DIGITAL },
	{ LPC_UART0_TX_PIO_0_2,  LPC_IO_DIGITAL },
	/* UART 1 */
	{ LPC_UART1_RX_PIO_0_8,  LPC_IO_DIGITAL },
	{ LPC_UART1_TX_PIO_0_9,  LPC_IO_DIGITAL },
	/* I2C 0 */
	{ LPC_I2C0_SCL_PIO_0_10, (LPC_IO_DIGITAL | LPC_IO_OPEN_DRAIN_ENABLE) },
	{ LPC_I2C0_SDA_PIO_0_11, (LPC_IO_DIGITAL | LPC_IO_OPEN_DRAIN_ENABLE) },
	/* SPI */
	{ LPC_SSP0_SCLK_PIO_0_14, LPC_IO_DIGITAL },
	{ LPC_SSP0_MOSI_PIO_0_17, LPC_IO_DIGITAL },
	{ LPC_SSP0_MISO_PIO_0_16, LPC_IO_DIGITAL },
	/* TIMER_32B0 - PWM */
	{ LPC_TIMER_32B0_M0_PIO_0_18, LPC_TIMER_PIN_CONFIG },
	{ LPC_TIMER_32B0_M1_PIO_0_19, LPC_TIMER_PIN_CONFIG },
	{ LPC_TIMER_32B0_M2_PIO_0_20, LPC_TIMER_PIN_CONFIG },
	/* TIMER_32B1 - PWM */
	{ LPC_TIMER_32B1_M0_PIO_0_23, LPC_TIMER_PIN_CONFIG },
	{ LPC_TIMER_32B1_M1_PIO_0_24, LPC_TIMER_PIN_CONFIG },
	{ LPC_TIMER_32B1_M2_PIO_0_25, LPC_TIMER_PIN_CONFIG },
	/* ADC */
	{ LPC_ADC_AD0_PIO_0_30, LPC_IO_ANALOG },
	{ LPC_ADC_AD1_PIO_0_31, LPC_IO_ANALOG },
	{ LPC_ADC_AD2_PIO_1_0,  LPC_IO_ANALOG },
	/* GPIO */
	{ LPC_GPIO_0_3, LPC_IO_DIGITAL},
	{ LPC_GPIO_0_4, LPC_IO_DIGITAL},
	{ LPC_GPIO_0_5, LPC_IO_DIGITAL},
	{ LPC_GPIO_0_6, LPC_IO_DIGITAL},
	{ LPC_GPIO_0_7, LPC_IO_DIGITAL},
	{ LPC_GPIO_0_21, LPC_IO_DIGITAL},
	{ LPC_GPIO_0_22, LPC_IO_DIGITAL},
	{ LPC_GPIO_0_26, LPC_IO_DIGITAL},
	{ LPC_GPIO_0_27, LPC_IO_DIGITAL},
	{ LPC_GPIO_0_28, LPC_IO_DIGITAL},
	{ LPC_GPIO_0_29, LPC_IO_DIGITAL},
	ARRAY_LAST_PIO,
};

/* RF - CC1101 */
const struct pio cc1101_cs_pin = LPC_GPIO_0_6;
const struct pio cc1101_miso_pin = LPC_SSP0_MISO_PIO_0_16;
const struct pio cc1101_gdo0 = LPC_GPIO_0_7;
/* 9D - LSM9DSO sensor */
const struct pio lsm9d_cs_pin = LPC_GPIO_0_5;

/* On-board I2C temperature sensor */
#define TMP101_ADDR  0x94  /* Pin Addr0 (pin5 of tmp101) connected to VCC */
struct tmp101_sensor_config tmp101_sensor = {
	.bus_num = I2C0,
	.addr = TMP101_ADDR,
	.resolution = TMP_RES_ELEVEN_BITS,
};

/* Servomotor functions definition by servo numbers */
#define DIRECTION_CTRL_SERVO  0
#define DIRECTION_CTRL_SERVO_MIN 60
#define DIRECTION_CTRL_SERVO_MAX 120
#define SPEED_CTRL_SERVO      1
#define SPEED_CTRL_SERVO_MIN 60
#define SPEED_CTRL_SERVO_MAX 110

#define DIST_SLOW 400
#define DIST_STOP 200
#define SLOW_MOTION 98
#define HALT 86
uint8_t max_forward_speed = SPEED_CTRL_SERVO_MAX;

/* Lights - WS2812B Leds strip */
const struct pio leds = LPC_GPIO_0_3;

/* External movement detector */
const struct pio ext_move_pin = LPC_GPIO_0_4;

/* Ultrasound sensors */
#define NUM_PULSE_SENSORS  6
const struct pio us_pulse_sensors[NUM_PULSE_SENSORS] = {
	LPC_GPIO_0_21, LPC_GPIO_0_22, LPC_GPIO_0_26,
	LPC_GPIO_0_27, LPC_GPIO_0_28, LPC_GPIO_0_29,
};

/* Status Led */
const struct pio status_led_green = LPC_GPIO_1_4;
const struct pio status_led_red = LPC_GPIO_1_5;


#define ADC_SMOKE  LPC_ADC(0)
#define ADC_VBAT1  LPC_ADC(1)
#define ADC_VBAT2  LPC_ADC(2)
#define ALL_BOARD_ADC   (ADC_MCH(ADC_SMOKE) | ADC_MCH(ADC_VBAT1) | ADC_MCH(ADC_VBAT2))

/***************************************************************************** */
void system_init()
{
	/* Stop the watchdog */
	startup_watchdog_disable(); /* Do it right now, before it gets a chance to break in */
	system_set_default_power_state();
	clock_config(SELECTED_FREQ);
	set_pins(common_pins);
	gpio_on();
	/* System tick timer MUST be configured and running in order to use the sleeping
	 * functions */
	systick_timer_on(1); /* 1ms */
	systick_start();
}

/* Define our fault handler. This one is not mandatory, the dummy fault handler
 * will be used when it's not overridden here.
 * Note : The default one does a simple infinite loop. If the watchdog is deactivated
 * the system will hang.
 */
void fault_info(const char* name, uint32_t len)
{
	uprintf(UART1, name);
	while (1);
}



/***************************************************************************** */
/* Temperature */
/* The I2C Temperature sensor is at address 0x94 */
void temp_config(int uart_num)
{
	int ret = 0;
	ret = tmp101_sensor_config(&tmp101_sensor);
	if (ret != 0) {
#if (DEBUG == 1)
		uprintf(uart_num, "Temp config error\n");
#endif
	}
}


/******************************************************************************/
/* Servo-motors */
static uint32_t servo_med_pos_cmd = 0;
static uint32_t servo_one_deg_step = 0;
int servo_config(uint8_t uart_num)
{
	uint32_t servo_command_period = 0;
	struct lpc_timer_pwm_config timer_conf = {
		.nb_channels = 3,
		.period_chan = 3,
		.outputs = { 0, 1, 2, },
	};

	/* compute the period and median position for the servo command */
	/* We want 20ms (50Hz), timer counts at main clock frequency */
	servo_command_period = get_main_clock() / 50;
	/* servo_command_period is 20ms, we need 1.5ms, which is 3/40. */
	servo_med_pos_cmd = ((servo_command_period / 40) * 3);
	servo_one_deg_step = ((servo_command_period / 41) / 48);

	timer_conf.match_values[0] = servo_med_pos_cmd;
	timer_conf.match_values[1] = servo_med_pos_cmd;
	timer_conf.match_values[2] = servo_med_pos_cmd;
	timer_conf.period = servo_command_period;

    timer_on(LPC_TIMER_32B0, 0, NULL);
	timer_pwm_config(LPC_TIMER_32B0, &timer_conf);
	timer_start(LPC_TIMER_32B0);

    timer_on(LPC_TIMER_32B1, 0, NULL);
	timer_pwm_config(LPC_TIMER_32B1, &timer_conf);
	timer_start(LPC_TIMER_32B1);

#if (DEBUG == 1)
	uprintf(uart_num, "Servos configured, Period : %d, med_pos : %d\n",
							servo_command_period, servo_med_pos_cmd);
#endif
	return 0;
}

int set_servo(int uart_num, int servo_num, int angle)
{
	uint32_t pos = servo_med_pos_cmd;
	int timer = (servo_num < 3) ? LPC_TIMER_32B0 : LPC_TIMER_32B1;
	int channel = (servo_num % 3);

	if (angle > 180) {
		angle = 180;
	}

	/* And compute the new match value for the angle */
	if (angle >= 90) {
		pos += ((angle - 90) * servo_one_deg_step);
	} else {
		pos -= ((90 - angle) * servo_one_deg_step);
	}
	timer_set_match(timer, channel, pos);
#if (DEBUG == 1)
	uprintf(uart_num, "Servo(%d - %d.%d): %d (%d)\n", servo_num, timer, channel, angle, pos);
#endif
	return pos;
}


/******************************************************************************/
/* Ultrasonic distance sensors */

/* Delay between measures should be at least 50ms  */
#define PULSE_DELAY 50
enum pulse_states {
	PULSE_STATE_NONE = 0,
	PULSE_STATE_MUST_TRIGGER,
	PULSE_STATE_TRIG_DELAY,
	PULSE_STATE_TRIG_START,
	PULSE_STATE_TRIG_DONE,
	PULSE_STATE_PULSE_START,
	PULSE_STATE_PULSE_END,
};

/* Note that clock cycles counter wraps every 89 seconds with system clock running at 48 MHz */
static volatile uint32_t pulse_start[NUM_PULSE_SENSORS];  /* Clock cycles counter upon echo start */
static volatile uint32_t pulse_duration[NUM_PULSE_SENSORS];
static volatile uint8_t pulse_state[6] = {0};
void pulse_feedback(uint32_t gpio)
{
	int sensor_num = 0;
	uint32_t now = systick_get_clock_cycles();
	/* Find sensor number */
	do {
		if (us_pulse_sensors[sensor_num].pin == gpio) {
			break;
		}
	} while (++sensor_num < NUM_PULSE_SENSORS);
	if (sensor_num >= NUM_PULSE_SENSORS) {
		return;
	}
	
	if (pulse_state[sensor_num] == PULSE_STATE_TRIG_DONE) {
		pulse_start[sensor_num] = now;
		pulse_state[sensor_num] = PULSE_STATE_PULSE_START;
	} else if (pulse_state[sensor_num] == PULSE_STATE_PULSE_START) {
		if (now > pulse_start[sensor_num]) {
			pulse_duration[sensor_num] = (now - pulse_start[sensor_num]);
		} else {
			pulse_duration[sensor_num] = (0xFFFFFFFF - pulse_start[sensor_num]);
			pulse_duration[sensor_num] += now;
		}
		pulse_state[sensor_num] = PULSE_STATE_PULSE_END;
		/* Remove GPIO interrupt, we got what we wanted, do not trig interrupt on
		 * sensor trigger pulse. */
		remove_gpio_callback(&us_pulse_sensors[sensor_num]);
	}
}

void ultrasound_sensors_trig_timer_int(uint32_t flags)
{
	int i = 0;
	for (i = 0; i < NUM_PULSE_SENSORS; i++) {
		switch (pulse_state[i]) {
			case PULSE_STATE_TRIG_DELAY:
				gpio_set(us_pulse_sensors[i]);
				pulse_state[i] = PULSE_STATE_TRIG_START;
				/* And restart timer for the high state */
				timer_restart(LPC_TIMER_16B0);
				break;
			case PULSE_STATE_TRIG_START:
				gpio_clear(us_pulse_sensors[i]);
				gpio_dir_in(us_pulse_sensors[i]);
				pulse_state[i] = PULSE_STATE_TRIG_DONE;
				/* Re-activate GPIO interrupt */
				set_gpio_callback(pulse_feedback, &(us_pulse_sensors[i]), EDGES_BOTH);
				break;
		}
	}
}

/* Initiate distance mesurement */
void pulse_trig(int sensor_num)
{
	/* Set pin as output and clear for 10us at least */
	gpio_dir_out(us_pulse_sensors[sensor_num]);
	gpio_clear(us_pulse_sensors[sensor_num]);
	pulse_state[sensor_num] = PULSE_STATE_TRIG_DELAY;
	/* Start or restart timer to generate event.
	 * If timer was already started, it will restart timer thus we get a delay of 10 to 60us
	 *   depending on how many times this function is called and when.
	 */
	timer_restart(LPC_TIMER_16B0);
}

static volatile uint16_t distances[NUM_PULSE_SENSORS] = {0};
uint32_t div = 0;
void ultrasound_sensors_update(uint32_t tick)
{
	static int i = 0;
	uint32_t distance_tmp = 0;

	if (pulse_state[i] == PULSE_STATE_PULSE_END) {
		/* Convert pulse width in us to distance in mm */
		distance_tmp = (((pulse_duration[i] * 10) / div) / 29);
		distances[i] = (uint16_t)byte_swap_16((uint16_t)distance_tmp);
		pulse_state[i] = PULSE_STATE_MUST_TRIGGER;
	}
	/* Always trigger a new round, this prevents being stuck somewhere */
	pulse_trig(i);

	/* Next time we will trigger the next sensor */
	i = ((i + 1) % NUM_PULSE_SENSORS);
}

int ultrasound_sensors_config(int uart_num)
{
	struct lpc_tc_config timer_conf = {
		.mode = (LPC_TIMER_MODE_TIMER | LPC_TIMER_MODE_MATCH),
		.match_control = { LPC_TIMER_INT_RESET_AND_STOP_ON_MATCH, 0, 0, 0, },
	};
	uint32_t match = get_main_clock() / (50 * 1000); /* 5 us */

	/* Set divisor only once : main clock must not be changed. */
	div = (get_main_clock() / (1000*1000));

	/* Add a callback to trigger the range measurement every 50 to 100 ms */
	add_systick_callback(ultrasound_sensors_update, 40);

	timer_conf.match[0] = match;
    timer_on(LPC_TIMER_16B0, 0, ultrasound_sensors_trig_timer_int);
	timer_counter_config(LPC_TIMER_16B0, &timer_conf);

#if (DEBUG == 1)
	uprintf(uart_num, "Ultrasound distance sensors configured.\n");
#endif
	return 0;
}


/******************************************************************************/
/* RF Communication */
#define RF_BUFF_LEN  64

static volatile uint32_t cc_tx = 0;
static volatile uint8_t cc_tx_buff[RF_BUFF_LEN];
static volatile uint8_t cc_ptr = 0;

static volatile int check_rx = 0;
void rf_rx_calback(uint32_t gpio)
{
	check_rx = 1;
}

static uint8_t rf_specific_settings[] = {
	CC1101_REGS(gdo_config[2]), 0x07, /* GDO_0 - Assert on CRC OK | Disable temp sensor */
	CC1101_REGS(gdo_config[0]), 0x2E, /* GDO_2 - FIXME : do something usefull with it for tests */
	CC1101_REGS(pkt_ctrl[0]), 0x0F, /* Accept all sync, CRC err auto flush, Append, Addr check and Bcast */
};

/* RF config */
void rf_config(void)
{
	config_gpio(&cc1101_gdo0, LPC_IO_MODE_PULL_UP, GPIO_DIR_IN, 0);
	cc1101_init(0, &cc1101_cs_pin, &cc1101_miso_pin); /* ssp_num, cs_pin, miso_pin */
	/* Set default config */
	cc1101_config();
	/* And change application specific settings */
	cc1101_update_config(rf_specific_settings, sizeof(rf_specific_settings));
	set_gpio_callback(rf_rx_calback, &cc1101_gdo0, EDGE_RISING);

#if (DEBUG == 1)
	uprintf(UART_DEBUG, "CC1101 RF link init done.\n");
#endif
}

void handle_rf_rx_data(void)
{
	uint8_t data[RF_BUFF_LEN];
	int8_t ret = 0;
	uint8_t status = 0;

	/* Check for received packet (and get it if any) */
	ret = cc1101_receive_packet(data, RF_BUFF_LEN, &status);
	/* Go back to RX mode */
	cc1101_enter_rx_mode();

#if (DEBUG == 1)
	uprintf(UART_DEBUG, "RF: ret:%d, st: %d.\n", ret, status);
#endif

	if (ret <= 0) {
		return;
	}

	switch (data[2]) {
		case 'D':
			if (data[3] > DIRECTION_CTRL_SERVO_MAX) {
				data[3] = DIRECTION_CTRL_SERVO_MAX;
			}
			if (data[3] < DIRECTION_CTRL_SERVO_MIN) {
				data[3] = DIRECTION_CTRL_SERVO_MIN;
			}
			set_servo(UART_DEBUG, DIRECTION_CTRL_SERVO, data[3]);
			break;
		case 'S':
			if (data[3] > max_forward_speed) {
				data[3] = max_forward_speed;
			}
			set_servo(UART_DEBUG, SPEED_CTRL_SERVO, data[3]);
			break;
		default:
			uprintf(UART_DEBUG, "Received unhandled command on RF\n");
			break;
	}
}

void send_on_rf(void)
{
    uint8_t cc_tx_data[RF_BUFF_LEN + 2];
    uint8_t tx_len = cc_ptr;
    int ret = 0;

    /* Create a local copy */
    memcpy((char*)&(cc_tx_data[2]), (char*)cc_tx_buff, tx_len);
    /* "Free" the rx buffer as soon as possible */
    cc_ptr = 0;
    /* Prepare buffer for sending */
    cc_tx_data[0] = tx_len + 1;
    cc_tx_data[1] = 0; /* Broadcast */
    /* Send */
    if (cc1101_tx_fifo_state() != 0) {
        cc1101_flush_tx_fifo();
    }
    ret = cc1101_send_packet(cc_tx_data, (tx_len + 2));

#if (DEBUG == 1)
	/* Give some feedback on UART 0 */
	uprintf(UART_DEBUG, "Tx ret: %d\n", ret);
#endif
	if (ret <= 0) {
		return;
	}
}



/******************************************************************************/
/* Serial Communication */

static struct dtplug_protocol_handle uart_handle;

void quickdata_to_packet(struct packet* pkt)
{
	struct header* head = &(pkt->info);
	if (head->seq_num & QUICK_DATA_PACKET) {
		pkt->data[0] = head->quick_data[0];
		pkt->data[1] = head->quick_data[1];
		if (head->seq_num & QUICK_DATA_PACKET_ONE_BYTE) {
			head->data.size = 1;
		} else {
			head->data.size = 2;
		}
	}
}


#define PKT_TYPE_GET_DISTANCES    (PKT_TYPE_LAST + 40)
#define PKT_TYPE_GET_SMOKE        (PKT_TYPE_LAST + 41)
#define PKT_TYPE_GET_BATTERY      (PKT_TYPE_LAST + 42)
#define PKT_TYPE_GET_IR           (PKT_TYPE_LAST + 43)

#define PKT_TYPE_SET_SPEED        (PKT_TYPE_LAST + 50)
#define PKT_TYPE_SET_DIRECTION    (PKT_TYPE_LAST + 51)

int board_temp = 0;
uint16_t adc_smoke = 0;

void handle_commands(struct packet* command)
{
	struct header* head = &(command->info);
	uint8_t* data = command->data;
	uint16_t value = 0;
	int i = 0;
	/* Transform quick data packets to standard packets */
	quickdata_to_packet(command);

	/* And handle commands */
	switch (head->type) {
		case PKT_TYPE_GET_TEMPERATURE:
			value = (int16_t)board_temp;
			value = (uint16_t)byte_swap_16(value);
			dtplug_protocol_send_reply(&uart_handle, command, NO_ERROR, 2, (uint8_t*)(&value));
			break;
		case PKT_TYPE_GET_ADC_VALUE:
			break;
		case PKT_TYPE_GET_SMOKE:
			value = (uint16_t)byte_swap_16(adc_smoke);
			dtplug_protocol_send_reply(&uart_handle, command, NO_ERROR, 2, (uint8_t*)(&value));
			break;
		case PKT_TYPE_GET_DISTANCES:
			dtplug_protocol_send_reply(&uart_handle, command, NO_ERROR, (2 * NUM_PULSE_SENSORS), (uint8_t*)(&distances));
			break;
		case PKT_TYPE_SET_RGB_LED:
			for (i = 0; i < head->data.size; i += 4) {
				ws2812_set_pixel(data[i], data[i + 1], data[i + 2], data[i + 3]);
				uprintf(UART_DEBUG, "Set led %d to {%d, %d, %d}.\n", data[i], data[i + 1], data[i + 2], data[i + 3]);
			}
			uprintf(UART_DEBUG, "Set %d leds.\n", (i / 4));
			break;
		case PKT_TYPE_CLEAR_LEDS:
			uprintf(UART_DEBUG, "All leds cleared.\n");
			ws2812_clear();
			break;
		case PKT_TYPE_SET_DIRECTION:
			if (data[0] > DIRECTION_CTRL_SERVO_MAX) {
				data[0] = DIRECTION_CTRL_SERVO_MAX;
			}
			if (data[0] < DIRECTION_CTRL_SERVO_MIN) {
				data[0] = DIRECTION_CTRL_SERVO_MIN;
			}
			set_servo(UART_DEBUG, DIRECTION_CTRL_SERVO, data[0]);
			uprintf(UART_DEBUG, "Set direction to %d.\n", data[0]);
			break;
		case PKT_TYPE_SET_SPEED:
			if (data[0] > SPEED_CTRL_SERVO_MAX) {
				data[0] = SPEED_CTRL_SERVO_MAX;
			}
			if (data[0] < SPEED_CTRL_SERVO_MIN) {
				data[0] = SPEED_CTRL_SERVO_MIN;
			}
			if (data[0] > max_forward_speed) {
				data[0] = max_forward_speed;
			}
			set_servo(UART_DEBUG, SPEED_CTRL_SERVO, data[0]);
			uprintf(UART_DEBUG, "Set speed to %d.\n", data[0]);
			break;
		case PKT_TYPE_SET_PWM_CHAN:
			set_servo(UART_DEBUG, 2, (distances[2] / 10));
			break;
		default:
			/* FIXME : dtplug_protocol_common_handles(&uart_handle, command); */
			uprintf(UART_DEBUG, "Unknown packet type (%d): packet not handled\n", head->type);
			dtplug_protocol_send_reply(&uart_handle, command, ERROR_PKT_NOT_HANDLED, 0, NULL);
			status_led(red_on);
			break;
	}
	dtplug_protocol_release_old_packet(&uart_handle);
}

void dtplug_protocol_send_info(struct dtplug_protocol_handle* handle, uint8_t type, int len, uint8_t* data)
{
	struct packet sensors = {
		.info = {
			.start = FIRST_PACKET_CHAR,
			.type = type,
			.seq_num = PACKET_NEEDS_REPLY,  /* Packet sequence number of 0 for unrequested packets */
		},
	};

	dtplug_protocol_send_reply(handle, &sensors, NO_ERROR, len, data);
}

/***************************************************************************** */
int main(void)
{
	system_init();

	dtplug_protocol_set_dtplug_comm_uart(UART0, &uart_handle);
	uart_on(UART1, 115200, NULL);

	ssp_master_on(0, LPC_SSP_FRAME_SPI, 8, 4*1000*1000); /* bus_num, frame_type, data_width, rate */
	i2c_on(I2C0, I2C_CLK_100KHz, I2C_MASTER);
	adc_on(NULL);
	status_led_config(&status_led_green, &status_led_red);

	/* Radio */
	rf_config();

	/* Temperature sensor */
	temp_config(UART_DEBUG);

	/* Ultrasonic distance sensors */
	ultrasound_sensors_config(UART_DEBUG);

	/* Smoke sensor and ADCs */
	adc_start_burst_conversion(ALL_BOARD_ADC, LPC_ADC_SEQ(0));

	/* Servo motors */
	msleep(2000);
	servo_config(UART_DEBUG);

	/* Config Leds for lights */
	ws2812_config(&leds);

	while (1) {
		struct packet* pkt = NULL;
		uint8_t status = 0;

		/* Request a Temp conversion on I2C TMP101 temperature sensor */
		tmp101_sensor_start_conversion(&tmp101_sensor); /* A conversion takes about 40ms */

		/* Tell we are alive :) */
		chenillard(10);

		/* Read the temperature */
		if (tmp101_sensor_read(&tmp101_sensor, NULL, &board_temp) != 0) {
#if (DEBUG == 1)
			uprintf(UART_DEBUG, "Temp read error\n");
		} else {
			uprintf(UART_DEBUG, "Temp read: %d,%d.\n", (board_temp / 10), (board_temp % 10));
#endif
		}

		/* Get and display the smoke detector voltage */
		if (adc_get_value(&adc_smoke, ADC_SMOKE) >= 0) {
			int milli_volts = ((adc_smoke * 32) / 5);
			adc_smoke = (uint16_t)milli_volts;
#if (DEBUG == 1)
			uprintf(UART_DEBUG , "Vsmoke: %d\n", adc_smoke);
#endif
		}

#if (DEBUG == 1)
		uprintf(UART_DEBUG, "D: %d - %d - %d - %d - %d - %d\n",
					byte_swap_16(distances[0]), byte_swap_16(distances[1]), byte_swap_16(distances[2]),
					byte_swap_16(distances[3]), byte_swap_16(distances[4]), byte_swap_16(distances[5]));
#endif
		if ((distances[3] < DIST_STOP) || (distances[4] < DIST_STOP) || (distances[5] < DIST_STOP)) {
			max_forward_speed = HALT;
		} else if ((distances[3] < DIST_SLOW) || (distances[4] < DIST_SLOW) || (distances[5] < DIST_SLOW)) {
			max_forward_speed = SLOW_MOTION;
		} else {
			max_forward_speed = SPEED_CTRL_SERVO_MAX;
		}

		/* Handle commands */
		pkt = dtplug_protocol_get_next_packet_ok(&uart_handle);
		if (pkt != NULL) {
			handle_commands(pkt);
		}

		/* Send leds frame if leds changed */
		ws2812_send_frame(0);

		/* RF */
		if (cc_tx == 1) {
			send_on_rf();
			cc_tx = 0;
		}
		/* Do not leave radio in an unknown or unwated state */
        do {
            status = (cc1101_read_status() & CC1101_STATE_MASK);
        } while (status == CC1101_STATE_TX);

		if (status != CC1101_STATE_RX) {
			if (cc1101_rx_fifo_state() != 0) {
				cc1101_flush_rx_fifo();
			}
			cc1101_enter_rx_mode();
		}
		if (check_rx == 1) {
			check_rx = 0;
			handle_rf_rx_data();
		}
	}
	return 0;
}




