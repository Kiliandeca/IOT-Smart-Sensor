/****************************************************************************
 *   apps/scialys/beta/main.c
 *
 * Scialys system for solar-panel power generation tracking and fair use.
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
#include "lib/errno.h"
#include "drivers/serial.h"
#include "drivers/gpio.h"
#include "drivers/adc.h"
#include "drivers/ssp.h"
#include "drivers/i2c.h"
#include "drivers/timers.h"

#include "extdrv/status_led.h"
#include "extdrv/ws2812.h"
#include "extdrv/max31855_thermocouple.h"
#include "extdrv/tmp101_temp_sensor.h"
#include "extdrv/rtc_pcf85363a.h"
#include "extdrv/ssd130x_oled_driver.h"
#include "extdrv/ssd130x_oled_buffer.h"
#include "lib/font.h"
#include "lib/time.h"

#include "extdrv/sdmmc.h"

#define MODULE_VERSION    0x01
#define MODULE_NAME "Scialys uC"


#define SELECTED_FREQ  FREQ_SEL_48MHz


/***************************************************************************** */
/* System configuration
 * Most of the defines in here should go to configuration setting in user flash
 */

/* Period of the decrementer handler from the systick interrupt */
#define DEC_PERIOD  100

/* If temperature falls bellow FORCE_HEATER_TEMP value, we enter forced heater mode, until
 *    TARGET_FORCED_HEATER_TEMP is reached.
 * When in forced heater mode, the heater is controlled to heat at FORCED_MODE_VALUE which
 *    is between 0 and 100.
 */
#define FORCE_HEATER_TEMP  28
#define TARGET_FORCED_HEATER_TEMP 32
#define FORCED_MODE_VALUE  75 /* A fraction of 100 */
/* mA prod value above which the system will not enter forced mode, waiting for home
 * to stop using power to start automatic heating */
#define NO_FORCED_HEATING_ON_SUNNY_DAYS 750

uint32_t forced_heater_mode = 0;
uint32_t forced_heater_delay = 0;
uint32_t forced_heater_time = 0;

#define FORCED_HEATER_DELAY      (2 * 3600 * 1000 / DEC_PERIOD)  /* Delay before automatic forced heating */
#define FORCED_HEATER_DURATION   (3 * 3600 * 1000 / DEC_PERIOD)  /* Duration of automatic forced heating */

#define MANUAL_ACTIVATION_DURATION   (3 * 3600 * 1000 / DEC_PERIOD)  /* Three hours */

uint32_t never_force = 0;


#define DAY_IS_EJP  0  /* Input is pulled low when EJP is ON */
int ejp_in = 0;



/***************************************************************************** */
/* Pins configuration */
/* pins blocks are passed to set_pins() for pins configuration.
 * Unused pin blocks can be removed safely with the corresponding set_pins() call
 * All pins blocks may be safelly merged in a single block for single set_pins() call..
 */
const struct pio_config common_pins[] = {
	/* UART 0 : Config / Debug / USB */
	{ LPC_UART0_RX_PIO_0_1,  LPC_IO_DIGITAL },
	{ LPC_UART0_TX_PIO_0_2,  LPC_IO_DIGITAL },
	/* UART 1 : UEXT */
	{ LPC_UART1_RX_PIO_0_8,  LPC_IO_DIGITAL },
	{ LPC_UART1_TX_PIO_0_9,  LPC_IO_DIGITAL },
	/* I2C : RTC, Display, UEXT */
	{ LPC_I2C0_SCL_PIO_0_10, (LPC_IO_DIGITAL | LPC_IO_OPEN_DRAIN_ENABLE) },
	{ LPC_I2C0_SDA_PIO_0_11, (LPC_IO_DIGITAL | LPC_IO_OPEN_DRAIN_ENABLE) },
	/* SPI (Thermocouple + uSD card + UEXT) */
	{ LPC_SSP0_SCLK_PIO_0_14, LPC_IO_DIGITAL },
	{ LPC_SSP0_MOSI_PIO_0_17, LPC_IO_DIGITAL },
	{ LPC_SSP0_MISO_PIO_0_16, LPC_IO_DIGITAL },
	/* TIMER_32B0 */
	{ LPC_TIMER_32B0_M0_PIO_0_18, (LPC_IO_MODE_PULL_UP | LPC_IO_DIGITAL) }, /* Fan control */
	/* GPIO */
	{ LPC_GPIO_0_0, LPC_IO_DIGITAL },  /* Clkout / interrupt from RTC */
	{ LPC_GPIO_0_3, LPC_IO_DIGITAL },  /* EJP / External switch input */
	{ LPC_GPIO_0_4, LPC_IO_DIGITAL },  /* Zero crossing detection input */
	{ LPC_GPIO_0_5, LPC_IO_DIGITAL },  /* Temperature driver warning (Mosfet board only) */
	{ LPC_GPIO_0_6, LPC_IO_DIGITAL },  /* Mosfet driver Shutdown (Mosfet board only) */
	{ LPC_GPIO_0_7, LPC_IO_DIGITAL },  /* Mosfet / Triac control */
	{ LPC_GPIO_0_12, LPC_IO_DIGITAL }, /* ISP / User button OK */
	{ LPC_GPIO_0_15, LPC_IO_DIGITAL }, /* Thermocouple chip select */
	{ LPC_GPIO_0_23, LPC_IO_DIGITAL }, /* WS2812B RGB Leds control */
	{ LPC_GPIO_0_26, LPC_IO_DIGITAL }, /* User button B2 */
	{ LPC_GPIO_0_27, LPC_IO_DIGITAL }, /* User button B1 */
	{ LPC_GPIO_0_28, LPC_IO_DIGITAL }, /* Charge State */
	{ LPC_GPIO_1_1, LPC_IO_DIGITAL },  /* Uext Chip select / Module eeprom select */
	{ LPC_GPIO_1_6, LPC_IO_DIGITAL },  /* uSD Card SPI Chip Select */
	ARRAY_LAST_PIO,
};

const struct pio_config adc_pins[] = {
	{ LPC_ADC_AD0_PIO_0_30, LPC_IO_ANALOG },  /* ADC0 */
	{ LPC_ADC_AD1_PIO_0_31, LPC_IO_ANALOG },  /* ADC1 */
	{ LPC_ADC_AD2_PIO_1_0, LPC_IO_ANALOG },   /* ADC2 */
	{ LPC_ADC_AD7_PIO_1_5, LPC_IO_ANALOG },   /* ADC3 */
	ARRAY_LAST_PIO,
};

const struct pio status_led_green = LPC_GPIO_1_2;
const struct pio status_led_red = LPC_GPIO_1_3;

/* Inputs */
/* Buttons */
const struct pio button_ok = LPC_GPIO_0_12;
const struct pio button_b1 = LPC_GPIO_0_27;
const struct pio button_b2 = LPC_GPIO_0_26;
/* External signals */
const struct pio rtc_in_pin = LPC_GPIO_0_0;
const struct pio ejp_in_pin = LPC_GPIO_0_3;
const struct pio zero_cross_in_pin = LPC_GPIO_0_4;
const struct pio th_warn_in_pin = LPC_GPIO_0_5;
const struct pio charge_status_in_pin = LPC_GPIO_0_28;

/* Outputs */
/* Led control data pin */
const struct pio ws2812_data_out_pin = LPC_GPIO_0_23;
/* AC output control (Mosfet / Triac) */
const struct pio ac_ctrl = LPC_GPIO_0_7;
/* Fixme : Fan */


/* Thermocouple reading */
const struct max31855_sensor_config thermo = {
	.ssp_bus_num = 0,
	.chip_select = LPC_GPIO_0_15,
};

/* TMP101 onboard I2C temperature sensor */
#define TMP101_ADDR  0x94  /* Pin Addr0 (pin5 of tmp101) connected to VCC */
struct tmp101_sensor_config tmp101_sensor = {
	.bus_num = I2C0,
	.addr = TMP101_ADDR,
	.resolution = TMP_RES_ELEVEN_BITS,
};


#define FAN_ON    (10 * 1000)
const struct lpc_timer_pwm_config fan_pwm_conf = {
	.nb_channels = 1,
	.period_chan = CHAN3,
	.period = FAN_ON,
	.outputs = { CHAN0, },
	.match_values = { 0, },
};

const struct lpc_tc_config ac_timer_conf = {
	.mode = LPC_TIMER_MODE_TIMER | LPC_TIMER_MODE_MATCH,
	.match_control = { 0, LPC_TIMER_INT_RESET_AND_STOP_ON_MATCH, 0, 0, },
	.match = { 0, 10, 0, 0, },
	.ext_match_config = { 0, LPC_TIMER_SET_ON_MATCH, 0, 0, },
};


/***************************************************************************** */
/* SD/MMC Card */
struct sdmmc_card micro_sd = {
	.ssp_bus_num = SSP_BUS_0,
	.card_type = MMC_CARDTYPE_UNKNOWN,
	.block_size = 64,
	.chip_select = LPC_GPIO_1_6,
};

uint8_t mmc_data[MMC_MAX_SECTOR_SIZE];


/***************************************************************************** */
/* RTC and time */
#define	RTC_ADDR   0xA2
struct rtc_pcf85363a_config rtc_conf = {
	.bus_num = I2C0,
	.addr = RTC_ADDR,
	.mode = PCF85363A_MODE_RTC,
	.config_marker = PCF85363A_CONFIGURED_1,
	.batt_ctrl = PCF85363A_CONF_BATT_TH_2_8V,
};
/* Oldest acceptable time in RTC. BCD coded. */
const struct rtc_time oldest = {
	.year = 0x16,
	.month = 0x12,
	.day = 0x01,
	.hour = 0x13,
	.min = 0x37,
};
static struct rtc_time now;


/***************************************************************************** */
/* Basic system init and configuration */
static volatile int got_wdt_int = 0;
void wdt_callback(void)
{
	got_wdt_int = 1;
}

const struct wdt_config wdconf = {
	.clk_sel = WDT_CLK_IRC,
	.intr_mode_only = 0,
	.callback = wdt_callback,
	.locks = 0,
	.nb_clk = 0x03FFFFFF, /* 0x3FF to 0x03FFFFFF */
	.wdt_window = 0,
	.wdt_warn = 0x3FF,
};

void system_init()
{
	/* Configure the Watchdog */
	watchdog_config(&wdconf);
	system_set_default_power_state();
	clock_config(SELECTED_FREQ);
	set_pins(common_pins);
	set_pins(adc_pins);
	gpio_on();
	status_led_config(&status_led_green, &status_led_red);
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
	uprintf(UART0, name);
	while (1);
}


/***************************************************************************** */
/* System configuration over USB */
static uint32_t fan_speed = 0;
static int act_cmd = 0;
void config_rx(uint8_t c)
{
	/* FAN control */
	if (c == 'f') {
		fan_speed = 100;
		timer_set_match(LPC_TIMER_32B0, CHAN0, FAN_ON);
	} else if (c == 'z') {
		fan_speed = 0;
		timer_set_match(LPC_TIMER_32B0, CHAN0, 0);
	} else if ((c >= '0') && (c <= '9')) {
		fan_speed = ((c - '0') * 10);
		if (fan_speed < 60) {
			fan_speed = 0;
		}
		timer_set_match(LPC_TIMER_32B0, CHAN0, (FAN_ON - ((FAN_ON / 100) * (100 - fan_speed))));
	}
}



/***************************************************************************** */
/* System communication over UART1 */
void cmd_rx(uint8_t c)
{
}


/***************************************************************************** */

void set_ctrl_duty_cycle(uint8_t value)
{
	act_cmd = value;
	if (act_cmd > 100) {
		/* 100 is the maximum allowed value */
		act_cmd = 100;
	} else if (act_cmd <= 2) {
		/* Below 3% there are triggering problems which lead to 50% instead of 1% or 2% */
		act_cmd = 0;
	}
}

static volatile uint8_t ac_ctrl_state = 0;
void ac_switch_on(uint32_t flags)
{
	/* Generate a 5us (approx) pulse */
	if (ac_ctrl_state == 1) {
		/* Start of pulse */
		gpio_set(ac_ctrl);
		/* Change state machine state */
		ac_ctrl_state = 0;
		/* And request interrupt in approx 5us to clear ac_ctrl output */
		timer_set_match(LPC_TIMER_32B1, CHAN1, (5 * 480));
		timer_restart(LPC_TIMER_32B1);
	} else {
		/* End of pulse */
		gpio_clear(ac_ctrl);
	}
}

static uint32_t clk_cycles_ac_zc = 0;
static volatile uint32_t zc_count = 0; /* Wraps every 1.36 year ... */
void zero_cross(uint32_t gpio)
{
	uint32_t delay = 0;

	zc_count ++;

	if (act_cmd == 100) {
		gpio_set(ac_ctrl);
		return;
	}
	gpio_clear(ac_ctrl);
	if (act_cmd == 0) {
		return;
	}
	/* Set timer to trigger ac out ON at given delay */
	delay = clk_cycles_ac_zc * (100 - act_cmd);
	timer_set_match(LPC_TIMER_32B1, CHAN1, delay);
	timer_restart(LPC_TIMER_32B1);
	/* Set "AC triggering" stame machine state */
	ac_ctrl_state = 1;
}

static uint8_t thermal_warn_flag = 0;
void th_warning(uint32_t gpio)
{
	/* FIXME : test for condition set or removed */
	/* Turn off AC output */
	gpio_clear(ac_ctrl);
	act_cmd = 0;
	thermal_warn_flag = 1;
	/* Turn on Fan at max speed */
	timer_set_match(LPC_TIMER_32B0, CHAN0, 0);
}



/***************************************************************************** */
/* System interface */
enum buttons {
	BUTTON_NONE = 0,
	BUTTON_OK,
	BUTTON_UP,
	BUTTON_DOWN,
};


uint32_t manual_activation_request = 0;
uint8_t button_pressed = 0;
void manual_activation(uint32_t gpio) {
    manual_activation_request = MANUAL_ACTIVATION_DURATION;
	button_pressed = BUTTON_OK;
}
void manual_up(uint32_t gpio) {
    manual_activation_request = MANUAL_ACTIVATION_DURATION;
	button_pressed = BUTTON_UP;
}
void manual_down(uint32_t gpio) {
    manual_activation_request = MANUAL_ACTIVATION_DURATION;
	button_pressed = BUTTON_DOWN;
}
void handle_dec_request(uint32_t curent_tick) {
	if (manual_activation_request > 0) {
		manual_activation_request--;
	}
	if (forced_heater_mode == 1) {
		if (forced_heater_delay > 0) {
			forced_heater_delay--;
		}
		if (forced_heater_time > 0) {
			forced_heater_time--;
		}
	}
}


/***************************************************************************** */
void temp_config(int uart_num)
{
	int ret = 0;
	ret = tmp101_sensor_config(&tmp101_sensor);
	if (ret != 0) {
		uprintf(uart_num, "Temp config error: %d\n", ret);
	}
}


/***************************************************************************** */
/* Oled Display */
#define DISPLAY_ADDR   0x78
static uint8_t gddram[ 4 + GDDRAM_SIZE ];
struct oled_display display = {
	.bus_type = SSD130x_BUS_I2C,
	.address = DISPLAY_ADDR,
	.bus_num = I2C0,
	.video_mode = SSD130x_DISP_NORMAL,
	.contrast = 128,
	.scan_dir = SSD130x_SCAN_BOTTOM_TOP,
	.read_dir = SSD130x_RIGHT_TO_LEFT,
	.display_offset_dir = SSD130x_MOVE_TOP,
	.display_offset = 4,
  .gddram = gddram,
};

#define ROW(x)   VERTICAL_REV(x)
DECLARE_FONT(font);

void display_char(uint8_t line, uint8_t col, uint8_t c)
{
	uint8_t tile = (c > FIRST_FONT_CHAR) ? (c - FIRST_FONT_CHAR) : 0;
	uint8_t* tile_data = (uint8_t*)(&font[tile]);
	ssd130x_buffer_set_tile(gddram, col, line, tile_data);
}
#define OLED_LINE_CHAR_LENGTH     (SSD130x_NB_COL / 8)
#define DISPLAY_LINE_LENGTH  (OLED_LINE_CHAR_LENGTH + 1)
int display_line(uint8_t line, uint8_t col, char* text)
{
	int len = strlen((char*)text);
	int i = 0;

	for (i = 0; i < len; i++) {
		uint8_t tile = (text[i] > FIRST_FONT_CHAR) ? (text[i] - FIRST_FONT_CHAR) : 0;
		uint8_t* tile_data = (uint8_t*)(&font[tile]);
		ssd130x_buffer_set_tile(gddram, col++, line, tile_data);
		if (col >= (OLED_LINE_CHAR_LENGTH)) {
			col = 0;
			line++;
			if (line >= SSD130x_NB_PAGES) {
				return i;
			}
		}
	}
	return len;
}




/***************************************************************************** */
#define NB_VAL 20

enum modes {
	heat = 'C',
	ejp = 'E',
	delayed_heat_prod = 'P',
	forced = 'F',
	temp_OK = 'T',
	manual = 'M',
	idle_heat = 'L',
	full_heat = 'F',
};

/***************************************************************************** */
int main(void)
{
	uint16_t isnail_solar_values[NB_VAL];
	uint16_t isnail_home_values[NB_VAL];
	uint8_t idx = 0;
	uint32_t loop = 0;
	char mode = heat; /* Debug info */
	int ret = 0;

	system_init();
	status_led(red_only);
	uart_on(UART0, 115200, config_rx);
	uart_on(UART1, 115200, cmd_rx);
	i2c_on(I2C0, I2C_CLK_100KHz, I2C_MASTER);
	ssp_master_on(thermo.ssp_bus_num, LPC_SSP_FRAME_SPI, 8, 4*1000*1000);
	adc_on(NULL);
	timer_on(LPC_TIMER_32B0, 0, NULL);
	timer_on(LPC_TIMER_32B1, 0, ac_switch_on);

	/* Immediatly turn off Mosfet / Triac */
	config_gpio(&ac_ctrl, 0, GPIO_DIR_OUT, 0);

	/* Thermocouple configuration */
	max31855_sensor_config(&thermo);
	uprintf(UART0, "Thermocouple config done\n");

	/* TMP101 sensor config */
	temp_config(UART0);

	/* Activate on Rising edge (button release) */
	set_gpio_callback(manual_activation, &button_ok, EDGE_RISING);
#if 0
	/* Debug */
	config_gpio(&button_b1, 0, GPIO_DIR_OUT, 0);
	config_gpio(&button_b2, 0, GPIO_DIR_OUT, 0);
#endif
	set_gpio_callback(manual_up, &button_b1, EDGE_RISING);
	set_gpio_callback(manual_down, &button_b2, EDGE_RISING);

	/* Zero cross and alert pin */
	set_gpio_callback(zero_cross, &zero_cross_in_pin, EDGE_FALLING);
	set_gpio_callback(th_warning, &th_warn_in_pin, EDGES_BOTH);

	/* Start ADC sampling */
	adc_start_burst_conversion(ADC_MCH(0) | ADC_MCH(1) | ADC_MCH(2) | ADC_MCH(7), LPC_ADC_SEQ(0));

	/* Configure Input GPIO */
	config_gpio(&ejp_in_pin, 0, GPIO_DIR_IN, 0);
	config_gpio(&rtc_in_pin, 0, GPIO_DIR_IN, 0);
	config_gpio(&charge_status_in_pin, 0, GPIO_DIR_IN, 1);

	/* WS2812B Leds on display board */
	ws2812_config(&ws2812_data_out_pin);

	/* FAN Config */
	timer_pwm_config(LPC_TIMER_32B0, &fan_pwm_conf);
	timer_start(LPC_TIMER_32B0);
	/* AC Switch Config */
	timer_counter_config(LPC_TIMER_32B1, &ac_timer_conf);
	/* We want 100 Hz (50 Hz but two zero crossings) with 1% granularity */
	clk_cycles_ac_zc = get_main_clock() / (100 * 100);

	status_led(green_only);

    /* Configure and start display */
    ret = ssd130x_display_on(&display);
    /* Erase screen */
    ssd130x_buffer_set(gddram, 0x00);
    ret = ssd130x_display_full_screen(&display);

	/* RTC init */
	ret = rtc_pcf85363a_config(&rtc_conf);
	ret = rtc_pcf85363a_is_up(&rtc_conf, &oldest);
	if (ret == 1) {
		char buff[30];
		rtc_pcf85363_time_read(&rtc_conf, &now);
		rtc_pcf85363_time_to_str(&now, buff, 30);
		/* Debug */
		uprintf(UART0, buff);
	} else if (ret == -EFAULT) {
		memcpy(&now, &oldest, sizeof(struct rtc_time));
		rtc_pcf85363_time_write(&rtc_conf, &now);
	}

	/* microSD card init */
	ret = sdmmc_init(&micro_sd);
	if (ret == 0) {
		msleep(1);
		ret = sdmmc_init_wait_card_ready(&micro_sd);
		if (ret == 0) {
			ret = sdmmc_init_end(&micro_sd);
		}
	}
	uprintf(UART0, "uSD init: %d, type: %d, bs: %d\n", ret, micro_sd.card_type, micro_sd.block_size);
	ret = sdmmc_read_block(&micro_sd, 0, mmc_data);
	uprintf(UART0, "uSD read: %s\n", mmc_data);

	/* Add a systick callback to handle time counting */
	//add_systick_callback(handle_dec_request, DEC_PERIOD);

	msleep(50);
	/* Read parameters from memory */
	if (1) {
		never_force = 0;
		forced_heater_delay = 0;
		forced_heater_time = FORCED_HEATER_DURATION;
	}

	while (1) {
		static uint8_t command_val = 0;
		static uint8_t n_dec = 0;  /* Add some PID like (derivative part) */
		static uint8_t n_inc = 0;  /* Add some PID like (derivative part) */
		int moyenne_solar = 0;
		int moyenne_home = 0;
		uint16_t isnail_val_solar = 0;
		uint16_t isnail_val_home = 0;
		uint16_t acs_val_load = 0;
		uint16_t user_potar = 0;
		int water_centi_degrees = 0;
		int tmp101_deci_degrees = 0;

		mode = heat;
		tmp101_sensor_start_conversion(&tmp101_sensor);
		/* Always track power consumption and production */
		adc_get_value(&isnail_val_solar, LPC_ADC(1));
		adc_get_value(&isnail_val_home, LPC_ADC(0));
		adc_get_value(&acs_val_load, LPC_ADC(2));
		adc_get_value(&user_potar, LPC_ADC(7));
		/* Convert to mA value */
		isnail_val_solar = ((isnail_val_solar * 32) * 2); /* 3.2mV / digit, 50mV -> 1A */
		isnail_val_home = ((isnail_val_home * 32) * 2); /* 3.2mV / digit, 50mV -> 1A */
		/* Store value */
		isnail_solar_values[idx] = isnail_val_solar;
		isnail_home_values[idx++] = isnail_val_home;
		if (idx == NB_VAL) {
			idx = 0;
		}
		/* Compute average value when we sampled enough values */
		/* FIXME : Improve by substracting oldest value before storing new one in table and adding new one */
		if ((idx == 0) || (idx == (NB_VAL / 2))) {
			int i = 0;
			for (i = 0; i < NB_VAL; i++) {
				moyenne_solar += isnail_solar_values[i];
				moyenne_home += isnail_home_values[i];
			}
			moyenne_solar = moyenne_solar / NB_VAL;
			moyenne_home = moyenne_home / NB_VAL;
		} else {
			/* Sleep for a litle more than a period (20ms at 50Hz) */
			msleep(23);
			continue;
		}

		/* Feed the dog */
		if ((moyenne_solar != 0) && (moyenne_home != 0)) {
			watchdog_feed();
		}

		/* Get internal temperature */
		if (1) {
			int ret = 0;
			msleep(40);
			ret = tmp101_sensor_read(&tmp101_sensor, NULL, &tmp101_deci_degrees);
			if (ret != 0) {
				uprintf(UART0, "TMP101 read error : %d\n", ret);
			}
		}
		/* If internal temperature is above 30°C, then turn on fan. Turn of when back to under 28.5°C */
		if (tmp101_deci_degrees > 300) {
			fan_speed = 100;
			timer_set_match(LPC_TIMER_32B0, CHAN0, FAN_ON);
		} else if (tmp101_deci_degrees < 285) {
			fan_speed = 0;
			timer_set_match(LPC_TIMER_32B0, CHAN0, 0);
		}

		/* Get thermocouple value */
		if (1) {
			int ret = 0;
			ret = max31855_sensor_read(&thermo, NULL, &water_centi_degrees);
			if (ret != 0) {
				uprintf(UART0, "Water Temp read error : %d\n", ret);
			}
		}

		/* Need to enter Forced heating mode ? */
		if (water_centi_degrees < (FORCE_HEATER_TEMP * 100)) {
			if (forced_heater_mode == 0) {
				uprintf(UART0, "Entering forced mode\n");
				forced_heater_mode = 1;
			}
			status_led(red_on);
			mode = forced;
		} else if ((water_centi_degrees > (TARGET_FORCED_HEATER_TEMP * 100)) && (forced_heater_mode == 1)) {
			status_led(red_off);
			forced_heater_mode = 0;
			command_val = 0;
			uprintf(UART0, "Forced mode exit\n");
			mode = temp_OK;
		}

		/* Do not force if there is a lot of sun, it may be enough to heat again soon */
		if (moyenne_solar > NO_FORCED_HEATING_ON_SUNNY_DAYS) {
			mode = delayed_heat_prod;
			forced_heater_mode = 0;
			/* Note : Do not set forced_heater_mode to 0 in order to keep decrementing the delay for force
			 * heating in case the house power usage does not fall below the production value. */
		}

		/* Do not force heating if this is an EJP day */
		ejp_in = gpio_read(ejp_in_pin);
		if (ejp_in == DAY_IS_EJP) {
			forced_heater_mode = 0;
			mode = ejp;
		}

		if (never_force == 1) {
			forced_heater_mode = 0;
		}

		/* Did the user request a forced heating ? */
		if (manual_activation_request > 1) {
			forced_heater_mode = 1;
			mode = manual;
			if (manual_activation_request == MANUAL_ACTIVATION_DURATION) {
				uprintf(UART0, "Entering manual forced mode for %d ticks\n", manual_activation_request);
				/* Add a systick callback to handle time counting */
				add_systick_callback(handle_dec_request, DEC_PERIOD);
			}
			if (manual_activation_request < 10) {
				uprintf(UART0, "Leaving manual forced mode\n");
				manual_activation_request = 0;
				remove_systick_callback(handle_dec_request);
			}
		}


		/* Which is the current mode ? */
		if (forced_heater_mode == 1) {
			/* Forced heating mode */
			if ((forced_heater_delay == 0) && (forced_heater_time > 0)) {
				command_val = FORCED_MODE_VALUE;
			}
			if (forced_heater_time == 0) {
				forced_heater_delay = FORCED_HEATER_DELAY;
				forced_heater_time = FORCED_HEATER_DURATION;
			}
		} else if (moyenne_solar < (moyenne_home - 75)) {
			/* Low production mode */
			if (command_val > 15) {
				command_val -= ((3 + n_dec) % 15);
				/* Asservissement */
				n_dec++;
				if (n_dec >= 3) {
					n_inc = 0;
				}
			} else {
				command_val = 0;
				mode = idle_heat;
			}
			status_led(green_off);
		} else if (moyenne_solar > (moyenne_home + 75)) {
			/* High production mode */
			if (command_val < 95) {
				command_val += (2 + n_inc);
				/* Asservissement */
				n_inc++;
				if (n_inc >= 5) {
					n_inc = 5;
					n_dec = 0;
				}
			} else {
				command_val = 100;
				mode = full_heat;
			}
			status_led(green_on);
		}

		/* Set Control Output duty cycle */
		set_ctrl_duty_cycle(command_val);
		/* Debug Nath TMP */
		//set_ctrl_duty_cycle( user_potar / 10 );
		/* Display */
		if (1) {
			char line[DISPLAY_LINE_LENGTH];
			int abs_centi = water_centi_degrees;
			int abs_deci = tmp101_deci_degrees;
			uprintf(UART0, "%c:%d - Is: %d,%04d - Ih: %d,%04d\n", mode, loop++,
						(moyenne_solar / 1000), (moyenne_solar % 1000),
						(moyenne_home / 1000), (moyenne_home % 1000));
			if (water_centi_degrees < 0) {
				abs_centi = -water_centi_degrees;
			}
			uprintf(UART0, "Water Temp : % 4d.%02d\n", (water_centi_degrees / 100), (abs_centi % 100));
			if (tmp101_deci_degrees < 0) {
				abs_deci = -tmp101_deci_degrees;
			}
			uprintf(UART0, "Internal Temp : % 4d.%02d\n", (tmp101_deci_degrees / 10), (abs_deci % 10));
			uprintf(UART0, "ZC_cnt: %d\n", zc_count);
			uprintf(UART0, "ADC: Sol: %dmA, Home: %dmA, Load: %d, User: %d\n",
							isnail_val_solar, isnail_val_home, acs_val_load, user_potar);
			if (button_pressed != 0) {
				uprintf(UART0, "Button : %d\n", button_pressed);
				button_pressed  = 0;
			}
			uprintf(UART0, "CMD: %d/%d, Fan: %d, ndec:%d, ninc:%d\n\n", command_val, act_cmd, fan_speed, n_dec, n_inc);
			ws2812_set_pixel(0, (isnail_val_home / 2000), (isnail_val_solar / 2000), fan_speed);
			ws2812_set_pixel(1, 0, 0, (user_potar >> 2));
			ws2812_send_frame(0);
			/* Erase screen (internal copy) */
			ssd130x_buffer_set(gddram, 0x00);
			/* Update time and time display on internal memory */
			rtc_pcf85363_time_read(&rtc_conf, &now);
			snprintf(line, DISPLAY_LINE_LENGTH, "%02xh%02x:%02x", now.hour, now.min, now.sec); 
			display_line(0, 0, line);
			/* Display info */
			snprintf(line, DISPLAY_LINE_LENGTH, "Water:% 2d.%03d %cC", (water_centi_degrees / 100), (abs_centi % 100), 0x1F);
			display_line(2, 0, line);
			snprintf(line, DISPLAY_LINE_LENGTH, "Prod :% 2d,%03dA", (isnail_val_solar / 1000), ((isnail_val_solar % 1000) / 10));
			display_line(3, 0, line);
			snprintf(line, DISPLAY_LINE_LENGTH, "Conso:% 2d,%03dA", (isnail_val_home / 1000), ((isnail_val_home % 1000) / 10));
			display_line(4, 0, line);
			snprintf(line, DISPLAY_LINE_LENGTH, "Command: %d%%", act_cmd);
			display_line(5, 0, line);
			snprintf(line, DISPLAY_LINE_LENGTH, "Mode: %c", mode);
			display_line(6, 0, line);
			/* Update Oled display */
			ret = ssd130x_display_full_screen(&display);
		}
	}
	return 0;
}



