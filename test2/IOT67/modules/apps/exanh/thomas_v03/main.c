/****************************************************************************
 *   apps/exanh/thomas_v03/main.c
 *
 * E-Xanh Gardener main board support
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

/* ISP button release disables the chenillard for 5 seconds */

#include "config.h"

#define MODULE_VERSION   0x03
#define MODULE_NAME "RobotGrow"




/***************************************************************************** */
#define MAX_SENSORS 6
volatile uint8_t poll_sensor = 0;
void request_samples(uint32_t tick)
{
	static uint8_t next_sensor = 1;
	poll_sensor = next_sensor++;
	if (next_sensor > MAX_SENSORS) {
		next_sensor = 1;
	}
}

#define LOOP_SETUP_TIME  3   /* 3 = 30 seconds */
volatile uint8_t loop_setup = LOOP_SETUP_TIME;
volatile uint8_t need_info = 1;
void decrementer(uint32_t tick)
{
	if (loop_setup > 0) {
		loop_setup--;
	}
	if (need_info > 1) {
		need_info--;
	} else {
		need_info = 1;
	}
}

volatile uint8_t pompe_cmd_force = 0;
void pompe_force(uint32_t gpio)
{
	if (gpio_read(button) == 0) {
		pompe_cmd_force = 1;
	} else {
		pompe_cmd_force = 0;
	}
	need_info = 1 * 6; /* 1 minute */
}

/* Executed before main loop, use pompe_cmd_force to detect button press */
#define NB_CYCLES_MAX_DEFAULT  3
uint8_t nb_cycles_max = NB_CYCLES_MAX_DEFAULT;
#define NB_MAX_VALUES  5
#define MAX_VAL_1  3
#define MAX_VAL_2  6
#define MAX_VAL_3  9
#define MAX_VAL_4  12
#define MAX_VAL_5  15
static const uint8_t max_values[NB_MAX_VALUES] = {
			MAX_VAL_1,
			MAX_VAL_2,
			MAX_VAL_3,
			MAX_VAL_4,
			MAX_VAL_5,
		 };  /* Nombre de cycles de 10s */
void system_setup(void)
{
	uint8_t old_max = MAX_VAL_1;
	uint8_t next_max = 0;

	uprintf(UART0, "Entering setup loop\n");
	/* Read old value from flash */
	/* FIXME */
	nb_cycles_max = old_max;

	while (loop_setup > 0) {
		/* Set led according to current max */
		switch (nb_cycles_max) {
			case MAX_VAL_1:
				ws2812_set_pixel(0, 40, 0, 0);
				ws2812_send_frame(2);
				break;
			case MAX_VAL_2:
				ws2812_set_pixel(0, 0, 0, 40);
				ws2812_send_frame(2);
				break;
			case MAX_VAL_3:
				ws2812_set_pixel(0, 0, 40, 0);
				ws2812_send_frame(2);
				break;
			case MAX_VAL_4:
				ws2812_set_pixel(0, 40, 0, 40);
				ws2812_send_frame(2);
				break;
			case MAX_VAL_5:
				ws2812_set_pixel(0, 30, 30, 30);
				ws2812_send_frame(2);
				break;
		}
		/* Move to next max ? */
		if (pompe_cmd_force == 1) {
			static uint32_t last_tick = 0;
			uint32_t tick = systick_get_tick_count();

			if (tick > (last_tick + 500)) { /* Anti-rebond - 500ms */
				last_tick = tick;
				nb_cycles_max = max_values[next_max++];
				if (next_max >= NB_MAX_VALUES) {
					next_max = 0;
				}
			}
			pompe_cmd_force = 0;
		}
	}
	if (nb_cycles_max > 100) {
		nb_cycles_max = NB_CYCLES_MAX_DEFAULT;
	}
	/* If modified, save to flash */
	if (nb_cycles_max != old_max) {
		/* FIXME */
	}
	uprintf(UART0, "Setup loop done.\n");
}


/***************************************************************************** */
#define VBATT_LOW   12000
#define VBATT_WARN  12800

#define HUMIDITY_THRESHOLD_HIGH  560
#define HUMIDITY_THRESHOLD  460
#define HUMIDITY_THRESHOLD_LOW  410
#define HUMIDITY_MIN  360

#define NB_HOURS_MAX  72
#define CYCLES_MAX_REACHED_DELAY  (((6 * 10) * 6) * NB_HOURS_MAX)  /* ((( 10min ) * 6 ) * N ) -> N hours */

enum pump_cycles {
	NO_PUMP_CYCLE = 0,
	PUMP_CYCLE_NEED_START,
	PUMP_CYCLE_ON,
	PUMP_CYCLE_DELAY,
};
/* TODO : add comments */
volatile int cycle_10s = NO_PUMP_CYCLE;
void pump_manage(uint32_t tick)
{
	static int nb_cycles_on = 0;
	if (cycle_10s == PUMP_CYCLE_NEED_START) {
		cycle_10s = PUMP_CYCLE_ON;
		nb_cycles_on = 1; /* 10s * nb_cycles_on = duree arosage */
	} else if (cycle_10s == PUMP_CYCLE_ON) {
		if (nb_cycles_on > 1) {
			nb_cycles_on--;
		} else {
			/* duree entre arrosages */
			cycle_10s = (6 * 10);  /* Start counter for 10 minutes (10s * 6 * 10) */
		}
	} else if (cycle_10s == PUMP_CYCLE_DELAY) {
		cycle_10s = NO_PUMP_CYCLE; /* End of 10 minutes delay, allow cycle to start again */
	} else if (cycle_10s > PUMP_CYCLE_DELAY) {
		cycle_10s--;
	}
}


/***************************************************************************** */

#define ENV_BUFF_SIZE  20
uint8_t env_buff[ENV_BUFF_SIZE];
volatile int idx = 0;
volatile int mode = 0;
volatile int display = 0;
void handle_cmd(uint8_t c)
{
	if (c == '$') {
		mode = 2; /* Configuration */
	} else if ((display == 0) && (c == '*')) {
		display = 1; /* Continuous display on UART0 */
		poll_sensor = 1; /* Poll on sensor 1 */
	} else if ((display == 1) && (c == '*')) {
		display = 0;
	} else if (mode == 2) {
		if (c == 'd') {
			/* Incoming date config */
			mode = 4;
		} else {
			mode = 0;
		}
	} else if (mode == 4) {
		static uint8_t tidx = 0;
		((uint8_t*)(&time_update))[tidx++] = c;
		if (tidx >= sizeof(struct rtc_time)) {
			tidx = 0;
			perform_time_update = 1;
			mode = 0;
		}
	} else {
		/* Echo ? */
		if (0) {
			serial_send_quickbyte(UART0, c);
		}
	}

}


/***************************************************************************** */

int main(void)
{
	uint8_t old_pompe = 0;
	uint8_t pompe_cmd = 0;
	int internal_temp = 0;
	int mv_batt = 0;
	uint16_t chirp_cap = 0;
	uint16_t chirp_temp = 0;
	int cycles_count = 0;
	int wr_delay = 0;

	system_init();
	system_config();

	system_setup();

	/* Erase environnement data buffer */
	memset(env_buff, 0, ENV_BUFF_SIZE);

	while (1) {
		/* Request a temperature conversion to onboard sensor */
		tmp101_sensor_start_conversion(&tmp101_sensor);

		/* Add some delay to the loop */
		chenillard(100);

		/* Read onboard Temperature sensor */
		if (1) {
			int ret = 0;
			ret = tmp101_sensor_read(&tmp101_sensor, NULL, &internal_temp);
			if (ret != 0) {
				uprintf(UART0, "TMP101 read error : %d\n", ret);
			}
		}

		/* Read onboard ADC for battery voltage */
		if (1) {
			uint16_t batt_val = 0;
			adc_start_convertion_once(LPC_ADC(4), LPC_ADC_SEQ(0), 0);
			msleep(8);
			adc_get_value(&batt_val, LPC_ADC(4));
			mv_batt = ((batt_val * (33 * 5)) / 10);  /*  = ((val * 33) / 10) * 5  (pont diviseur) */
		}

		/* Send the poll request to one of the sensors */
		if (poll_sensor != 0) {
			uint8_t req_buf[4];
			req_buf[0] = '#';
			req_buf[1] = poll_sensor;
			poll_sensor = 0;
			req_buf[2] = 'a';
			serial_write(UART0, (char*)req_buf, 3);
		}

		/* Read chirp sensor information */
		if (1) {
			chirp_cap = chirp_sensor_cap_read(&chirp); /* approx 300 (free air) to 800 (in water) */
			chirp_temp = chirp_sensor_temp_read(&chirp); /* Deci-degrees */
		}

		/* Pump activation ? */
		pompe_cmd = 0;
		/* Only activate if battery voltage is high enough */
		if (mv_batt > VBATT_LOW) {
			static int hist_cycle = 0;
			int need_water = 0;
			/* Need to water the plants ? */
			if (chirp_cap > HUMIDITY_MIN) {
				if (chirp_cap < HUMIDITY_THRESHOLD_LOW) {
					/* Below minimum value, always water the plants, and enter hysteresys cycle */
					hist_cycle = 1;
					need_water = 1;
				} else if ((hist_cycle == 1) && (chirp_cap < HUMIDITY_THRESHOLD)) {
					/* Inside hysteresys range, only enter when we were "low" */
					need_water = 1;
				} else {
					/* Top hysteresys limit reached, no watering and mark hysteresys cycle as done. */
					hist_cycle = 0;
				}
			}
			if (need_water == 1) {
				/* Check watering stage */
				switch (cycle_10s) {
					case NO_PUMP_CYCLE:
						/* Request 10s cycle start */
						cycles_count++;
						cycle_10s = PUMP_CYCLE_NEED_START;
						if (cycles_count >= nb_cycles_max) {
							/* Max cycles is reached, wait for some time before starting again */
							cycle_10s = CYCLES_MAX_REACHED_DELAY;
						}
						break;
					case PUMP_CYCLE_ON:
						/* Water the plant for 10 seconds */
						pompe_cmd = 1;
						break;
					case PUMP_CYCLE_DELAY:
					default:
						/* Do not water for 10 minutes */
						/* nothing to do */
						break;
				}
			}
		}
		if (pompe_cmd_force == 1) {
			pompe_cmd = 1;
		}

		/* Start pump ? */
		if (old_pompe != pompe_cmd) {
			/* Add data to the next mmc data block to be written */
			rtc_pcf85363_time_read(&rtc_conf, &now);
			memcpy((mmc_data + mmc_idx), &now, sizeof(struct rtc_time));
			if (pompe_cmd == 1) {
				if (pompe_cmd_force == 1) {
					memcpy((mmc_data + mmc_idx + sizeof(struct rtc_time)), "Pump FORCE", 11);
					uprintf(UART0, "Pump FORCE\n");
				} else {
					memcpy((mmc_data + mmc_idx + sizeof(struct rtc_time)), "Pump ON", 8);
					uprintf(UART0, "Pump On\n");
				}
				/* Turn on pump */
				gpio_set(pompe_on);
			} else {
				memcpy((mmc_data + mmc_idx + sizeof(struct rtc_time)), "Pump OFF", 9);
				uprintf(UART0, "Pump Stopped\n");
				/* Turn pump off */
				gpio_clear(pompe_on);
			}
			/* Soil sensor value */
			memcpy((mmc_data + mmc_idx + sizeof(struct rtc_time) + 16), &chirp_cap, 2);
			mmc_idx += 32;
			old_pompe = pompe_cmd;
		}

		/* Update led colors ? */
		if (need_info >= 1) {
			uint8_t blue = 0;

			/* Set Humidity led */
			if (chirp_cap > 400) {
				blue = ((chirp_cap - 400) / 3);
			}
			if (chirp_cap > HUMIDITY_THRESHOLD) {
				ws2812_set_pixel(0, 0, 50, blue); /* Vert -> Arrosage OK */
			} else if (chirp_cap > HUMIDITY_THRESHOLD_LOW) {
				ws2812_set_pixel(0, 40, 40, blue); /* Jaune -> Presque OK */
			} else {
				ws2812_set_pixel(0, 50, 0, blue); /* Rouge -> Dry (or out of earth) */
			}
			/* Set Battery led */
			if (mv_batt < VBATT_LOW) {
				ws2812_set_pixel(1, 50, 0, 0);
			} else if (mv_batt < VBATT_WARN) {
				ws2812_set_pixel(1, 40, 40, 0);
			} else {
				ws2812_set_pixel(1, 0, 50, 0);
			}
			ws2812_send_frame(2);
		}
		if (need_info == 1) {
			msleep(10);
			ws2812_set_pixel(1, 0, 0, 0);
			ws2812_set_pixel(0, 0, 0, 0);
			ws2812_send_frame(2);
			need_info = 0;
		}

		/* Read onboard ADC for external analog sensor */
		if (0) {
			uint16_t humidity_val = 0;
			int mv_humidity = 0;
			adc_start_convertion_once(LPC_ADC(1), LPC_ADC_SEQ(0), 0);
			msleep(8);
			adc_get_value(&humidity_val, LPC_ADC(1));
			mv_humidity = ((humidity_val * 32) / 10);
			uprintf(UART0, "External sensor : %d\n", mv_humidity);
		}

		/* Display */
		if (display == 1) {
			int abs_deci = internal_temp;
			if (internal_temp < 0) {
				abs_deci = -internal_temp;
			}
			rtc_pcf85363_time_read(&rtc_conf, &now);
			uprintf(UART0, "Date: %02x-%02x-%02x %02xh%02x:%02x\n",
							now.year, now.month, now.day,
							now.hour, now.min, now.sec);
			uprintf(UART0, "OnBoard :\n");
			uprintf(UART0, "- Internal Temp : % 4d.%02d\n", (internal_temp / 10), (abs_deci % 10));
			uprintf(UART0, "- Battery: %d.%03d V\n", (mv_batt/1000), ((mv_batt/10)%100));
			uprintf(UART0, "- Pump: %s\n", (pompe_cmd_force ? "FORCE" : (pompe_cmd ? "ON" : "OFF")));
			uprintf(UART0, "- Cycle : %d\n", cycle_10s);
			uprintf(UART0, "From chirp :\n");
			uprintf(UART0, "- Cap: %d\n", chirp_cap);
			uprintf(UART0, "- Temp: %d\n", chirp_temp);
			poll_sensor = 1; /* Poll on sensor 1 */
			if (idx >= ENV_BUFF_SIZE) {
				uint16_t data[10];
				int i = 0;
				memcpy(data, env_buff, 20);
				/* Switch data to our endianness */
				for (i = 1; i <= 8; i++) {
					data[i] = (uint16_t)ntohs(data[i]);
				}
				uprintf(UART0, "From sensor %d:\n", (env_buff[1] & 0x1F));
				uprintf(UART0, "- Soil: %d\n", data[1]);
				uprintf(UART0, "- Lux: %d, IR: %d, UV: %d\n", data[2], data[3], data[4]);
				uprintf(UART0, "- Patm: %d hPa, Temp: %d,%02d degC, Humidity: %d,%d rH\n\n",
								data[5],
								data[6] / 10,  (data[6]> 0) ? (data[6] % 10) : ((-data[6]) % 10),
								data[7] / 10, data[7] % 10);
			}
		}

		/* uSD storage */
		#define NB_LOOP_DELAY 100
		if ((wr_delay++ >= NB_LOOP_DELAY) && (mmc_idx <= (MMC_BUF_SIZE - MMC_RECORD_SIZE))) {
			#define TMP_BUFF_SIZE  8
			uint8_t buff[TMP_BUFF_SIZE];
			uint16_t * buff16 = (uint16_t*)&buff;
			wr_delay = 0;
			/* Timestamp */
			rtc_pcf85363_time_read(&rtc_conf, &now);
			memcpy((mmc_data + mmc_idx), &now, sizeof(struct rtc_time));
			buff16[0] = (int16_t)internal_temp;
			buff16[1] = (int16_t)mv_batt;
			buff16[2] = chirp_cap;
			buff16[3] = chirp_temp;
			memcpy((mmc_data + mmc_idx + sizeof(struct rtc_time)), buff, TMP_BUFF_SIZE);
			if (idx >= ENV_BUFF_SIZE) {
				memcpy((mmc_data + mmc_idx + sizeof(struct rtc_time) + TMP_BUFF_SIZE), env_buff, 16);
			}
			mmc_idx += 32;
		}


		/* If we received data from a sensor, then add to SD buffer for storage and display */
		if ((idx >= ENV_BUFF_SIZE) && (mmc_idx <= (MMC_BUF_SIZE - MMC_RECORD_SIZE))) {
			uint16_t* data = (uint16_t*)env_buff;
			int i = 0;
			/* Add data to the next mmc data block to be written */
			rtc_pcf85363_time_read(&rtc_conf, &now);
			memcpy((mmc_data + mmc_idx), &now, sizeof(struct rtc_time));
			memcpy((mmc_data + mmc_idx + sizeof(struct rtc_time)), env_buff, ENV_BUFF_SIZE);
			mmc_idx += 32;
			/* Switch data to our endianness */
			for (i = 1; i <= 8; i++) {
				data[i] = (uint16_t)ntohs(data[i]);
			}
			idx = 0;
			/* Data got received, clear sensor led. */
			env_buff[1] &= 0x1F;
			env_buff[2] = 'l';
			env_buff[3] = 0;
			env_buff[4] = 0;
			env_buff[5] = 0;
			serial_write(UART0, (char*)env_buff, 6);
		}

		/* If MMC buffer is full, then write it to the uSD card */
		if (mmc_idx > (MMC_BUF_SIZE - MMC_RECORD_SIZE)) {
			int ret = 0;
			/* Set the block write time */
			rtc_pcf85363_time_read(&rtc_conf, &now);
			memcpy((mmc_data + 48), &now, sizeof(struct rtc_time));
			/* Save data to uSD card */
			ret = sdmmc_write_block(&micro_sd, mmc_block_num, mmc_data);
			if (ret == 0) {
				uprintf(UART0, "Wrote data to block %d\n", mmc_block_num);
			} else {
				uprintf(UART0, "Error while writting data to block %d: %d\n", mmc_block_num, ret);
			}
			memset(mmc_data, 0, MMC_BUF_SIZE);
			/* Maybe update the "last block written index" */
			/* FIXME : Set to 0x3F */
			if ((mmc_block_num & 0x0F) == 0x00) {
				/* Prepare the block number 2 */
				memcpy(mmc_data, MMC_BLK_VALID_IDX_MAGIC, MMC_BLK_VALID_IDX_MAGIC_SIZE);
				rtc_pcf85363_time_read(&rtc_conf, &now);
				memcpy((mmc_data + MMC_LAST_DATA_IDX_DATE_OFFSET), &now, sizeof(struct rtc_time));
				((uint32_t*)mmc_data)[MMC_LAST_DATA_IDX_OFFSET] = mmc_block_num;
				/* Write to block number 2 (MMC_LAST_DATA_IDX_BLOCK) */
				ret = sdmmc_write_block(&micro_sd, MMC_LAST_DATA_IDX_BLOCK, mmc_data);
				if (ret == 0) {
					uprintf(UART0, "Wrote last bock number to block %d\n", MMC_LAST_DATA_IDX_BLOCK);
				} else {
					uprintf(UART0, "Error while writting last bock number: %d\n", ret);
				}
				memset(mmc_data, 0, MMC_BUF_SIZE);
			}
			/* Prepare buffer for next set of data */
			if (1) {
				int offset = 0;
				/* Start with magic */
				memcpy(mmc_data, MMC_BLK_VALID_MAGIC, sizeof(MMC_BLK_VALID_MAGIC));
				offset = 32;
				/* Add date */
				rtc_pcf85363_time_read(&rtc_conf, &now);
				memcpy((mmc_data + offset), &now, sizeof(struct rtc_time));
				offset += sizeof(struct rtc_time);
				/* Then add internal temp */
				memcpy((mmc_data + offset), &internal_temp, sizeof(internal_temp));
				offset += sizeof(internal_temp);
				/* Add battery voltage */
				memcpy((mmc_data + offset), &mv_batt, sizeof(mv_batt));
				offset += sizeof(mv_batt);
				mmc_idx = 64;
			}
			/* Display internal temp and battery level */
			if (1) {
				int abs_deci = internal_temp;
				if (internal_temp < 0) {
					abs_deci = -internal_temp;
				}
				uprintf(UART0, "Internal Temp : % 4d.%02d\n", (internal_temp / 10), (abs_deci % 10));
				uprintf(UART0, "Batterie: %d.%03d V\n", (mv_batt/1000), ((mv_batt/10)%100));
			}
			mmc_block_num++;
		}

		if (perform_time_update == 1) {
			char buff[30];
			perform_time_update = 0;
			rtc_pcf85363_time_write(&rtc_conf, &time_update);
			rtc_pcf85363_time_to_str(&time_update, buff, 30);
			uprintf(UART0, "Using updated time as now : %s\n", buff);
			time_valid = 1;
		}
	}
	return 0;
}



