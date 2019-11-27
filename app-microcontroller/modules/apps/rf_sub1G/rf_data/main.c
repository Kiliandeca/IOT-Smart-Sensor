/****************************************************************************
 *   apps/rf_sub1G/simple/main.c
 *
 * sub1G_module support code - USB version
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
 *************************************************************************** */


#include "core/termios.h"
#include "core/system.h"
#include "core/systick.h"
#include "core/pio.h"
#include "lib/stdio.h"
#include "drivers/serial.h"
#include "drivers/gpio.h"
#include "drivers/ssp.h"
#include "extdrv/cc1101.h"
#include "extdrv/status_led.h"
#include "drivers/i2c.h"

#include "extdrv/bme280_humidity_sensor.h"
#include "extdrv/veml6070_uv_sensor.h"
#include "extdrv/tsl256x_light_sensor.h"

#define MODULE_VERSION	0x03
#define MODULE_NAME "RF Sub1G - USB"

#define RF_868MHz  1
#define RF_915MHz  0
#if ((RF_868MHz) + (RF_915MHz) != 1)
#error Either RF_868MHz or RF_915MHz MUST be defined.
#endif

#define DEBUG 1
#define BUFF_LEN 60
#define RF_BUFF_LEN  64
#define RF_BUFF_LEN2 64

#define SELECTED_FREQ  FREQ_SEL_48MHz
#define DEVICE_ADDRESS  67 /* Addresses 0x00 and 0xFF are broadcast */
#define NEIGHBOR_ADDRESS 66 /* Address of the associated device */
#define SERIAL_BAUD  B1152000
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
	/* SPI */
	{ LPC_SSP0_SCLK_PIO_0_14, LPC_IO_DIGITAL },
	{ LPC_SSP0_MOSI_PIO_0_17, LPC_IO_DIGITAL },
	{ LPC_SSP0_MISO_PIO_0_16, LPC_IO_DIGITAL },
	/* I2C 0 */
	{ LPC_I2C0_SCL_PIO_0_10, (LPC_IO_DIGITAL | LPC_IO_OPEN_DRAIN_ENABLE) },
	{ LPC_I2C0_SDA_PIO_0_11, (LPC_IO_DIGITAL | LPC_IO_OPEN_DRAIN_ENABLE) },
	ARRAY_LAST_PIO,
};

const struct pio cc1101_cs_pin = LPC_GPIO_0_15;
const struct pio cc1101_miso_pin = LPC_SSP0_MISO_PIO_0_16;
const struct pio cc1101_gdo0 = LPC_GPIO_0_6;
const struct pio cc1101_gdo2 = LPC_GPIO_0_7;

const struct pio status_led_green = LPC_GPIO_0_28;
const struct pio status_led_red = LPC_GPIO_0_29;

const struct pio button = LPC_GPIO_0_12; /* ISP button */

// Message
struct message 
{
	uint32_t temp;
	uint16_t hum;
	uint32_t lum;
};
typedef struct message message;


// Variables pour stocker l'odre d'affichage
int posTemp;
int posHum;
int posLum;


/***************************************************************************** */
/* Chiffrement configuration */
void encrypt(uint8_t *data, uint8_t *key);
void decrypt(uint8_t *data, uint8_t *key);

static const uint8_t sbox[256] = {
    //0     1    2      3     4    5     6     7      8    9     A      B    C     D     E     F
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
    0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
    0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
    0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
    0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
    0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
    0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
    0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
    0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
    0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
    0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
    0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
    0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
    0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
    0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16 };

uint8_t key[16] = {0x63, 0x7c, 0x77, 0x7b, 0xf2, 0xa5, 0xd8, 0x31, 0x15, 0x04, 0xc7, 0x23, 0xc3, 0x3b, 0xd6, 0x16 };
/***************************************************************************** */


/***************************************************************************** */
/* Fonction permettant le chiffrement et de déchiffrement  */
void encrypt(uint8_t *data, uint8_t *key){

    int i, j, k;
    uint8_t buff;

    for(k=0;k<9;k++){
        // Sub byte
        for(i=0;i<16;i++){
            data[i] = sbox[data[i]];
        }

        // Rotate byte 
        for(i=0;i<4;i++){
            for(j=0;j<i;j++){
                buff = data[i*4];
                data[i*4] = data[i*4+1];
                data[i*4+1] = data[i*4+2];
                data[i*4+2] = data[i*4+3];
                data[i*4+3] = buff;
            }
        }

        // Mix Columns


        // Add Round Key
        for(i=0;i<16;i++){
            data[i] ^= key[i];
        }
    }

    

}

void decrypt(uint8_t *data, uint8_t *key){
    
    int i, j, k;
    uint8_t buff;

    for(k=0;k<9;k++){
        // Remove Round Key
        for(i=0;i<16;i++){
            data[i] ^= key[i];
        }

        // Unmix Columns

        // Unrotate byte 
        for(i=0;i<4;i++){
            for(j=0;j<i;j++){
                buff = data[i*4+3];
                data[i*4+3] = data[i*4+2];
                data[i*4+2] = data[i*4+1];
                data[i*4+1] = data[i*4];
                data[i*4] = buff;
            }
        }

        // Unsub byte
        for(i=0;i<16;i++){
            j = 0;
            while(sbox[j] != data[i]){
                j++;
            }
            data[i] = j;
        }

    }

}
/***************************************************************************** */

/************************************ HEX TO DIC ***************************************** */
// char * HexToDic(int number){
// 	long int decimalNumber,remainder,quotient;
// 	int i=1,j,temp;
// 	char hexadecimalNumber[100];
// 	printf("Enter any decimal number: ");
// 	scanf("%ld",&decimalNumber);
// 	quotient = decimalNumber;
// 	while(quotient!=0) {
// 		temp = quotient % 16;
// 		//To convert integer into character
// 		if( temp < 10)
// 					temp =temp + 48; else
// 					temp = temp + 55;
// 		hexadecimalNumber[i++]= temp;
// 		quotient = quotient / 16;
// 	}
// 	printf("Equivalent hexadecimal value of decimal number %d: ",decimalNumber);
// 	for (j = i -1 ;j> 0;j--)
// 			printf("%c",hexadecimalNumber[j]);
// 	return 0;
// }

/***************************************************************************** */



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
	uprintf(UART0, name);
	while (1);
}

static volatile int check_rx = 0;
void rf_rx_calback(uint32_t gpio)
{
	check_rx = 1;
}

static uint8_t rf_specific_settings[] = {
	CC1101_REGS(gdo_config[2]), 0x07, /* GDO_0 - Assert on CRC OK | Disable temp sensor */
	CC1101_REGS(gdo_config[0]), 0x2E, /* GDO_2 - FIXME : do something usefull with it for tests */
	CC1101_REGS(pkt_ctrl[0]), 0x0F, /* Accept all sync, CRC err auto flush, Append, Addr check and Bcast */
#if (RF_915MHz == 1)
	/* FIXME : Add here a define protected list of settings for 915MHz configuration */
#endif
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
    cc1101_set_address(DEVICE_ADDRESS);
#ifdef DEBUG
	// uprintf(UART0, "CC1101 RF link init done.\n\r");
#endif
}


uint8_t chenillard_active = 1;
void handle_rf_rx_data(void)
{
	uint8_t data[RF_BUFF_LEN];
	int8_t ret = 0;
	uint8_t status = 0;

	/* Check for received packet (and get it if any) */
	ret = cc1101_receive_packet(data, RF_BUFF_LEN, &status);
	/* Go back to RX mode */
	cc1101_enter_rx_mode();
	message msg_data;
	

	int crcReceived = data[0] + data[1] + data[2];

	if(data[1] == DEVICE_ADDRESS && data[2] == NEIGHBOR_ADDRESS && data[3] == crcReceived ){
		// memcpy(&msg_data,&data[12],sizeof(message));
		int8_t test[8] = {data[4],data[5],data[6],data[7],data[8],data[9],data[10],data[11]};
		// uprintf(UART0, "Msg recu chiffrer  ## Lum : %d %d %d -- T: %d,%d %d degC -- Humidity : %d,%d  \n\r",test[0], test[1], test[2],test[3],test[4], test[5], test[6], test[7]);
		// uprintf(UART0, "\t formate in8t ---- Lum : %d %d %d -- T: %d,%d %d degC -- Humidity : %d,%d  \n\r",test[0], test[1], test[2],test[3],test[4], test[5], test[6], test[7]);
		decrypt(test,key);

		// uprintf(UART0, "Msg recu decrypter  ## Lum : %d %d %d -- T: %d,%d %d degC -- Humidity : %d,%d  \n\r",test[0], test[1], test[2],test[3],test[4], test[5], test[6], test[7]);

		uprintf(UART0, "{ \"Lux\": \"%d%d%d\", \"Temp\": \"%d%d.%d\", \"Humidity\": \"%d.%d\"}\n\r",test[0], test[1], test[2],test[3],test[4], test[5], test[6], test[7]);



		// uprintf(UART0, "\t \t---- Lum : %d %d %d -- T: %d,%d %d degC -- Humidity : %d,%d  \n\r",test[0], test[1], test[2],test[3],test[4], test[5], test[6], test[7]);
				// uprintf(UART0, "######## MSG RECEIVED ######## \n\r");
				// uprintf(UART0, "\t Tx ret: %d\n\r", ret);
				// uprintf(UART0, "\t RF: data lenght: %d.\n\r", data[0]);
				// uprintf(UART0, "\t RF: destination : %d.\n\r", data[1]);
				// uprintf(UART0, "\t RF: source : %d.\n\r", data[2]);
				// uprintf(UART0, "\t RF: CRC RECU : %d.\n\r", data[3]);
				// uprintf(UART0, "{ \"Lux\": %d, \"Temp\": %d.%02d, \"Humidity\": %d.%d}\n\r",  
				// 		msg_data.lum,
				// 		msg_data.temp / 10,  (msg_data.temp > 0) ? (msg_data.temp % 10) : ((-msg_data.temp) % 10),
				// 		msg_data.hum / 10, msg_data.hum % 10);
				// uprintf(UART0, "######## END RECEIVED MSG ######## \n\r");
	}

#ifdef DEBUG
	
	
	/* JSON PRINT*/
		// uprintf(UART0, "{ \"Lux\": %d, \"Temp\": %d.%02d, \"Humidity\": %d.%d}\n\r",  
		// 				msg_data.lum,
		// 				msg_data.temp / 10,  (msg_data.temp > 0) ? (msg_data.temp % 10) : ((-msg_data.temp) % 10),
		// 				msg_data.hum / 10, msg_data.hum % 10);
    /*uprintf(UART0, "RF: message: %c.\n\r", data[2]);*/
#endif

}

static volatile uint32_t cc_tx = 0;
static volatile uint8_t cc_tx_buff[RF_BUFF_LEN];
static volatile uint8_t cc_ptr = 0;
uint8_t chenillard_activation_request = 1;
void activate_chenillard(uint32_t gpio) {
	if (chenillard_activation_request == 1){
        cc_tx_buff[0]='0';
        cc_ptr = 1;
        cc_tx=1;
        chenillard_activation_request = 0;
    }
    else{
        cc_tx_buff[0]='1';
        cc_ptr = 1;
        cc_tx=1;
        chenillard_activation_request = 1;
    }
}


/***************************************************************************** */
/* Luminosity */

/* Note : These are 8bits address */
#define TSL256x_ADDR   0x52 /* Pin Addr Sel (pin2 of tsl256x) connected to GND */
struct tsl256x_sensor_config tsl256x_sensor = {
	.bus_num = I2C0,
	.addr = TSL256x_ADDR,
	.gain = TSL256x_LOW_GAIN,
	.integration_time = TSL256x_INTEGRATION_100ms,
	.package = TSL256x_PACKAGE_T,
};

void lux_config(int uart_num)
{
	int ret = 0;
	ret = tsl256x_configure(&tsl256x_sensor);
	if (ret != 0) {
		// uprintf(uart_num, "Lux config error: %d\n\r", ret);
	}
}

void lux_display(int uart_num, uint16_t* ir, uint32_t* lux)
{
	uint16_t comb = 0;
	int ret = 0;

	ret = tsl256x_sensor_read(&tsl256x_sensor, &comb, ir, lux);
	if (ret != 0) {
		// uprintf(uart_num, "Lux read error: %d\n\r", ret);
	} else {
		// uprintf(uart_num, "Lux: %d  (Comb: 0x%04x, IR: 0x%04x)\n\r", *lux, comb, *ir);
	}
}

/***************************************************************************** */
/* BME280 Sensor */

/* Note : 8bits address */
#define BME280_ADDR   0xEC
struct bme280_sensor_config bme280_sensor = {
	.bus_num = I2C0,
	.addr = BME280_ADDR,
	.humidity_oversampling = BME280_OS_x16,
	.temp_oversampling = BME280_OS_x16,
	.pressure_oversampling = BME280_OS_x16,
	.mode = BME280_NORMAL,
	.standby_len = BME280_SB_62ms,
	.filter_coeff = BME280_FILT_OFF,
};

void bme_config(int uart_num)
{
	int ret = 0;

	ret = bme280_configure(&bme280_sensor);
	if (ret != 0) {
		// uprintf(uart_num, "Sensor config error: %d\n\r", ret);
	}
}

/* BME will obtain temperature, pressure and humidity values */

void bme_display(int uart_num, uint32_t* pressure, uint32_t* temp, uint16_t* humidity)
{
	int ret = 0;

	ret = bme280_sensor_read(&bme280_sensor, pressure, temp, humidity);
	if (ret != 0) {
		// uprintf(uart_num, "Sensor read error: %d\n\r", ret);
	} else {
		int comp_temp = 0;
		uint32_t comp_pressure = 0;
		uint32_t comp_humidity = 0;

		comp_temp = bme280_compensate_temperature(&bme280_sensor, *temp) / 10;
		comp_pressure = bme280_compensate_pressure(&bme280_sensor, *pressure) / 100;
		comp_humidity = bme280_compensate_humidity(&bme280_sensor, *humidity) / 10;
		// uprintf(uart_num, "P: %d hPa, T: %d,%02d degC, H: %d,%d rH\n\r",
		// 		comp_pressure,
		// 		comp_temp / 10,  (comp_temp > 0) ? (comp_temp % 10) : ((-comp_temp) % 10),
		// 		comp_humidity / 10, comp_humidity % 10);
		*temp = comp_temp;
		*pressure = comp_pressure;
		*humidity = comp_humidity;
	}
}

/***************************************************************************** */
/* UV */

/* The I2C UV light sensor is at addresses 0x70, 0x71 and 0x73 */
/* Note : These are 8bits address */
#define VEML6070_ADDR   0x70
struct veml6070_sensor_config veml6070_sensor = {
	.bus_num = I2C0,
	.addr = VEML6070_ADDR,
};

void uv_config(int uart_num)
{
	int ret = 0;

	/* UV sensor */
	ret = veml6070_configure(&veml6070_sensor);
	if (ret != 0) {
		// uprintf(uart_num, "UV config error: %d\n\r", ret);
	}
	
}

void uv_display(int uart_num, uint16_t* uv_raw)
{
	int ret = 0;

	ret = veml6070_sensor_read(&veml6070_sensor, uv_raw);
	if (ret != 0) {
		// uprintf(uart_num, "UV read error: %d\n\r", ret);
	} else {
		// uprintf(uart_num, "UV: 0x%04x\n\r", *uv_raw);
	}
}

static volatile uint32_t update_display = 0;

/***************************************************************************** */
void periodic_display(uint32_t tick)
{
	update_display = 1;
}

static volatile message cc_tx_msg;
void send_on_rf(void)
{
	message data;

	

	int8_t test[3] = {posTemp,posHum,posLum};
	encrypt(test,key);


	data.temp = test[0];
	data.hum = test[1];
	data.lum = test[2];

	

				// data.temp = posTemp;
				// data.hum = posHum;
				// data.lum = posLum;
	

	// uint32_t v[] = {posTemp,0xA000000A};
			
	// uprintf(UART0," Original Values: \n");
	// uprintf(UART0,"[ %X ]\n", v[0]);

	// encrypt(v);

	// uprintf(UART0,"Encrypted:       \n");
	// uprintf(UART0,"[ %X ]\n", v[0]);
	// data.temp = v[0];

	// decrypt(v);
	// uprintf(UART0,"\n Decrypted: ");
	// uprintf(UART0,"[ %X ]", v[0]);
	// uprintf(UART0,"\n");


	uint8_t cc_tx_data[sizeof(message)+4];
	cc_tx_data[0]=sizeof(message)+3;
	cc_tx_data[1]=NEIGHBOR_ADDRESS;
	cc_tx_data[2]=DEVICE_ADDRESS;
	cc_tx_data[3]=cc_tx_data[0]+cc_tx_data[1]+cc_tx_data[2];
	memcpy(&cc_tx_data[4], &data, sizeof(message));


	// uprintf(UART0,"av encryp : taille : %d neighbor : %d \n\r",cc_tx_data[0],cc_tx_data[1]);
    
	// encrypt(cc_tx_data,key);
	
	// uprintf(UART0,"ap encryp : taille : %d neighbor : %d \n\r",cc_tx_data[0],cc_tx_data[1]);
	
	// decrypt(cc_tx_data,key);
	
	// uprintf(UART0,"ap decrypt : taille : %d neighbor : %d \n\r",cc_tx_data[0],cc_tx_data[1]);

	/* Send */
	if (cc1101_tx_fifo_state() != 0) {
		cc1101_flush_tx_fifo();
	}

	int ret = cc1101_send_packet(cc_tx_data, sizeof(message)+4);

	// uprintf(UART0, "{ \"Lux\": %d, \"Temp\": %d.%02d, \"Humidity\": %d.%d}\n\r",  
	// 				data.lum,
	// 				data.temp / 10,  (data.temp > 0) ? (data.temp % 10) : ((-data.temp) % 10),
	// 				data.hum / 10, data.hum % 10);
	
// #ifdef DEBUG
// 	uprintf(UART0, "######## MSG SENT ######## \n\r");
// 	uprintf(UART0, "\t Tx ret: %d\n\r", ret);
//     uprintf(UART0, "\t RF: data lenght: %d.\n\r", cc_tx_data[0]);
//     uprintf(UART0, "\t RF: destination: %d.\n\r", cc_tx_data[1]);
//     uprintf(UART0, "\t RF: source: %d.\n\r", cc_tx_data[2]);
// 	uprintf(UART0, "\tRF: CRC %d.\n\r", cc_tx_data[3]);
// 	uprintf(UART0, "######## END SENT MSG ######## \n\r");
// #endif
}




/* Data sent on radio comes from the UART, put any data received from UART in
 * cc_tx_buff and send when either '\r' or '\n' is received.
 * This function is very simple and data received between cc_tx flag set and
 * cc_ptr rewind to 0 may be lost. */
void handle_uart_cmd(uint8_t c)
{
	if (cc_ptr < RF_BUFF_LEN) {
		cc_tx_buff[cc_ptr++] = c;
	} else {
		cc_ptr = 0;
	}
	if ((c == '\n') || (c == '\r')) {
		cc_tx = 1;
	}
}


int main(void)
{
	// uprintf(UART0, "App started\n\r");
	system_init();
	uart_on(UART0, 115200, handle_uart_cmd);
	i2c_on(I2C0, I2C_CLK_100KHz, I2C_MASTER);
	ssp_master_on(0, LPC_SSP_FRAME_SPI, 8, 4*1000*1000); /* bus_num, frame_type, data_width, rate */
	status_led_config(&status_led_green, &status_led_red);


	/* Sensors config */
		// uprintf(UART0, "Config Sensors\n\r");
		// uprintf(UART0, "Config UV\n\r");
	uv_config(UART0);
		// uprintf(UART0, "Config LUX\n\r");
	lux_config(UART0);
		// uprintf(UART0, "Config BME\n\r");
	bme_config(UART0);

	/* Radio */
	rf_config();


	add_systick_callback(periodic_display, 1000);

    /* Activate the chenillard on Rising edge (button release) */
	set_gpio_callback(activate_chenillard, &button, EDGE_RISING);

	// ;



															// uint32_t v[] = {0x0000007B};
	
		// uint32_t v[] = {0xLJBLKECB, 0x0CELCIMF};
		// 64 bit ASCII->HEX translation of LJBLKECB: 4C4A424C4B454342  
		// 64 bit ASCII->HEX trnaslation of 0CELCIMF: 3043454C43494D46
		
															// uprintf(UART0," Original Values: ");
															// uprintf(UART0,"[ %X ]", v[0]);
															
															// encrypt(v);

															// uprintf(UART0,"\n Encrypted:       ");
															// uprintf(UART0,"[ %X ]", v[0]);

															// decrypt(v);
															// uprintf(UART0,"\n Decrypted:       ");
															// uprintf(UART0,"[ %X ]", v[0]);
															// uprintf(UART0,"\n");





	while (1) {
		uint8_t status = 0;

		
        /* Verify that chenillard is enable */
        if (chenillard_active == 1) {
			/* Tell we are alive :) */
		    chenillard(250);
        }
        else{
            status_led(none);
			msleep(250);
        }

		if (cc_ptr == 3){
			// uprintf(UART0,"TEST0 ----> %c",cc_tx_buff[0]);
			// uprintf(UART0,"TEST1 ----> %c",cc_tx_buff[1]);
			// uprintf(UART0,"TEST2 ----> %c",cc_tx_buff[2]);
			handle_uart_cmd('\n');
			cc_ptr = 0;
		}
		cc_tx_buff[0] = 'T';
		cc_tx_buff[1] = 'H';
		cc_tx_buff[2] = 'L';


		if((cc_tx_buff[0] == 'T') && (cc_tx_buff[1] == 'H')){
			posTemp = 1;
			posHum = 2;
			posLum = 3;
			// posTemp = 0x00000001;
			// posHum = 0x00000002;
			// posLum = 0x00000003;
			// uprintf(UART0,"TEST0 ----> %x",posTemp);

		}
		else if((cc_tx_buff[0] == 'T') && (cc_tx_buff[1] == 'L')){
			posTemp = 1;
			posLum = 2;
			posHum = 3;
		}else if((cc_tx_buff[0] == 'L') && (cc_tx_buff[1] == 'T')){
			posLum = 1;
			posTemp = 2;
			posHum = 3;
		}
		else if((cc_tx_buff[0] == 'L') && (cc_tx_buff[1] == 'H')){
			posLum = 1;
			posHum = 2;
			posTemp = 3;

		}
		else if((cc_tx_buff[0] == 'H') && (cc_tx_buff[1] == 'L')){
			posHum = 1;
			posLum = 2;
			posTemp = 3;

		}
		else if((cc_tx_buff[0] == 'H') && (cc_tx_buff[1] == 'T')){
			posHum = 1;
			posLum = 3;
			posTemp = 2;
		}
		
		if (update_display == 1) {
			uint16_t uv = 0, ir = 0, humidity = 0;
			uint32_t pressure = 0, temp = 0, lux = 0;
			
			/* Read the sensors */
			uv_display(UART0, &uv);
			bme_display(UART0, &pressure, &temp, &humidity);
			lux_display(UART0, &ir, &lux);
			cc_tx_msg.temp=temp;
			cc_tx_msg.hum=humidity;
			cc_tx_msg.lum=lux;
			update_display = 0;

			send_on_rf();
		}


		/* RF */
		if (cc_tx == 1) {
			cc_tx = 0;
		}
		/* Do not leave radio in an unknown or unwated state */
		do {
			status = (cc1101_read_status() & CC1101_STATE_MASK);
		} while (status == CC1101_STATE_TX);

		if (status != CC1101_STATE_RX) {
			static uint8_t loop = 0;
			loop++;
			if (loop > 10) {
				if (cc1101_rx_fifo_state() != 0) {
					cc1101_flush_rx_fifo();
				}
				cc1101_enter_rx_mode();
				loop = 0;
			}
		}
		if (check_rx == 1) {
			check_rx = 0;
			handle_rf_rx_data();
		}

			
	}
	
	return 0;
}




