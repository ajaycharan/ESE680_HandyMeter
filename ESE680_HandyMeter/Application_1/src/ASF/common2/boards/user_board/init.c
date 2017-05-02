/**
 * \file
 *
 * \brief User board initialization template
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include <asf.h>
#include <board.h>
#include <conf_board.h>

#if defined(__GNUC__)
void board_init(void) WEAK __attribute__((alias("system_board_init")));
#elif defined(__ICCARM__)
void board_init(void);
#  pragma weak board_init=system_board_init
#endif


#define CFG(x)	port_pin_set_config(x, &config)
#define LO(x)	port_pin_set_output_level(x,0)
#define HI(x)	port_pin_set_output_level(x,1)
void system_board_init(void)
{
	// Initialize all IO's to turn everything off and low-power
	struct port_config config;
	port_get_config_defaults(&config);

	// SD Card pins, all inputs pull-up
	config.direction	= PORT_PIN_DIR_INPUT;
	config.input_pull	= PORT_PIN_PULL_UP;
	config.powersave	= 0;
	CFG(SD_SPI_MOSI_PIN); CFG(SD_SPI_MISO_PIN); CFG(SD_SPI_SCK_PIN); CFG(SD_SPI_CS_PIN);
	
	// Logic inputs, all input pull-none
	config.direction	= PORT_PIN_DIR_INPUT;
	config.input_pull	= PORT_PIN_PULL_NONE;
	config.powersave	= 1;
	CFG(LOGIC1_TC_CAP_PIN); CFG(LOGIC2_TC_CAP_PIN);

	// USB / UART, both inputs pull-up
	config.direction	= PORT_PIN_DIR_INPUT;
	config.input_pull	= PORT_PIN_PULL_UP;
	config.powersave	= 1;
	CFG(USB_DN_UART_TX_PIN); CFG(USB_DP_UART_RX_PIN);

	// External ADC pins, all inputs pull-down
	config.direction	= PORT_PIN_DIR_INPUT;
	config.input_pull	= PORT_PIN_PULL_DOWN;
	config.powersave	= 1;
	CFG(ADC_SPI_CS_PIN); CFG(ADC_SPI_MISO_PIN); CFG(ADC_SPI_MOSI_PIN); CFG(ADC_SPI_SCK_PIN); CFG(ADC_GCLK_MCK_PIN);

	// Buttons, all inputs pull-none
	config.direction	= PORT_PIN_DIR_INPUT;
	config.input_pull	= PORT_PIN_PULL_UP;
	config.powersave	= 0;
	CFG(BUT1_IRQ_IN_PIN); CFG(BUT2_IRQ_IN_PIN);

	// Battery sense, input pull-none
	config.direction	= PORT_PIN_DIR_INPUT;
	config.input_pull	= PORT_PIN_PULL_NONE;
	config.powersave	= 1;
	CFG(BATSENSE_ADC_PIN);

	// LEDs, input pull-none
	config.direction	= PORT_PIN_DIR_INPUT;
	config.input_pull	= PORT_PIN_PULL_NONE;
	config.powersave	= 1;
	CFG(LED_OUT_PIN);
	
	// Range switches, pull-down
	config.direction	= PORT_PIN_DIR_INPUT;
	config.input_pull	= PORT_PIN_PULL_DOWN;
	config.powersave	= 1;
	CFG(SW1A_GP_OUT_PIN); CFG(SW2A_GP_OUT_PIN); CFG(SW3A_GP_OUT_PIN); CFG(SW4A_GP_OUT_PIN);
	CFG(SW1B_GP_OUT_PIN); CFG(SW2B_GP_OUT_PIN); CFG(SW3B_GP_OUT_PIN); CFG(SW4B_GP_OUT_PIN);
	
	// Accessory power enable, drive low
	config.direction	= PORT_PIN_DIR_OUTPUT;
	config.input_pull	= PORT_PIN_PULL_DOWN;
	config.powersave	= 0;
	CFG(ACCYEN_GP_OUT_PIN); LO(ACCYEN_GP_OUT_PIN);

	// ATWINC, input pull-down communication pins
	config.direction	= PORT_PIN_DIR_INPUT;
	config.input_pull	= PORT_PIN_PULL_DOWN;
	config.powersave	= 1;
	CFG(WINC_SPI_CS_PIN); CFG(WINC_SPI_MISO_PIN); CFG(WINC_SPI_MOSI_PIN); CFG(WINC_SPI_SCK_PIN); CFG(WINC_SPI_INT_PIN); CFG(WINC_GCLK_RTC_PIN); CFG(WINC_WAKE_GP_OUT_PIN);
	
	// ATWINC, drive enable pins low
	config.direction	= PORT_PIN_DIR_OUTPUT;
	config.input_pull	= PORT_PIN_PULL_NONE;
	config.powersave	= 0;
	LO(WINC_RESET_GP_OUT_PIN);  LO(WINC_CHIPEN_GP_OUT_PIN);
	CFG(WINC_RESET_GP_OUT_PIN); CFG(WINC_CHIPEN_GP_OUT_PIN);
	
}