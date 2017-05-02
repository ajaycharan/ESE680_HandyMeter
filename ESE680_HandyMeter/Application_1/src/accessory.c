#include "application.h"
#include "asf.h"
#include "accessory.h"
#include "adc.h"
#include "driver/include/m2m_periph.h"

extern void fatal_error(int numBlinks, const char* fmt, ...);
extern void debug_print(const char* fmt, ...);

extern void set_loads_winc(bool loadA, bool loadB);

void set_loads_winc(bool loadA, bool loadB) {
	m2m_periph_gpio_set_dir(M2M_PERIPH_GPIO15, 1);	// A
	m2m_periph_gpio_set_dir(M2M_PERIPH_GPIO16, 1);	// B
	m2m_periph_gpio_set_val(M2M_PERIPH_GPIO15, loadA);
	m2m_periph_gpio_set_val(M2M_PERIPH_GPIO15, loadB);
}

bool print=0;
void accy_powerUp() {

	// Power enable
	struct port_config cfg;
	port_get_config_defaults(&cfg);
	cfg.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_output_level(ACCYEN_GP_OUT_PIN, 1);
	port_pin_set_output_level(ADC_SPI_CS_PIN, 1);
	port_pin_set_config(ACCYEN_GP_OUT_PIN, &cfg);
	port_pin_set_config(ADC_SPI_CS_PIN, &cfg);

	// Range switches
	port_pin_set_config(SW1A_GP_OUT_PIN, &cfg);
	port_pin_set_config(SW2A_GP_OUT_PIN, &cfg);
	port_pin_set_config(SW3A_GP_OUT_PIN, &cfg);
	port_pin_set_config(SW4A_GP_OUT_PIN, &cfg);
	port_pin_set_config(SW1B_GP_OUT_PIN, &cfg);
	port_pin_set_config(SW2B_GP_OUT_PIN, &cfg);
	port_pin_set_config(SW3B_GP_OUT_PIN, &cfg);
	port_pin_set_config(SW4B_GP_OUT_PIN, &cfg);
	port_pin_set_output_level(SW1A_GP_OUT_PIN, 0);
	port_pin_set_output_level(SW2A_GP_OUT_PIN, 1);
	port_pin_set_output_level(SW3A_GP_OUT_PIN, 1);
	port_pin_set_output_level(SW4A_GP_OUT_PIN, 1);
	port_pin_set_output_level(SW1B_GP_OUT_PIN, 0);
	port_pin_set_output_level(SW2B_GP_OUT_PIN, 1);
	port_pin_set_output_level(SW3B_GP_OUT_PIN, 1);
	port_pin_set_output_level(SW4B_GP_OUT_PIN, 1);

	// Logic channels
	cfg.direction = PORT_PIN_DIR_INPUT;
	port_pin_set_config(LOGIC1_TC_CAP_PIN, &cfg);
	port_pin_set_config(LOGIC2_TC_CAP_PIN, &cfg);

	adc_init();

	// Testing
	adc_rangeSet(RANGE_1_224, RANGE_1_224);
	int cnt=0;
	while(1) {
		cnt++;
		int32_t s1=0;
		int32_t s2=0;
		int32_t s3=0;
		if (adc_fifo_getSample(&s1, &s2, &s3))
				if (cnt % 10 == 0)
				debug_print("S1: %d S2: %d\n", s1, s2);
	}
}

void accy_powerDown() {

	// Switch off the pins by running the init routine
	adc_deinit();
	system_board_init();
}