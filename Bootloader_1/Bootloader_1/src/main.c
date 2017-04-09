/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# Minimal main function that starts with a call to system_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>

int main (void)
{
	system_init();
	delay_s(3);

	while(1) {
		for (int i=1; i<0x26; i++) {
			system_gclk_chan_disable(i);			struct system_gclk_chan_config cfg;
			cfg.source_generator = GCLK_GENERATOR_7;
			system_gclk_chan_set_config(i, &cfg);
		}

		system_apb_clock_clear_mask(SYSTEM_CLOCK_APB_APBA,~0);
		system_apb_clock_clear_mask(SYSTEM_CLOCK_APB_APBB,~0);
		system_apb_clock_clear_mask(SYSTEM_CLOCK_APB_APBC,~0);
		
		sleepmgr_sleep(SLEEPMGR_STANDBY);
	}
}
