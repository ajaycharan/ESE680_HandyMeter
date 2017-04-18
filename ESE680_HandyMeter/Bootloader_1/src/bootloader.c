#include <asf.h>

#include "bootloader.h"

static void program_memory(uint32_t address, uint8_t *buffer, uint32_t len);
static void start_application(void);
static void fatal_error(int numBlinks, const char* fmt, ...);
static void debug_print(const char* fmt, ...);
static void led_blink(int num_blinks, int blink_period, bool green);
static enum status_code configure_wakeup(void);
static enum status_code configure_bod(void);
static enum status_code configure_rtc(void);
static enum status_code configure_nvm(void);
static uint8_t buff[MAX_BUF_SIZE] = {0};
struct rtc_module rtc_instance;

static enum status_code configure_wakeup(void) {
	struct extint_chan_conf ext_conf;
	extint_chan_get_config_defaults(&ext_conf);
	ext_conf.detection_criteria = EXTINT_DETECT_FALLING;
	ext_conf.filter_input_signal = true;
	ext_conf.wake_if_sleeping = true;
	ext_conf.gpio_pin = BUT1_IRQ_IN_PIN;
	ext_conf.gpio_pin_mux = MUX_PA03A_EIC_EXTINT3;
	ext_conf.gpio_pin_pull = EXTINT_PULL_UP;
	extint_chan_set_config(3, &ext_conf);
	enum status_code code1 = extint_chan_enable_callback(3, EXTINT_CALLBACK_TYPE_DETECT);
	ext_conf.gpio_pin = BUT2_IRQ_IN_PIN;
	ext_conf.gpio_pin_mux = MUX_PA31A_EIC_EXTINT11;
	extint_chan_set_config(11, &ext_conf);
	enum status_code code2 = extint_chan_enable_callback(11, EXTINT_CALLBACK_TYPE_DETECT);
	if (code1 != STATUS_OK) return code1;
	else if (code2 != STATUS_OK) return code2;
	else return STATUS_OK;
}
static enum status_code configure_rtc(void)
{
	// config base structs
	rtc_instance.hw = RTC;
	rtc_instance.clock_24h = 1;
	rtc_instance.year_init_value = 2000;
	struct rtc_calendar_config config_rtc_calendar;
	rtc_calendar_get_config_defaults(&config_rtc_calendar);
	config_rtc_calendar.clock_24h = true;
	
	// enable the GCLK to RTC to read out the initial time before enabling
	struct system_gclk_chan_config gclk_chan_conf;
	system_gclk_chan_get_config_defaults(&gclk_chan_conf);
	gclk_chan_conf.source_generator = GCLK_GENERATOR_2;
	system_gclk_chan_set_config(RTC_GCLK_ID, &gclk_chan_conf);
	system_gclk_chan_enable(RTC_GCLK_ID);

	// enable the RTC and set the initial time
	rtc_calendar_init(&rtc_instance, RTC, &config_rtc_calendar);
	rtc_calendar_enable(&rtc_instance);
	return STATUS_OK;
}
static enum status_code configure_nvm(void) {
	struct nvm_config config;
	nvm_get_config_defaults(&config);
	config.manual_page_write = false;
	enum status_code code = nvm_set_config(&config);
	return code;
}
static enum status_code configure_bod(void)
{
	struct bod_config config_bod33;
	bod_get_config_defaults(&config_bod33);
	config_bod33.action = BOD_ACTION_INTERRUPT;
	config_bod33.hysteresis = 1;
	config_bod33.run_in_standby = 0;
	config_bod33.mode = BOD_MODE_SAMPLED;
	config_bod33.level = 40;						// Set to 2.8V
	enum status_code code1 = bod_set_config(BOD_BOD33, &config_bod33);
	enum status_code code2 = bod_enable(BOD_BOD33);
	SYSCTRL->INTENSET.reg = SYSCTRL_INTENCLR_BOD33DET;
	system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_SYSCTRL);
	if (code1 != STATUS_OK) return code1;
	else if (code2 != STATUS_OK) return code2;
	else return STATUS_OK;
}

// Brown-out action, when supply-voltage < 2.8V ...
void SYSCTRL_Handler(void) {
	if (SYSCTRL->INTFLAG.reg & SYSCTRL_INTFLAG_BOD33DET) {
		SYSCTRL->INTFLAG.reg = SYSCTRL_INTFLAG_BOD33DET;
		system_sleep();
	}
}

int main(void)
{
	delay_init();
	led_blink(1,100,1);				// Blink once to indicate entry

	bool gotoApplication = 1;		// Can we skip to the application?
	bool sdError = 0;				// Any error in SD card process?

	// Check buttons, if both pressed, force bootloader execution
	struct port_config but_cfg;
	port_get_config_defaults(&but_cfg);
	port_pin_set_config(BUT1_IRQ_IN_PIN, &but_cfg);
	port_pin_set_config(BUT2_IRQ_IN_PIN, &but_cfg);
	bool b1 = !port_pin_get_input_level(BUT1_IRQ_IN_PIN);
	bool b2 = !port_pin_get_input_level(BUT2_IRQ_IN_PIN);
	if (b1 && b2) {
		gotoApplication = 0;
	}
	
	// Check SD card for new boot file, force bootloader execution if found
	// Initialize SD card driver
	sd_mmc_init();

	// Attempt to initialize SD card
	Ctrl_status status = CTRL_FAIL;
	while (status != CTRL_GOOD) {
		status = sd_mmc_test_unit_ready(0);
		if (status == CTRL_FAIL || status == CTRL_NO_PRESENT) {
			sdError = 1;
		}
	}

	// Check for new firmware image on SD
	FATFS fs;
	FIL file_object;
	const char* image_name = IMAGE_DEFAULT_NAME;
	FRESULT res;
	FILINFO fno;
	memset(&fs, 0, sizeof(FATFS));	
	if (sdError == 0) {
		// Mount FAT32 file system on SD card
		res = f_mount(LUN_ID_SD_MMC_0_MEM, &fs);
	
		// Find the image file
		res = f_stat(image_name, &fno);
		if (res == FR_OK) {				// File exists
			gotoApplication = 0;
			sdError = 0;
		} else sdError = 1;
	}

	// Disable SD card SPI
	struct spi_module spi;
	spi.hw = SD_MMC_SPI;
	spi_reset(&spi);
	spi_disable(&spi);

	// Disable interrupts
	system_interrupt_disable_global();

	// Jump to application if allowed
	if (gotoApplication) {
		start_application();
	}

	// START BOOTLOADER MAIN PROGRAM
	system_init();						// clocks and I/O pins
	delay_init();						// delay clock update
	system_interrupt_enable_global();	// Enable interrupts
	system_set_sleepmode(SYSTEM_SLEEPMODE_STANDBY); // Allow sleep mode
#ifdef ENABLE_USB_DEBUG
	stdio_usb_init();					// stdio over USB CDC
	stdio_usb_enable();					// stdio over USB CDC
	delay_ms(500);						// Wait for USB to be ready
#endif
	// Print diagnostic information
	debug_print("\n\n\n");
	debug_print("..... Bootloader started .....\n");
	debug_print("Compiled on: %s %s \n\n\n", __DATE__, __TIME__);

	// Peripheral init
	enum status_code code;
	code = configure_rtc();				// RTC module
	if (code != STATUS_OK) fatal_error(1,"RTC module init failed.\n");
	code = configure_bod();				// BOD detector
	if (code != STATUS_OK) fatal_error(2,"BOD module init failed.\n");
	code = configure_wakeup();			// Wake-up from sleep using buttons
	if (code != STATUS_OK) fatal_error(3,"EXTINT module init failed.\n");
	code = configure_nvm();				// NVM controller
	if (code != STATUS_OK) fatal_error(4,"NVM module init failed.\n");

	// RTC time check
	struct rtc_calendar_time time;
	rtc_calendar_get_time(&rtc_instance, &time);
	debug_print("Local Time: %d/%d/%d  %d:%d:%d\n", time.month, time.day, time.year, time.hour, time.minute, time.second);
	debug_print("New image file: %s\n", sdError?"NOT FOUND":"FOUND");
	if (sdError) {
		debug_print("Nothing to do. Going to sleep.\n");
		led_blink(1,100,0);		// Blink once to indicate nothing to do
		delay_ms(100);
		system_sleep();
		system_reset();
	}

	// Init the SD card driver (spi port)
	sd_mmc_init();

	// Open the image file
	res = f_open(&file_object,(const char *)image_name, FA_READ);
	if (res != FR_OK) fatal_error(4,"File open failed.\n");

	// Program memory with image
	UINT iRead = 0;
	uint32_t len = 0;
	uint32_t curr_prog_addr = APP_START_ADDRESS;
	do {
		// Read block
		if(file_object.fsize > MAX_CODE_SIZE) fatal_error(6,"File exceeds max size of %d bytes.\n", MAX_CODE_SIZE);
		res = f_read(&file_object, (void *) buff, MAX_BUF_SIZE, &iRead);
		if(res != FR_OK) fatal_error(7,"File read error after %d bytes\n", iRead);
			
		// Program block
		program_memory(curr_prog_addr, buff, iRead);
		debug_print("Programed %u of %u bytes\n", (unsigned int)(len), (unsigned int)(file_object.fsize));

		// Increment
		curr_prog_addr += iRead;
		len += iRead;
		
		// Check size
		if(len > MAX_CODE_SIZE)
		fatal_error(8,"Too many bytes\n");
	} while (iRead != 0);
	
	// Delete image from SD
	res = f_unlink((const char *)image_name);
	if (res != FR_OK) fatal_error(9,"Unable to delete file.\n");
	
	// Trigger re-boot
	system_reset();

	fatal_error(10,"Should not be here\n");
}

// Jump to the application
static void start_application(void) {
	uint32_t *app_check_address_ptr;
	void (*application_code_entry)(void);
	app_check_address_ptr = (uint32_t *) APP_START_ADDRESS;
	
	// Don't jump if the application is blank
	if (*app_check_address_ptr == 0xFFFFFFFF) {
		return;
	}

	// Jump to application
	led_blink(2,200,1);				// Good blink

	// Get the reset vector address
	application_code_entry  = *(uint32_t*)(APP_START_ADDRESS + 4);

	// Set stack pointer, set vector table offset, jump to application
	__set_MSP(*(uint32_t *) APP_START_ADDRESS);
	SCB->VTOR = ((uint32_t) APP_START_ADDRESS & SCB_VTOR_TBLOFF_Msk);
	application_code_entry();
}

// Write a buffer of length len to address in flash memory
static void program_memory(uint32_t address, uint8_t *buffer, uint32_t len) {
	volatile enum status_code code;
	uint32_t offset = 0;
	// Multiples of Page Size
	while (len >= NVMCTRL_PAGE_SIZE) {
		if ((address & 0xFF) == 0) {
			do {
				code = nvm_erase_row(address);
			} while (code == STATUS_BUSY);
			if (code != STATUS_OK) fatal_error(11,"NVM erase row error\n");
		}
		do {
			code = nvm_write_buffer(address, buffer + offset, NVMCTRL_PAGE_SIZE);
		} while (code == STATUS_BUSY);
		if (code != STATUS_OK) fatal_error(12,"NVM write buffer error\n");
		address += NVMCTRL_PAGE_SIZE;
		offset += NVMCTRL_PAGE_SIZE;
		len -= NVMCTRL_PAGE_SIZE;
	}
	// Fragment (leftover) bytes
	if (len > 0) {
		if ((address & 0xFF) == 0) {
			do {
				code = nvm_erase_row(address);
			} while (code == STATUS_BUSY);
			if (code != STATUS_OK) fatal_error(11,"NVM erase row error\n");
		}
		do {
			code = nvm_write_buffer(address, buffer + offset, len);
		} while (code == STATUS_BUSY);
		if (code != STATUS_OK) fatal_error(12,"NVM write buffer error\n");
	}

}

// Toggle LED for blink codes
static void led_blink(int num_blinks, int blink_period_ms, bool green) {
	bool state = 0;
	struct port_config cfg;
	port_get_config_defaults(&cfg);
	while(num_blinks*2 > 0) {
		state = !state;
		if (state) {
			cfg.direction = PORT_PIN_DIR_OUTPUT;
			cfg.input_pull = PORT_PIN_PULL_NONE;
			port_pin_set_output_level(LED_OUT_PIN, !green);
			port_pin_set_config(LED_OUT_PIN, &cfg);
			} else {
			cfg.direction = PORT_PIN_DIR_INPUT;
			cfg.input_pull = PORT_PIN_PULL_NONE;
			port_pin_set_output_level(LED_OUT_PIN, 0);
			port_pin_set_config(LED_OUT_PIN, &cfg);
		}
		delay_ms(blink_period_ms/2);
		num_blinks--;
	}
	cfg.direction = PORT_PIN_DIR_INPUT;
	cfg.input_pull = PORT_PIN_PULL_NONE;
	port_pin_set_output_level(LED_OUT_PIN, 0);
	port_pin_set_config(LED_OUT_PIN, &cfg);
}

// Fatal errors: blink light, print, sleep
static void fatal_error(int numBlinks, const char* fmt, ...) {
	va_list args;
	va_start(args, fmt);
	debug_print(fmt, args);
	va_end(args);
	led_blink(numBlinks, 200, 0);
	delay_ms(100);

	// Sleep, wakes on button press and restarts bootloader
	system_sleep();
	system_reset();
}

static void debug_print(const char* fmt, ...) {
#ifdef ENABLE_USB_DEBUG
	va_list args;
	va_start(args, fmt);
	vprintf(fmt, args);
	va_end(args);
#endif
}