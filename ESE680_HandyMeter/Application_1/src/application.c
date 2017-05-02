#include "errno.h"
#include "stdarg.h"
#include "application.h"
#include "string.h"
#include "accessory.h"
#include "asf.h"
#include "driver/include/m2m_wifi.h"
#include "driver/include/m2m_periph.h"
#include "socket/include/socket.h"
#include "iot/http/http_client.h"

void debug_print(const char* fmt, ...);
void fatal_error(int numBlinks, const char* fmt, ...);


#define STRING_EOL                      "\r\n"
#define STRING_HEADER                   "-- HTTP file downloader example --"STRING_EOL \
"-- "BOARD_NAME " --"STRING_EOL	\
"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL

static download_state down_state = NOT_READY;
static FATFS fatfs;
static FIL file_object;
static uint32_t http_file_size = 0;
static uint32_t received_file_size = 0;
static char save_file_name[MAIN_MAX_FILE_NAME_LENGTH + 1] = "0:";
struct sw_timer_module swt_module_inst;
struct http_client_module http_client_module_inst;
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

static void configure_iot_sw_timer(void) {
	struct sw_timer_config swt_conf;
	sw_timer_get_config_defaults(&swt_conf);
	sw_timer_init(&swt_module_inst, &swt_conf);
	sw_timer_enable(&swt_module_inst);
}

static void init_state(void)
{
	down_state = NOT_READY;
}

static void clear_state(download_state mask)
{
	down_state &= ~mask;
}


static void add_state(download_state mask)
{
	down_state |= mask;
}

static inline bool is_state_set(download_state mask)
{
	return ((down_state & mask) != 0);
}

static bool is_exist_file(FIL *fp, const char *file_path_name)
{
	if (fp == NULL || file_path_name == NULL) {
		return false;
	}

	FRESULT ret = f_open(&file_object, (char const *)file_path_name, FA_OPEN_EXISTING);
	f_close(&file_object);
	return (ret == FR_OK);
}

static bool rename_to_unique(FIL *fp, char *file_path_name, uint8_t max_len)
{
	#define NUMBRING_MAX (3)
	#define ADDITION_SIZE (NUMBRING_MAX + 1)
	uint16_t i = 1, name_len = 0, ext_len = 0, count = 0;
	char name[MAIN_MAX_FILE_NAME_LENGTH + 1] = {0};
	char ext[MAIN_MAX_FILE_EXT_LENGTH + 1] = {0};
	char numbering[NUMBRING_MAX + 1] = {0};
	char *p = NULL;
	bool valid_ext = false;

	if (file_path_name == NULL) {
		return false;
	}

	if (!is_exist_file(fp, file_path_name)) {
		return true;
	} 
	else if (strlen(file_path_name) > MAIN_MAX_FILE_NAME_LENGTH) {
		return false;
	}

	p = strrchr(file_path_name, '.');
	if (p != NULL) {
		ext_len = strlen(p);
		if (ext_len < MAIN_MAX_FILE_EXT_LENGTH) {
			valid_ext = true;
			strcpy(ext, p);
			if (strlen(file_path_name) - ext_len > MAIN_MAX_FILE_NAME_LENGTH - ADDITION_SIZE) {
				name_len = MAIN_MAX_FILE_NAME_LENGTH - ADDITION_SIZE - ext_len;
				strncpy(name, file_path_name, name_len);
			} 
			else {
				name_len = (p - file_path_name);
				strncpy(name, file_path_name, name_len);
			}
		} 
		else {
			name_len = MAIN_MAX_FILE_NAME_LENGTH - ADDITION_SIZE;
			strncpy(name, file_path_name, name_len);
		}
	} 
	else {
		name_len = MAIN_MAX_FILE_NAME_LENGTH - ADDITION_SIZE;
		strncpy(name, file_path_name, name_len);
	}

	name[name_len++] = '-';

	for (i = 0, count = 1; i < NUMBRING_MAX; i++) {
		count *= 10;
	}
	for (i = 1; i < count; i++) {
		sprintf(numbering, MAIN_ZERO_FMT(NUMBRING_MAX), i);
		strncpy(&name[name_len], numbering, NUMBRING_MAX);
		if (valid_ext) {
			strcpy(&name[name_len + NUMBRING_MAX], ext);
		}

		if (!is_exist_file(fp, name)) {
			memset(file_path_name, 0, max_len);
			strcpy(file_path_name, name);
			return true;
		}
	}
	return false;
}


static void start_download(void)
{
	if (!is_state_set(STORAGE_READY)) {
		printf("start_download: MMC storage not ready.\r\n");
		return;
	}

	if (!is_state_set(WIFI_CONNECTED)) {
		printf("start_download: Wi-Fi is not connected.\r\n");
		return;
	}

	if (is_state_set(GET_REQUESTED)) {
		printf("start_download: request is sent already.\r\n");
		return;
	}

	if (is_state_set(DOWNLOADING)) {
		printf("start_download: running download already.\r\n");
		return;
	}

	printf("start_download: sending HTTP request...\r\n");
	http_client_send_request(&http_client_module_inst, MAIN_HTTP_FILE_URL, HTTP_METHOD_GET, NULL, NULL);
}

static void store_file_packet(char *data, uint32_t length)
{
	FRESULT ret;
	if ((data == NULL) || (length < 1)) {
		printf("store_file_packet: empty data.\r\n");
		return;
	}

	if (!is_state_set(DOWNLOADING)) {
		char *cp = NULL;
		save_file_name[0] = LUN_ID_SD_MMC_0_MEM + '0';
		save_file_name[1] = ':';
		cp = (char *)(MAIN_HTTP_FILE_URL + strlen(MAIN_HTTP_FILE_URL));
		while (*cp != '/') {
			cp--;
		}
		if (strlen(cp) > 1) {
			cp++;
			strcpy(&save_file_name[2], cp);
		} else {
			printf("store_file_packet: file name is invalid. Download canceled.\r\n");
			add_state(CANCELED);
			return;
		}

		rename_to_unique(&file_object, save_file_name, MAIN_MAX_FILE_NAME_LENGTH);
		printf("store_file_packet: creating file [%s]\r\n", save_file_name);
		ret = f_open(&file_object, (char const *)save_file_name, FA_CREATE_ALWAYS | FA_WRITE);
		if (ret != FR_OK) {
			printf("store_file_packet: file creation error! ret:%d\r\n", ret);
			return;
		}

		received_file_size = 0;
		add_state(DOWNLOADING);
	}

	if (data != NULL) {
		UINT wsize = length;
		ret = f_write(&file_object, (const void *)data, length, &wsize);
		if (ret != FR_OK) {
			f_close(&file_object);
			add_state(CANCELED);
			printf("store_file_packet: file write error, download canceled.\r\n");
			return;
		}

		received_file_size += wsize;
		printf("store_file_packet: received[%lu], file size[%lu]\r\n", (unsigned long)received_file_size, (unsigned long)http_file_size);
		if (received_file_size >= http_file_size) {
			f_close(&file_object);
			printf("store_file_packet: file downloaded successfully.\r\n");
			add_state(COMPLETED);
			return;
		}
	}
}


static void http_client_callback(struct http_client_module *module_inst, int type, union http_client_data *data)
{
	switch (type) {
	case HTTP_CLIENT_CALLBACK_SOCK_CONNECTED:
		printf("http_client_callback: HTTP client socket connected.\r\n");
		break;

	case HTTP_CLIENT_CALLBACK_REQUESTED:
		printf("http_client_callback: request completed.\r\n");
		add_state(GET_REQUESTED);
		break;

	case HTTP_CLIENT_CALLBACK_RECV_RESPONSE:
		printf("http_client_callback: received response %u data size %u\r\n",
				(unsigned int)data->recv_response.response_code,
				(unsigned int)data->recv_response.content_length);
		if ((unsigned int)data->recv_response.response_code == 200) {
			http_file_size = data->recv_response.content_length;
			received_file_size = 0;
		} 
		else {
			add_state(CANCELED);
			return;
		}
		if (data->recv_response.content_length <= MAIN_BUFFER_MAX_SIZE) {
			store_file_packet(data->recv_response.content, data->recv_response.content_length);
			add_state(COMPLETED);
		}
		break;

	case HTTP_CLIENT_CALLBACK_RECV_CHUNKED_DATA:
		store_file_packet(data->recv_chunked_data.data, data->recv_chunked_data.length);
		if (data->recv_chunked_data.is_complete) {
			add_state(COMPLETED);
		}

		break;

	case HTTP_CLIENT_CALLBACK_DISCONNECTED:
		printf("http_client_callback: disconnection reason:%d\r\n", data->disconnected.reason);

		if (data->disconnected.reason == -EAGAIN) {
			if (is_state_set(DOWNLOADING)) {
				f_close(&file_object);
				clear_state(DOWNLOADING);
			}

			if (is_state_set(GET_REQUESTED)) {
				clear_state(GET_REQUESTED);
			}

			start_download();
		}

		break;
	}
}


static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
	http_client_socket_event_handler(sock, u8Msg, pvMsg);
}


static void resolve_cb(uint8_t *pu8DomainName, uint32_t u32ServerIP)
{
	printf("resolve_cb: %s IP address is %d.%d.%d.%d\r\n\r\n", pu8DomainName,
			(int)IPV4_BYTE(u32ServerIP, 0), (int)IPV4_BYTE(u32ServerIP, 1),
			(int)IPV4_BYTE(u32ServerIP, 2), (int)IPV4_BYTE(u32ServerIP, 3));
	http_client_socket_resolve_handler(pu8DomainName, u32ServerIP);
}


static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType) {
	case M2M_WIFI_RESP_CON_STATE_CHANGED:
	{
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
			printf("wifi_cb: M2M_WIFI_CONNECTED\r\n");
			m2m_wifi_request_dhcp_client();
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
			printf("wifi_cb: M2M_WIFI_DISCONNECTED\r\n");
			clear_state(WIFI_CONNECTED);
			if (is_state_set(DOWNLOADING)) {
				f_close(&file_object);
				clear_state(DOWNLOADING);
			}

			if (is_state_set(GET_REQUESTED)) {
				clear_state(GET_REQUESTED);
			}

			m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID),
					MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
		}

		break;
	}

	case M2M_WIFI_REQ_DHCP_CONF:
	{
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		printf("wifi_cb: IP address is %u.%u.%u.%u\r\n",
				pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
		add_state(WIFI_CONNECTED);
		start_download();
		break;
	}

	default:
		break;
	}
}


static void configure_http_client(void)
{
	struct http_client_config httpc_conf;
	int ret;

	http_client_get_config_defaults(&httpc_conf);

	httpc_conf.recv_buffer_size = MAIN_BUFFER_MAX_SIZE;
	httpc_conf.timer_inst = &swt_module_inst;

	ret = http_client_init(&http_client_module_inst, &httpc_conf);
	if (ret < 0) {
		printf("configure_http_client: HTTP client initialization failed! (res %d)\r\n", ret);
		while (1) {
		}
	}

	http_client_register_callback(&http_client_module_inst, http_client_callback);
}

static void init_storage(void)
{
	FRESULT res;
	Ctrl_status status;

	sd_mmc_init();
	while (1) {
		do {
			status = sd_mmc_test_unit_ready(0);
			if (CTRL_FAIL == status) {
				printf("init_storage: SD Card install failed.\r\n");
				printf("init_storage: try unplug and re-plug the card.\r\n");
				while (CTRL_NO_PRESENT != sd_mmc_check(0)) {
				}
			}
		} while (CTRL_GOOD != status);

		printf("init_storage: mounting SD card...\r\n");
		memset(&fatfs, 0, sizeof(FATFS));
		res = f_mount(LUN_ID_SD_MMC_0_MEM, &fatfs);
		if (FR_INVALID_DRIVE == res) {
			printf("init_storage: SD card mount failed! (res %d)\r\n", res);
			return;
		}
		printf("init_storage: SD card mount OK.\r\n");
		add_state(STORAGE_READY);
		return;
	}
}

COMPILER_ALIGNED(16) DmacDescriptor tx_spi_desc SECTION_DMAC_DESCRIPTOR;
COMPILER_ALIGNED(16) DmacDescriptor rx_spi_desc SECTION_DMAC_DESCRIPTOR;

struct dma_resource tx_spi_dma;
struct dma_resource rx_spi_dma;

struct tc_module clock_tc;
struct spi_module adc_spi;
struct events_resource events;
volatile bool transfer_is_done = 0;

static void transfer_done(struct dma_resource* const resource )
{
	transfer_is_done = true;

	// Raise the CS pin and release it back to hardware SPI
	port_pin_set_output_level(ADC_SPI_CS_PIN, 1);
	struct system_pinmux_config mcfg;
	system_pinmux_get_config_defaults(&mcfg);
	mcfg.direction = SYSTEM_PINMUX_PIN_DIR_OUTPUT;
	mcfg.input_pull = SYSTEM_PINMUX_PIN_PULL_NONE;
	mcfg.mux_position = MUX_PB22D_SERCOM5_PAD2;
	system_pinmux_pin_set_config(ADC_SPI_CS_PIN, &mcfg);
}
static void transfer_start(struct tc_module* const resource )
{
	// Take over the CS pin and hold it low
	struct system_pinmux_config mcfg;
	system_pinmux_get_config_defaults(&mcfg);
	mcfg.direction = SYSTEM_PINMUX_PIN_DIR_OUTPUT;
	mcfg.input_pull = SYSTEM_PINMUX_PIN_PULL_NONE;
	mcfg.mux_position = MUX_PB22A_EIC_EXTINT6;
	port_pin_set_output_level(ADC_SPI_CS_PIN, 0);
	system_pinmux_pin_set_config(ADC_SPI_CS_PIN, &mcfg);
}
static void configure_dma_resources()
{
	struct dma_resource_config config;
	dma_get_config_defaults(&config);
	config.peripheral_trigger = SERCOM5_DMAC_ID_TX;
	config.trigger_action = DMA_TRIGGER_ACTION_BEAT;
	config.priority = DMA_PRIORITY_LEVEL_3;
	config.event_config.input_action = DMA_EVENT_INPUT_CBLOCK;
	dma_allocate(&tx_spi_dma, &config);
	config.priority = DMA_PRIORITY_LEVEL_3;
	config.peripheral_trigger = SERCOM5_DMAC_ID_RX;
	config.event_config.input_action = DMA_EVENT_INPUT_NOACT;
	dma_allocate(&rx_spi_dma, &config);
}
uint8_t tx_buffer[9];
uint8_t rx_buffer[9];
static void setup_transfer_descriptor()
{
	struct dma_descriptor_config descriptor_config;
	dma_descriptor_get_config_defaults(&descriptor_config);
	descriptor_config.beat_size = DMA_BEAT_SIZE_BYTE;
	descriptor_config.block_transfer_count = 9;
	descriptor_config.source_address = (uint32_t)tx_buffer + sizeof(tx_buffer);
	descriptor_config.destination_address = (uint32_t)(&SERCOM5->SPI.DATA.reg);
	descriptor_config.dst_increment_enable = 0;
	descriptor_config.src_increment_enable = 1;
	dma_descriptor_create(&tx_spi_desc, &descriptor_config);

	descriptor_config.source_address = (uint32_t)(&SERCOM5->SPI.DATA.reg);
	descriptor_config.destination_address = (uint32_t)rx_buffer + sizeof(rx_buffer);
	descriptor_config.src_increment_enable = 0;
	descriptor_config.dst_increment_enable = 1;
	dma_descriptor_create(&rx_spi_desc, &descriptor_config);
}

int main(void)
{
	system_init();
	delay_init();
	configure_bod();
	configure_rtc();
	configure_wakeup();
	sleepmgr_init();
	stdio_usb_init();
	stdio_usb_enable();
	system_interrupt_disable_global();
	for (int i=0; i<PERIPH_COUNT_IRQn-1; i++) {
		system_interrupt_set_priority(i, SYSTEM_INTERRUPT_PRIORITY_LEVEL_3);
	}
	system_interrupt_set_priority(SysTick_IRQn, SYSTEM_INTERRUPT_PRIORITY_LEVEL_3);
	system_interrupt_set_priority(PendSV_IRQn, SYSTEM_INTERRUPT_PRIORITY_LEVEL_3);
	system_interrupt_set_priority(SVCall_IRQn, SYSTEM_INTERRUPT_PRIORITY_LEVEL_3);
	system_interrupt_set_priority(HardFault_IRQn, SYSTEM_INTERRUPT_PRIORITY_LEVEL_3);
	system_interrupt_set_priority(NonMaskableInt_IRQn, SYSTEM_INTERRUPT_PRIORITY_LEVEL_3);
	system_interrupt_enable_global();
	delay_ms(500);
	
	// QOS change
	uint32_t *cpu = (uint32_t*)(0x41007110);
	*cpu &= ~3;
	*cpu |= 2;

	USB->DEVICE.QOSCTRL.bit.CQOS = 0;
	USB->DEVICE.QOSCTRL.bit.DQOS = 0;

	DMAC->QOSCTRL.bit.DQOS = 3;
	DMAC->QOSCTRL.bit.FQOS = 3;
	DMAC->QOSCTRL.bit.WRBQOS = 3;

	accy_powerUp();
	
	// CS pin setup
	struct port_config pcfg;
	port_get_config_defaults(&pcfg);
	pcfg.direction = PORT_PIN_DIR_OUTPUT;
	pcfg.input_pull = PORT_PIN_PULL_NONE;
	port_pin_set_config(ADC_SPI_CS_PIN, &pcfg);
	port_pin_set_output_level(ADC_SPI_CS_PIN, 1);

	// SPI port setup
	struct spi_config adc_cfg;
	spi_get_config_defaults(&adc_cfg);
	adc_cfg.transfer_mode = SPI_TRANSFER_MODE_1;
	adc_cfg.master_slave_select_enable = 1;
	adc_cfg.mode_specific.master.baudrate = 4096000;
	adc_cfg.mux_setting = SPI_SIGNAL_MUX_SETTING_I;
	adc_cfg.pinmux_pad0 = PINMUX_PB02D_SERCOM5_PAD0;	// MISO
	adc_cfg.pinmux_pad1 = PINMUX_PB03D_SERCOM5_PAD1;	// SCK
	adc_cfg.pinmux_pad2 = PINMUX_PB22D_SERCOM5_PAD2;	// nCS
	adc_cfg.pinmux_pad3 = PINMUX_PB23D_SERCOM5_PAD3;	// MOSI
	enum status_code code = spi_init(&adc_spi, SERCOM5, &adc_cfg);
	if (code != STATUS_OK) fatal_error(0,"ADC spi init failed\n");
	spi_enable(&adc_spi);

	// Timer setup to count pulses
	struct tc_config tcfg;
	tc_get_config_defaults(&tcfg);
	tcfg.clock_source = GCLK_GENERATOR_4;
	tcfg.counter_size = TC_COUNTER_SIZE_16BIT;
	tcfg.wave_generation = TC_WAVE_GENERATION_MATCH_FREQ;
	tcfg.counter_16_bit.value = 0;
	tcfg.counter_16_bit.compare_capture_channel[0] = 8192;
	tcfg.counter_16_bit.compare_capture_channel[1] = 8192-23;
	tc_init(&clock_tc, TC3, &tcfg);
	tc_register_callback(&clock_tc, transfer_start, TC_CALLBACK_CC_CHANNEL1);
	tc_enable_callback(&clock_tc, TC_CALLBACK_CC_CHANNEL1);
	system_interrupt_set_priority(SYSTEM_INTERRUPT_MODULE_TC3, SYSTEM_INTERRUPT_PRIORITY_LEVEL_0);

	// Make event on compare
	struct tc_events tev;
	tev.event_action = TC_EVENT_ACTION_OFF;
	tev.on_event_perform_action = 0;
	tev.invert_event_input = 0;
	tev.generate_event_on_overflow = 0;
	tev.generate_event_on_compare_channel[0] = 1;
	tev.generate_event_on_compare_channel[1] = 0;
	tc_enable_events(&clock_tc, &tev);
	
	// Pipe event to DMA
	struct events_config ecfg;
	events_get_config_defaults(&ecfg);
	ecfg.generator = EVSYS_ID_GEN_TC3_MCX_0;
	ecfg.path = EVENTS_PATH_RESYNCHRONIZED;
	events_allocate(&events, &ecfg);
	events_attach_user(&events, EVSYS_ID_USER_DMAC_CH_0);

	for (int i=0; i<9;i++) {
		if (i % 2 == 0)
			tx_buffer[i] = 0x3c;
		else
			tx_buffer[i] = 0xc3;

		rx_buffer[i] = 0x55;
	}

	// Enable GCLK output
	struct system_pinmux_config mcfg;
	system_pinmux_get_config_defaults(&mcfg);
	mcfg.direction = SYSTEM_PINMUX_PIN_DIR_OUTPUT;
	mcfg.input_pull = SYSTEM_PINMUX_PIN_PULL_NONE;
	mcfg.mux_position = MUX_PB10H_GCLK_IO4;
	system_pinmux_pin_set_config(ADC_GCLK_MCK_PIN, &mcfg);
	system_pinmux_pin_set_output_strength(ADC_GCLK_MCK_PIN, SYSTEM_PINMUX_PIN_STRENGTH_HIGH);

	tc_enable(&clock_tc);
	int cnt = 0;

	// Configure tx DMA to transfer 9 bytes
	configure_dma_resources();
	setup_transfer_descriptor();
	dma_add_descriptor(&tx_spi_dma, &tx_spi_desc);
	dma_add_descriptor(&rx_spi_dma, &rx_spi_desc);
	dma_register_callback(&tx_spi_dma, transfer_done, DMA_CALLBACK_TRANSFER_DONE);
	dma_enable_callback(&tx_spi_dma, DMA_CALLBACK_TRANSFER_DONE);
	dma_enable_callback(&rx_spi_dma, DMA_CALLBACK_TRANSFER_DONE);
	system_interrupt_set_priority(SYSTEM_INTERRUPT_MODULE_DMA, SYSTEM_INTERRUPT_PRIORITY_LEVEL_0);
	dma_start_transfer_job(&tx_spi_dma);
	dma_start_transfer_job(&rx_spi_dma);

	while(1) {


		transfer_is_done = 0;

		while (!transfer_is_done) {
		}

//		dma_free(&rx_spi_dma);
//		dma_free(&tx_spi_dma);
		cnt++;
		dma_start_transfer_job(&tx_spi_dma);
		dma_start_transfer_job(&rx_spi_dma);

		if (cnt % 100 == 0)
			printf("C: %d\n", cnt);
	}
	
	while(1);
	return 0;
}


// Toggle LED for blink codes
static void led_blink(int num_blinks, int blink_period_ms, bool green) {
	bool state = 0;
	struct port_config cfg;
	port_get_config_defaults(&cfg);
	num_blinks = num_blinks * 2;
	while(num_blinks > 0) {
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
void fatal_error(int numBlinks, const char* fmt, ...) {
	va_list args;
	va_start(args, fmt);
	debug_print(fmt, args);
	va_end(args);
	led_blink(numBlinks, 200, 0);
	delay_ms(500);

	// Sleep, wakes on button press
	sleepmgr_sleep(SLEEPMGR_STANDBY);
	system_reset();
}

void debug_print(const char* fmt, ...) {
	#ifdef ENABLE_USB_DEBUG
	va_list args;
	va_start(args, fmt);
	vprintf(fmt, args);
	va_end(args);
	#endif
}