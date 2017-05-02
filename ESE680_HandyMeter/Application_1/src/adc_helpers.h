#ifndef ADC_HELPERS_INCLUDE
#define ADC_HELPERS_INCLUDE

struct spi_module				adc_spi;
struct tc_module				clock_tc;
struct tc_module				post_tc;
struct events_resource			events;
struct dma_resource				tx_spi_dma;
struct dma_resource				rx_spi_dma;
struct dma_resource_config		tx_spi_config;
struct dma_resource_config		rx_spi_config;
struct dma_descriptor_config	tx_spi_desc_config;
struct dma_descriptor_config	rx_spi_desc_config;

COMPILER_ALIGNED(16) DmacDescriptor tx_spi_desc SECTION_DMAC_DESCRIPTOR;
COMPILER_ALIGNED(16) DmacDescriptor rx_spi_desc SECTION_DMAC_DESCRIPTOR;
#define NUM_TRANSFER 9
volatile uint8_t tx_buffer[NUM_TRANSFER];
volatile uint8_t rx_buffer[NUM_TRANSFER];

// ADC command words and register addresses
const uint16_t COMMAND_NULL		= 0x0000;
const uint16_t COMMAND_RESET	= 0x0011;
const uint16_t COMMAND_STANDBY	= 0x0022;
const uint16_t COMMAND_WAKEUP	= 0x0033;
const uint16_t COMMAND_LOCK		= 0x0555;
const uint16_t COMMAND_UNLOCK	= 0x0655;
const uint16_t RREG				= 0x2000;
const uint16_t WREG				= 0x4000;
const uint8_t  ADDR_ID_MSB		= 0x00;
const uint8_t  ADDR_ID_LSB		= 0x01;
const uint8_t  ADDR_STAT_1		= 0x02;
const uint8_t  ADDR_STAT_P		= 0x03;
const uint8_t  ADDR_STAT_N		= 0x04;
const uint8_t  ADDR_STAT_S		= 0x05;
const uint8_t  ADDR_ERROR_CNT	= 0x06;
const uint8_t  ADDR_STAT_M2		= 0x07;
const uint8_t  ADDR_A_SYS_CFG	= 0x0b;
const uint8_t  ADDR_D_SYS_CFG	= 0x0c;
const uint8_t  ADDR_CLK1		= 0x0d;
const uint8_t  ADDR_CLK2		= 0x0e;
const uint8_t  ADDR_ADC_ENA		= 0x0f;
const uint8_t  ADDR_ADC1		= 0x11;
const uint8_t  ADDR_ADC2		= 0x12;

// Preset default configurations for ADC registers
const uint8_t a_sys_cfg	= 0xe8;
const uint8_t d_sys_cfg	= 0x3c;
const uint8_t clk1		= 0x02;
const uint8_t clk2		= 0x20;
const uint8_t adc1		= 0x01;
const uint8_t adc2		= 0x01;

int  adc_synchronous_start();
int  adc_synchronous_stop();
void adc_transfer_start_callback(struct tc_module* module);
void adc_transfer_complete_callback(struct tc_module* module);

// Setup GCLK generator for ADC master clock to frequency
void adc_gclk_config(int hz) {
	int main_clock = system_gclk_gen_get_hz(GCLK_GENERATOR_0);
	int divider = main_clock / hz;
	struct system_gclk_gen_config gclk_conf;
	system_gclk_gen_get_config_defaults(&gclk_conf);
	gclk_conf.source_clock    = SYSTEM_CLOCK_SOURCE_DFLL;
	gclk_conf.division_factor = divider;
	gclk_conf.run_in_standby  = false;
	gclk_conf.output_enable   = true;
	system_gclk_gen_set_config(GCLK_GENERATOR_4, &gclk_conf);
	system_gclk_gen_enable(GCLK_GENERATOR_4);
}

// Enable output of the GCLK4 on the ADC clock pin
void adc_gclk_output_start() {
	struct system_pinmux_config mcfg;
	system_pinmux_get_config_defaults(&mcfg);
	mcfg.direction = SYSTEM_PINMUX_PIN_DIR_OUTPUT;
	mcfg.input_pull = SYSTEM_PINMUX_PIN_PULL_NONE;
	mcfg.mux_position = MUX_PB10H_GCLK_IO4;
	system_pinmux_pin_set_config(ADC_GCLK_MCK_PIN, &mcfg);
	system_pinmux_pin_set_output_strength(ADC_GCLK_MCK_PIN, SYSTEM_PINMUX_PIN_STRENGTH_HIGH);
}

// Disable output of the GCLK4 on the ADC clock pin
void adc_gclk_output_stop() {
	port_pin_set_output_level(ADC_GCLK_MCK_PIN, 0);
	struct system_pinmux_config mcfg;
	system_pinmux_get_config_defaults(&mcfg);
	mcfg.direction = SYSTEM_PINMUX_PIN_DIR_OUTPUT;
	mcfg.input_pull = SYSTEM_PINMUX_PIN_PULL_NONE;
	mcfg.mux_position = SYSTEM_PINMUX_GPIO;
	system_pinmux_pin_set_config(ADC_GCLK_MCK_PIN, &mcfg);
}

// Take control of CS pin from SPI port, set to level
void adc_cs_pin_takeover(bool level) {
	port_pin_set_output_level(ADC_SPI_CS_PIN, level);
	struct system_pinmux_config mcfg;
	system_pinmux_get_config_defaults(&mcfg);
	mcfg.direction = SYSTEM_PINMUX_PIN_DIR_OUTPUT;
	mcfg.input_pull = SYSTEM_PINMUX_PIN_PULL_NONE;
	mcfg.mux_position = SYSTEM_PINMUX_GPIO;
	system_pinmux_pin_set_config(ADC_SPI_CS_PIN, &mcfg);
	system_pinmux_pin_set_output_strength(ADC_SPI_CS_PIN, SYSTEM_PINMUX_PIN_STRENGTH_HIGH);
}

// Give control of CS pin back to SPI port, set to level
void adc_cs_pin_release(bool level) {
	port_pin_set_output_level(ADC_SPI_CS_PIN, level);
	struct system_pinmux_config mcfg;
	system_pinmux_get_config_defaults(&mcfg);
	mcfg.direction = SYSTEM_PINMUX_PIN_DIR_OUTPUT;
	mcfg.input_pull = SYSTEM_PINMUX_PIN_PULL_NONE;
	mcfg.mux_position = MUX_PB22D_SERCOM5_PAD2;
	system_pinmux_pin_set_config(ADC_SPI_CS_PIN, &mcfg);
}

// ADC SPI port
int adc_spi_config(int baud) {
	int r = 0;
	struct spi_config adc_cfg;
	spi_get_config_defaults(&adc_cfg);
	adc_cfg.transfer_mode = SPI_TRANSFER_MODE_1;
	adc_cfg.master_slave_select_enable = 1;
	adc_cfg.mode_specific.master.baudrate = baud;
	adc_cfg.mux_setting = SPI_SIGNAL_MUX_SETTING_I;
	adc_cfg.pinmux_pad0 = PINMUX_PB02D_SERCOM5_PAD0;	// MISO
	adc_cfg.pinmux_pad1 = PINMUX_PB03D_SERCOM5_PAD1;	// SCK
	adc_cfg.pinmux_pad2 = PINMUX_PB22D_SERCOM5_PAD2;	// nCS
	adc_cfg.pinmux_pad3 = PINMUX_PB23D_SERCOM5_PAD3;	// MOSI
	r |= spi_init(&adc_spi, SERCOM5, &adc_cfg);
	spi_enable(&adc_spi);
	return r;
}

// Change the pulse counter (used to change adc sample rate)
void adc_timer_change_pulses(int pulses, int pretrigger) {
	tc_set_compare_value(&clock_tc, TC_COMPARE_CAPTURE_CHANNEL_0, pulses);
	tc_set_compare_value(&clock_tc, TC_COMPARE_CAPTURE_CHANNEL_0, pulses - pretrigger);
	tc_set_compare_value(&post_tc, TC_COMPARE_CAPTURE_CHANNEL_0, pulses);
	tc_set_compare_value(&post_tc, TC_COMPARE_CAPTURE_CHANNEL_0, pulses/2);
}

// ADC timer and event system config to count GCLK pulses and route event to DMA
int adc_timer_config(int pulses, int pretrigger) {
	int r = 0;

	// Timer setup, pre-trigger count
	struct tc_config tcfg;
	tc_get_config_defaults(&tcfg);
	tcfg.clock_source = GCLK_GENERATOR_4;
	tcfg.counter_size = TC_COUNTER_SIZE_16BIT;
	tcfg.wave_generation = TC_WAVE_GENERATION_MATCH_FREQ;
	tcfg.counter_16_bit.value = 0;
	tcfg.counter_16_bit.compare_capture_channel[0] = pulses;
	tcfg.counter_16_bit.compare_capture_channel[1] = pulses - pretrigger;
	r |= tc_init(&clock_tc, TC3, &tcfg);
	r |= tc_register_callback(&clock_tc, adc_transfer_start_callback, TC_CALLBACK_CC_CHANNEL1);
	tc_enable_callback(&clock_tc, TC_CALLBACK_CC_CHANNEL1);
	r |= system_interrupt_set_priority(SYSTEM_INTERRUPT_MODULE_TC3, SYSTEM_INTERRUPT_PRIORITY_LEVEL_0);

	// Timer setup, post-trigger, drop for half the frame
	tcfg.counter_16_bit.compare_capture_channel[0] = pulses;
	tcfg.counter_16_bit.compare_capture_channel[1] = pulses/2;
	r |= tc_init(&post_tc, TC4, &tcfg);
	r |= tc_register_callback(&post_tc, adc_transfer_complete_callback, TC_CALLBACK_CC_CHANNEL1);
	tc_enable_callback(&post_tc, TC_CALLBACK_CC_CHANNEL1);
	r |= system_interrupt_set_priority(SYSTEM_INTERRUPT_MODULE_TC4, SYSTEM_INTERRUPT_PRIORITY_LEVEL_0);

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
	r |= events_allocate(&events, &ecfg);
	r |= events_attach_user(&events, EVSYS_ID_USER_DMAC_CH_0);
	return r;
}

// Configure the DMA structures
int adc_dma_config() {
	int r = 0;
	dma_get_config_defaults(&tx_spi_config);
	dma_get_config_defaults(&rx_spi_config);
	tx_spi_config.peripheral_trigger = SERCOM5_DMAC_ID_TX;
	tx_spi_config.trigger_action = DMA_TRIGGER_ACTION_BEAT;
	tx_spi_config.priority = DMA_PRIORITY_LEVEL_3;
	tx_spi_config.event_config.input_action = DMA_EVENT_INPUT_CBLOCK;
	rx_spi_config.peripheral_trigger = SERCOM5_DMAC_ID_RX;
	rx_spi_config.trigger_action = DMA_TRIGGER_ACTION_BEAT;
	rx_spi_config.priority = DMA_PRIORITY_LEVEL_3;
	rx_spi_config.event_config.input_action = DMA_EVENT_INPUT_NOACT;

	dma_descriptor_get_config_defaults(&tx_spi_desc_config);
	dma_descriptor_get_config_defaults(&rx_spi_desc_config);
	tx_spi_desc_config.beat_size = DMA_BEAT_SIZE_BYTE;
	tx_spi_desc_config.block_transfer_count = NUM_TRANSFER;
	tx_spi_desc_config.source_address = (uint32_t)tx_buffer + sizeof(tx_buffer);
	tx_spi_desc_config.destination_address = (uint32_t)(&SERCOM5->SPI.DATA.reg);
	tx_spi_desc_config.src_increment_enable = 1;
	tx_spi_desc_config.dst_increment_enable = 0;
	tx_spi_desc_config.next_descriptor_address = (uint32_t)(&tx_spi_desc);
	rx_spi_desc_config.beat_size = DMA_BEAT_SIZE_BYTE;
	rx_spi_desc_config.block_transfer_count = NUM_TRANSFER;
	rx_spi_desc_config.source_address = (uint32_t)(&SERCOM5->SPI.DATA.reg);
	rx_spi_desc_config.destination_address = (uint32_t)rx_buffer + sizeof(rx_buffer);
	rx_spi_desc_config.src_increment_enable = 0;
	rx_spi_desc_config.dst_increment_enable = 1;
	rx_spi_desc_config.next_descriptor_address = (uint32_t)(&rx_spi_desc);
	
	r |= dma_allocate(&tx_spi_dma, &tx_spi_config);
	r |= dma_allocate(&rx_spi_dma, &rx_spi_config);
	dma_descriptor_create(&tx_spi_desc, &tx_spi_desc_config);
	dma_descriptor_create(&rx_spi_desc, &rx_spi_desc_config);
	r |= dma_add_descriptor(&tx_spi_dma, &tx_spi_desc);
	r |= dma_add_descriptor(&rx_spi_dma, &rx_spi_desc);

	for(int i=0; i<sizeof(rx_buffer)/sizeof(rx_buffer[0]); i++) {
		rx_buffer[i] = 0;
		tx_buffer[i] = 0;
	}
	return r;
}

// Start the synchronous ADC peripherals (clocks, timer, dma's)
int adc_synchronous_start() {
	int r = 0;
	adc_cs_pin_release(1);
	adc_gclk_output_start();
	r |= dma_start_transfer_job(&tx_spi_dma);
	r |= dma_start_transfer_job(&rx_spi_dma);
	tc_enable(&clock_tc);
	tc_enable(&post_tc);
	return r;
}

// Stop the synchronous ADC peripherals (clocks, timer, dma's)
int adc_synchronous_stop() {
	int r = 0;
	dma_suspend_job(&tx_spi_dma);
	dma_suspend_job(&rx_spi_dma);
	tc_disable(&clock_tc);
	tc_disable(&post_tc);
	adc_gclk_output_stop();
	adc_cs_pin_takeover(1);
	return r;
}

// Called by the timer at start of SPI frame
void adc_transfer_start_callback(struct tc_module* module) {
	adc_cs_pin_takeover(0);
}

// Called by DMA at the end of SPI frame, load results into buffer
void adc_transfer_complete_callback(struct tc_module* module) {
	adc_cs_pin_release(1);

	int32_t s1=0;
	int32_t s2=0;
	int32_t s3=0;
	for (int i=0; i<3; i++) {
		s3 |= rx_buffer[i+0] << ((3-i-1) * 8);
		s1 |= rx_buffer[i+3] << ((3-i-1) * 8);
		s2 |= rx_buffer[i+6] << ((3-i-1) * 8);
	}
	if (s1 != 0 && s2 != 0 && !(s3 & (4<<8))) {
		adc_rangeConvert(&s1, &s2);
		adc_fifo_storeSample(s1, s2, s3);
	}
}

// Configure the hardware peripherals used by ADC sampling process
int adc_peripheralConfig(int gclk_hz, int pretrigger) {
	
	int r = 0;
	r |= adc_spi_config(gclk_hz/2);
	adc_gclk_config(gclk_hz);
	r |= adc_timer_config(gclk_hz/1000-1, pretrigger);
	r |= adc_dma_config();
	adc_cs_pin_takeover(1);

	return r;
}

#endif