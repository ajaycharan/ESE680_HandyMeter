#include "asf.h"
#include "stdlib.h"
#include "adc.h"
#include "string.h"
#include "fifo_buffer.h"

extern void fatal_error(int numBlinks, const char* fmt, ...);
extern void debug_print(const char* fmt, ...);

bool adc_started=0;
enum AdcSampleRate adc_sampleRate = RATE_1000;
struct FifoBuffer adc_sampleBuffer;

bool adc_fifo_getSample(int32_t* s1, int32_t* s2, int32_t* s3) {
	return fifo_pop(&adc_sampleBuffer, s1, s2, s3);
}
bool adc_fifo_peekSample(int offset, int32_t* s1, int32_t* s2, int32_t* s3) {
	return fifo_peek(&adc_sampleBuffer, offset, s1, s2, s3);
}
bool adc_fifo_storeSample(int32_t s1, int32_t s2, int32_t s3) {
	return fifo_push(&adc_sampleBuffer, s1, s2, s3);
}
int adc_fifo_getCount() {
	return fifo_numItems(&adc_sampleBuffer);
}
void adc_fifo_init() {
	fifo_init(&adc_sampleBuffer);
}

#include "adc_ranges.h"
#include "adc_helpers.h"

int adc_spi_command_response(uint16_t command) {
	if (adc_started) return -1;

	uint8_t tx[3] = {0};
	uint8_t rx[3] = {0};
	tx[0] = command >> 8;
	tx[1] = command >> 0;
	tx[2] = 0;
	port_pin_set_output_level(ADC_SPI_CS_PIN, 0);
	spi_write_buffer_wait(&adc_spi, tx, 3);
	port_pin_set_output_level(ADC_SPI_CS_PIN, 1);
	port_pin_set_output_level(ADC_SPI_CS_PIN, 0);
	spi_read_buffer_wait(&adc_spi, rx, 3, 0);
	port_pin_set_output_level(ADC_SPI_CS_PIN, 1);
	uint32_t ret = (rx[0] << 8) | (rx[1] << 0);
	return (int)(ret);
}

int adc_spi_readReg(uint8_t addr) {
	if (adc_started) return -1;

	uint16_t comm = RREG | (addr << 8);
	int resp;
	resp = adc_spi_command_response(comm);
	if ((resp & ~0xff) != comm) return -1;
	return resp & 0xff;
}

int adc_spi_writeReg(uint8_t addr, uint8_t data) {
	if (adc_started) return -1;

	uint16_t comm = WREG | (addr << 8) | (data << 0);
	int resp;
	resp = adc_spi_command_response(comm);
	if ((resp & 0xff) != data) return -1;
	if ((resp & ~0xff) != (RREG | (addr << 8))) return -1;
	return 0;
}

int adc_command(uint16_t command) {
	if (adc_started) return -1;

	int resp;
	resp = adc_spi_command_response(command);
	if (resp != command) return -1;
	return 0;
}

int adc_reset() {
	if (adc_started) return -1;

	int cnt=0;
	int resp;
	resp = adc_spi_command_response(COMMAND_RESET);

	// Reset takes 5ms, poll for ready word
	while(resp != 0xff02) {
		cnt++;
		if (cnt > 500) return -1;
		resp = adc_spi_command_response(COMMAND_NULL);
	}
	return 0;
}

int adc_fault(union AdcFaultCode* code) {
	if (adc_started) return -1;

	int resp;
	code->all = 0;
	resp = adc_spi_readReg(ADDR_STAT_1);
	if (resp == -1) return -1;
	code->b.OPC		= resp & (1 << 6);
	code->b.SPI		= resp & (1 << 5);
	code->b.ADCIN	= resp & (1 << 4);
	code->b.WDTR	= resp & (1 << 3);
	code->b.RESYNC	= resp & (1 << 2);
	code->b.DRDY	= resp & (1 << 1);
	code->b.CHECK	= resp & (1 << 0);
	resp = adc_spi_readReg(ADDR_STAT_P);
	if (resp == -1) return -1;
	code->b.IN2P	=  resp & (1 << 1);
	code->b.IN1P	=  resp & (1 << 0);
	resp = adc_spi_readReg(ADDR_STAT_N);
	if (resp == -1) return -1;
	code->b.IN2N	=  resp & (1 << 1);
	code->b.IN1N	=  resp & (1 << 0);
	resp = adc_spi_readReg(ADDR_STAT_S);
	if (resp == -1) return -1;
	code->b.STARTUP	=  resp & (1 << 2);
	code->b.CS		=  resp & (1 << 1);
	code->b.FRAME	=  resp & (1 << 0);
	return 0;
}

int adc_configure() {
	if (adc_started) return -1;

	if (adc_spi_writeReg(ADDR_A_SYS_CFG, a_sys_cfg) == -1) return -1;
	if (adc_spi_writeReg(ADDR_D_SYS_CFG, d_sys_cfg) == -1) return -1;
	if (adc_spi_writeReg(ADDR_CLK1, clk1) == -1) return -1;
	if (adc_spi_writeReg(ADDR_CLK2, clk2) == -1) return -1;
	if (adc_spi_writeReg(ADDR_ADC1, adc1) == -1) return -1;
	if (adc_spi_writeReg(ADDR_ADC2, adc2) == -1) return -1;
	return 0;
}

int adc_start() {
	if (adc_started) return -1;

	if (adc_command(COMMAND_UNLOCK) == -1) return -1;
	if (adc_spi_writeReg(ADDR_ADC_ENA, 0x3) == -1) return -1;
	if (adc_command(COMMAND_LOCK) == -1) return -1;
	adc_synchronous_start();
	adc_started=1;
	return 0;
}

int adc_stop() {
	if (!adc_started) return -1;

	adc_synchronous_stop();
	if (adc_command(COMMAND_UNLOCK) == -1) return -1;
	if (adc_spi_writeReg(ADDR_ADC_ENA, 0x0) == -1) return -1;
	if (adc_command(COMMAND_LOCK) == -1) return -1;
	adc_started=0;
	return 0;
}

int adc_setSampleRate(enum AdcSampleRate rate) {
	if (adc_started) {
		adc_stop();
	}
	uint8_t data = clk2;
	data |= rate;
	if (adc_spi_writeReg(ADDR_CLK2, data) == -1) return -1;
	adc_sampleRate = rate;
	
	if (adc_started) {
		adc_start();
	}
	return 0;
}

enum AdcSampleRate adc_getSampleRate() {
	return adc_sampleRate;
};


// startup and configure the adc
int adc_init() {

	adc_fifo_init();
	adc_rangeSet(RANGE_1_224, RANGE_1_224);
	if (adc_peripheralConfig(8192000, 25))	fatal_error(2,"ADC peripheral config failed.\n");

	// Setup procedure
	if (adc_reset())					fatal_error(2,"ADC init reset failed.\n");
	if (adc_command(COMMAND_UNLOCK))	fatal_error(2,"ADC init unlock failed.\n");
	if (adc_configure())				fatal_error(2,"ADC init config failed.\n");
	if (adc_setSampleRate(RATE_2000))	fatal_error(2,"ADC init sample rate failed.\n");
	if (adc_command(COMMAND_WAKEUP))	fatal_error(2,"ADC init wakeup failed.\n");
	if (adc_command(COMMAND_LOCK))		fatal_error(2,"ADC init lock failed.\n");
	
	// clear and check faults
	union AdcFaultCode fcode;
	fcode.all = 0;
	if (adc_fault(&fcode))				fatal_error(2,"ADC init fault check failed.\n");
	if (adc_fault(&fcode))				fatal_error(2,"ADC init fault check failed.\n");
	if (fcode.all) {
		fatal_error(1,"ADC has faults after init: 0x%04x\n", fcode.all);
	}
	debug_print("ADC init success.\n");
	
	if (adc_start())					fatal_error(2,"ADC init start failed.\n");

	return 0;
}

int adc_deinit() {
	adc_stop();
	spi_disable(&adc_spi);
	spi_reset(&adc_spi);
	tc_disable(&clock_tc);
	tc_reset(&clock_tc);
	tc_disable(&post_tc);
	tc_reset(&post_tc);
	adc_fifo_init();
	return 0;
}