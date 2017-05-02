#ifndef ADC_INCLUDE
#define ADC_INCLUDE


enum AdcRange {
	RANGE_1_224=0,
	RANGE_13_46,
	RANGE_81_44,
	RANGE_151_4,
	RANGE_219_4,
	RANGE_290_3,
	RANGE_358_2,
	RANGE_428_2,
	RANGE_496_2,
};

union AdcFaultCode {
	struct {
		bool OPC:1;
		bool SPI:1;
		bool ADCIN:1;
		bool WDTR:1;
		bool RESYNC:1;
		bool DRDY:1;
		bool CHECK:1;
		bool IN2P:1;
		bool IN1P:1;
		bool IN2N:1;
		bool IN1N:1;
		bool STARTUP:1;
		bool CS:1;
		bool FRAME:1;
	} b;
	uint32_t all;
};

enum AdcSampleRate {
	RATE_1000=0x0,
	RATE_2000=0x1,
	RATE_4000=0x2,
	RATE_5120=0x3,
	RATE_5330=0x4,
	RATE_8000=0x5,
	RATE_10240=0x6,
	RATE_10667=0x7,
	RATE_16000=0x8,
	RATE_20480=0x9,
	RATE_21333=0xa,
	RATE_32000=0xb,
	RATE_42667=0xc,
	RATE_64000=0xd,
	RATE_85333=0xe,
	RATE_128000=0xf,
};

int adc_fault(union AdcFaultCode* code);
int adc_start();
int adc_stop();
int adc_setSampleRate(enum AdcSampleRate rate);
enum AdcSampleRate adc_getSampleRate();
int adc_deinit();
int adc_init();

bool adc_fifo_getSample(int32_t* s1, int32_t* s2, int32_t* s3);
bool adc_fifo_peekSample(int offset, int32_t* s1, int32_t* s2, int32_t* s3);
bool adc_fifo_storeSample(int32_t s1, int32_t s2, int32_t s3);
int adc_fifo_getCount();
void adc_fifo_init();

#endif