#ifndef ADC_RANGES_INCLUDE
#define ADC_RANGES_INCLUDE

// For each range setting, multiply raw value (24-bit FS) by this number, right-shift by 24 to get value scaled to microvolts
const int32_t rangeConversionTable[9] = {
	2448000,
	26920000,
	162880000,
	302800000,
	438800000,
	580600000,
	716400000,
	856400000,
	992400000,
};

enum AdcRange current_range_ch1 = RANGE_1_224;
enum AdcRange current_range_ch2 = RANGE_1_224;

// Changes val to be in units of uV, based on current range setting
bool adc_rangeConvert(int32_t* v1, int32_t* v2) {
	uint32_t vu1 = *v1 << (32-24);
	uint32_t vu2 = *v2 << (32-24);
	int32_t vs1 = (int32_t)(vu1) >> (32-24);
	int32_t vs2 = (int32_t)(vu2) >> (32-24);
	int64_t vl1 = vs1;
	int64_t vl2 = vs2;
	if (vl1 > 8388607 || vl1 < -8388608) return false;
	if (vl2 > 8388607 || vl2 < -8388608) return false;
	vl1 = vl1 * rangeConversionTable[current_range_ch1];
	vl2 = vl2 * rangeConversionTable[current_range_ch2];
	*v1 = (vl1 >> 24);
	*v2 = (vl2 >> 24);
	return true;
}

// Set switches to a new range setting, -1 to skip setting
bool adc_rangeSet(enum AdcRange r1, enum AdcRange r2) {
	if (r1 > 8 || r2 > 8) return false;
	if (r1 < -1|| r2 <-1) return false;

	// Set the outputs
	if (r1 != -1) {
		if (r1 & 1)		port_pin_set_output_level(SW1A_GP_OUT_PIN, 0);
		else			port_pin_set_output_level(SW1A_GP_OUT_PIN, 1);
		if (r1 & 2)		port_pin_set_output_level(SW2A_GP_OUT_PIN, 0);
		else			port_pin_set_output_level(SW2A_GP_OUT_PIN, 1);
		if (r1 & 4)		port_pin_set_output_level(SW3A_GP_OUT_PIN, 0);
		else			port_pin_set_output_level(SW3A_GP_OUT_PIN, 1);
		if (r1 & 8)		port_pin_set_output_level(SW4A_GP_OUT_PIN, 0);
		else			port_pin_set_output_level(SW4A_GP_OUT_PIN, 1);
		current_range_ch1 = r1;
	}
	if (r2 != -1) {
		if (r2 & 1)		port_pin_set_output_level(SW1B_GP_OUT_PIN, 0);
		else			port_pin_set_output_level(SW1B_GP_OUT_PIN, 1);
		if (r2 & 2)		port_pin_set_output_level(SW2B_GP_OUT_PIN, 0);
		else			port_pin_set_output_level(SW2B_GP_OUT_PIN, 1);
		if (r2 & 4)		port_pin_set_output_level(SW3B_GP_OUT_PIN, 0);
		else			port_pin_set_output_level(SW3B_GP_OUT_PIN, 1);
		if (r2 & 8)		port_pin_set_output_level(SW4B_GP_OUT_PIN, 0);
		else			port_pin_set_output_level(SW4B_GP_OUT_PIN, 1);
		current_range_ch2 = r2;
	}
	// Special case mV range load resistor
	// if (r1 == 0)	set_winc_gpio(0);
	// if (r2 == 0)	set_winc_gpio(0);

	return true;
}

#endif