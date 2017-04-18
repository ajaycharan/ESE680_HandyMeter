#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#define MAX_BUF_SIZE			512					// Block size of each SD card read
#define MAX_CODE_SIZE			0x10000				// Maximum available free space in SAMD (excluding Boot loader)
#define IMAGE_DEFAULT_NAME		"sd_image.bin"		// Filename for the image

#ifdef DEBUG
#define ENABLE_USB_DEBUG
#define APP_START_ADDRESS		0x18000				// Bootloader takes 96KB in debug -O0
#else
#define APP_START_ADDRESS		0x8000				// Bootloader takes 32KB in release -Os
#endif