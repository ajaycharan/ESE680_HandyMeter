/**
 * \file
 *
 * \brief User board configuration template
 *
 * Copyright (C) 2013-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#ifndef CONF_BOARD_H
#define CONF_BOARD_H

// Pin definitions and multiplexer settings
// SD card
#define SD_SPI_MISO_PIN			PIN_PA16
#define SD_SPI_MISO_PORT		PORT_PA16
#define SD_SPI_SCK_PIN			PIN_PA17
#define SD_SPI_SCK_PORT			PORT_PA17
#define SD_SPI_CS_PIN			PIN_PA18
#define SD_SPI_CS_PORT			PORT_PA18
#define SD_SPI_MOSI_PIN			PIN_PA19
#define SD_SPI_MOSI_PORT		PORT_PA19

// Logic inputs
#define LOGIC1_TC_CAP_PIN		PIN_PA20
#define LOGIC1_TC_CAP_PORT		PORT_PA20
#define LOGIC2_TC_CAP_PIN		PIN_PA21
#define LOGIC2_TC_CAP_PORT		PORT_PA21

// USB (or UART header)
#define USB_DN_UART_TX_PIN		PIN_PA24
#define USB_DN_UART_TX_PORT		PORT_PA24
#define USB_DP_UART_RX_PIN		PIN_PA25
#define USB_DP_UART_RX_PORT		PORT_PA25
#define CONF_BOARD_USB_PORT

// External ADC
#define ADC_SPI_MISO_PIN		PIN_PB02
#define ADC_SPI_MISO_PORT		PORT_PB02
#define ADC_SPI_MOSI_PIN		PIN_PB23
#define ADC_SPI_MOSI_PORT		PORT_PB23
#define ADC_SPI_SCK_PIN			PIN_PB03
#define ADC_SPI_SCK_PORT		PORT_PB03
#define ADC_SPI_CS_PIN			PIN_PB22
#define ADC_SPI_CS_PORT			PORT_PB22
#define ADC_GCLK_MCK_PIN		PIN_PB10
#define ADC_GCLK_MCK_PORT		PORT_PB10

// Buttons
#define BUT1_IRQ_IN_PIN			PIN_PA03
#define BUT1_IRQ_IN_PORT		PORT_PA03
#define BUT2_IRQ_IN_PIN			PIN_PA31
#define BUT2_IRQ_IN_PORT		PORT_PA31

// Battery sense
#define BATSENSE_ADC_PIN		PIN_PA02
#define BATSENSE_ADC_PORT		PORT_PA02

// LEDs (high=Red, low=Green, none=off)
#define LED_OUT_PIN				PIN_PB11
#define LED_OUT_PORT			PORT_PB11

// Range switches
#define SW1A_GP_OUT_PIN			PIN_PA04
#define SW1A_GP_OUT_PORT		PORT_PA04
#define SW2A_GP_OUT_PIN			PIN_PA05
#define SW2A_GP_OUT_PORT		PORT_PA05
#define SW3A_GP_OUT_PIN			PIN_PA06
#define SW3A_GP_OUT_PORT		PORT_PA06
#define SW4A_GP_OUT_PIN			PIN_PA07
#define SW4A_GP_OUT_PORT		PORT_PA07
#define SW1B_GP_OUT_PIN			PIN_PA08
#define SW1B_GP_OUT_PORT		PORT_PA08
#define SW2B_GP_OUT_PIN			PIN_PA09
#define SW2B_GP_OUT_PORT		PORT_PA09
#define SW3B_GP_OUT_PIN			PIN_PA10
#define SW3B_GP_OUT_PORT		PORT_PA10
#define SW4B_GP_OUT_PIN			PIN_PA11
#define SW4B_GP_OUT_PORT		PORT_PA11

// Accessory power enable
#define ACCYEN_GP_OUT_PIN		PIN_PA23
#define ACCYEN_GP_OUT_PORT		PORT_PA23

// ATWINC1500 chip
#define WINC_RESET_GP_OUT_PIN	PIN_PA27
#define WINC_RESET_GP_OUT_PORT	PORT_PA27
#define WINC_CHIPEN_GP_OUT_PIN	PIN_PA28
#define WINC_CHIPEN_GP_OUT_PORT	PORT_PA28
#define WINC_WAKE_GP_OUT_PIN	PIN_PB08
#define WINC_WAKE_GP_OUT_PORT	PORT_PB08
#define WINC_SPI_MOSI_PIN		PIN_PA12
#define WINC_SPI_MOSI_PORT		PORT_PA12
#define WINC_SPI_SCK_PIN		PIN_PA13
#define WINC_SPI_SCK_PORT		PORT_PA13
#define WINC_SPI_CS_PIN			PIN_PA14
#define WINC_SPI_CS_PORT		PORT_PA14
#define WINC_SPI_MISO_PIN		PIN_PA15
#define WINC_SPI_MISO_PORT		PORT_PA15
#define WINC_SPI_INT_PIN		PIN_PB09
#define WINC_SPI_INT_PORT		PORT_PB09
#define WINC_GCLK_RTC_PIN		PIN_PA22
#define WINC_GCLK_RTC_PORT		PORT_PA22

#endif // CONF_BOARD_H
