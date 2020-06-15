/**
 * Copyright (c) 2015 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINTRAUMNGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

 #ifndef AD_SPI_H_
#define AD_SPI_H_


#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "app_timer.h"
#include "nrf_drv_timer.h"
#include "traumschreiber_service.h"


//SPI pins for the three ADs
#define SPI_0_DRDY 17 // connected to \DRDY-signal from AD0
#define SPI_0_SCLK  4 // connected to SCLK-signal from AD0
#define SPI_0_CS    6 // connected to CS-signal from AD0
#define SPI_0_SDI  15 // connected to SDI-signal from AD0
#define SPI_0_SDO  16 // connected to SDO-signal from AD0

//#define SPI_0_DRDY 14 // connected to \DRDY-signal from AD1
//#define SPI_0_SCLK  19 // connected to SCLK-signal from AD1
//#define SPI_0_CS    8 // connected to CS-signal from AD1
//#define SPI_0_SDI  7 // connected to SDI-signal from AD1
//#define SPI_0_SDO  5 // connected to SDO-signal from AD1

//#define SPI_0_DRDY 25 // connected to \DRDY-signal from AD2
//#define SPI_0_SCLK  22 // connected to SCLK-signal from AD2
//#define SPI_0_CS    24 // connected to CS-signal from AD2
//#define SPI_0_SDI  28 // connected to SDI-signal from AD2
//#define SPI_0_SDO  29 // connected to SDO-signal from AD2

#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */


//SPI Constants and buffers
#define SPI_READ_SIGNAL   {0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} //default read signal for 32 Bytes
#define SPI_READ_CHANNEL_NUMBER   8
#define SPI_READ_PER_CHANNEL      4
#define SPI_READ_LENGTH   SPI_READ_CHANNEL_NUMBER*SPI_READ_PER_CHANNEL
static uint8_t       m_tx_buf[] = SPI_READ_SIGNAL;           /**< TX buffer. */
static uint8_t       m_rx_buf[SPI_READ_LENGTH + 1];    /**< RX buffer. */
static const uint8_t m_length  =  sizeof(m_tx_buf);        /**< Transfer length. */
static const uint8_t m_rlength =  sizeof(m_rx_buf);        /**< RX Buffer length. */

#define ADREG_GENERAL_USER_CONFIG_1 {0x11, 0x24} //write signal for register with standart settings
#define ADREG_GENERAL_USER_CONFIG_2 {0x12, 0x09} //write signal for register with standart settings
#define ADREG_GENERAL_USER_CONFIG_3 {0x13, 0x80} //write signal for register with standart settings
#define ADREG_GENERAL_USER_CONFIG_3_BYTE_OR 0x10 //Byte for or operation with GUC3-Register to set Bit#4

#define ADREG_DECIMATION_RATE_N     {0x60, 0x00, 0x61, 0x80} //write signal for register with standart settings
#define ADREG_DECIMATION_RATE_N_MOD {0x60, 0x08, 0x61, 0x00} //write signal for register, ODR = 250Hz (should, but actually seems to be 500Hz)
#define ADREG_DECIMATION_RATE_N_MAD {0x60, 0x0F, 0x61, 0xFF} //write signal for register, ODR = 250Hz (should not work, but seems to do 250Hz)

#define ADREG_SRC_MODE_UPDATE_CLEAR {0x64, 0x00} //write signal for register with standart settings
#define ADREG_SRC_MODE_UPDATE_SET   {0x64, 0x01} //write signal for register with update instruction


static uint8_t triggerSkipCounter = 0; //counter used to skip trigger events so log buffer does not overflow
static uint8_t triggerSkipCounterMax = 0; // how many events are skipped (+1)

static uint16_t packetSkipCounter = 0;


//SPI data recieve ring buffer
#define SPI_DATA_BUFFER_LENGTH      SPI_READ_LENGTH*8
#define SPI_DATA_BUFFER_WRITE_SIZE  SPI_READ_LENGTH
#define SPI_DATA_BUFFER_READ_SIZE   TRAUM_SERVICE_VALUE_LENGTH
//static uint8_t  spi_read_buf[SPI_DATA_BUFFER_LENGTH];    /**< RX buffer. */
//static const uint16_t  srb_buffer_length    = SPI_DATA_BUFFER_LENGTH; //needed because somehow constants can't be used in calculations...
//static uint16_t        srb_write_position  = 0;
//static const uint16_t  srb_packet_size     = SPI_READ_LENGTH; //needed because somehow constants can't be used in calculations...
//static uint16_t        srb_read_position   = 0;
//static uint16_t        srb_capacity_used   = 0;

//BLE data send ring buffer
#define SPI_BLE_BUFFER_LENGTH      SPI_DATA_BUFFER_READ_SIZE*12
static uint8_t  spi_send_buf[SPI_BLE_BUFFER_LENGTH];    /**< TX buffer. */
static const uint16_t  stb_buffer_length    = SPI_BLE_BUFFER_LENGTH; //needed because somehow constants can't be used in calculations...
static uint16_t        stb_write_position  = 0;
static const uint16_t  stb_packet_size     = SPI_DATA_BUFFER_READ_SIZE; //needed because somehow constants can't be used in calculations...
static uint16_t        stb_read_position   = 0;
static uint16_t        stb_capacity_used   = 0;

//compression, old, not yet cleaned up
static int32_t  spi_last_abs_values[SPI_READ_CHANNEL_NUMBER];
static int32_t  spi_new_diff_values[SPI_READ_CHANNEL_NUMBER];
static uint8_t  recieved_packets_counter      = 0;
static uint8_t  recieved_packets_counter_max  = 16;
static const uint16_t spi_min_bits_per_channel      = 5;
static const uint16_t spi_max_bits_per_channel      = 12;
static int16_t  spi_max_difval_per_channel    = (1 << (spi_max_bits_per_channel-1)) - 1; //2**spi_max_bits_per_channel
static int16_t  spi_min_difval_per_channel    = -(1 << (spi_max_bits_per_channel-1));
static uint8_t  spi_overflow_send_buf[SPI_DATA_BUFFER_READ_SIZE*2];    /**< overflow buffer. */
static uint16_t sob_capacity_used   = 0; //number of bits
static uint16_t sob_capacity_max    = SPI_DATA_BUFFER_READ_SIZE*2*8; //number of bits
static uint16_t sob_read_position   = 0; //number of bits
#define SPI_COMP_CHANNELS_PER_FULL_PACKAGE          6
#define SPI_COMP_BYTES_PER_FULL_PACKAGE_CHANNELS    3


//Debug Data Generation
#define SPI_DATA_GEN_FLAG   0
#define SPI_DATA_GEN_BASE   {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00} //default read signal for 32 Bytes
static uint8_t  spi_data_gen_buf[8*4] = SPI_DATA_GEN_BASE;


// timer event handler
static void spi_timer_timeout_handler(void * p_context);
static void spi_data_generation_timeout_handler(void * p_context);

/**
 * @brief Handler for timer events.
 */
void timer_led_event_handler(nrf_timer_event_t event_type, void* p_context);

void spi_transfer_read(void);

/**
 * @brief GPIO user event handler.
 * @param event
 * triggered when conversion of AD is complete and data can be accessed vie SPI
 * initiates SPI-read
 */
void spi_trigger_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);


/**
 * @brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output,
 * and configures GPIOTE to give an interrupt on pin change.
 */
static void gpio_init(void);

/**
 * @brief SPI user event handler.
 * @param event
 * called when SPI-transfer is completed
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context);

void spi_ble_connect(ble_traum_t * p_traum_service);
void spi_ble_disconnect();
void spi_ble_notify(uint16_t notify);

bool spi_new_data(void);

uint8_t* spi_read_data(void);
void spi_data_sent(void);
void spi_ble_sent(uint8_t count);


void spi_init(void);


#endif  /* _ AD_SPI_H__ */